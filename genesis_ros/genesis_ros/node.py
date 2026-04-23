"""Core runtime for the Genesis<->ROS 2 bridge.

Owns:

* the :class:`GenesisPublisher` and :class:`GenesisSubscriber` base classes
  that every plugin implements,
* :class:`GenesisRosBridge` -- the orchestrator that owns the main-thread
  step loop, the background ``rclpy`` executor thread, the entity registry,
  sim time, and the RTF tracker.

The contract here is frozen; other groups import these names. See
``ROS2_INTEGRATION_PLAN.md`` -> "Base-class contract".
"""
from __future__ import annotations

import signal
import threading
import time
import types
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Dict, List, Optional

if TYPE_CHECKING:  # pragma: no cover - imports used only for type hints
    import builtin_interfaces.msg  # noqa: F401
    import rclpy.node  # noqa: F401
    import genesis as gs  # noqa: F401

from .conversions import ros_time_from_ns


class GenesisPublisher(ABC):
    """Base class for anything that reads Genesis state and publishes to ROS.

    Concrete subclasses implement :meth:`step`, which is called on the main
    thread right after :meth:`genesis.Scene.step` so they always observe a
    consistent simulation snapshot. Publishing itself is thread safe in rclpy.
    """

    def __init__(self, node, scene, registry, cfg: Optional[dict] = None):
        self.node = node
        self.scene = scene
        self.registry = registry
        self.cfg: dict = dict(cfg or {})

    @abstractmethod
    def step(self, sim_time) -> None:
        """Publish one tick worth of data.

        ``sim_time`` is a builtin_interfaces.msg.Time stamped with the
        current ``scene.t * scene.dt``.
        """


class GenesisSubscriber(ABC):
    """Base class for anything that ingests a ROS topic / service / action
    and mutates the simulator.

    Subscribers stage incoming data from the rclpy executor thread;
    :meth:`apply` is invoked on the main thread by the bridge before
    :meth:`scene.step` so commands take effect on the next integration step.
    """

    def __init__(self, node, scene, registry, cfg: Optional[dict] = None):
        self.node = node
        self.scene = scene
        self.registry = registry
        self.cfg: dict = dict(cfg or {})

    @abstractmethod
    def apply(self, scene) -> None:
        """Flush staged ROS commands into scene before the next step."""



class GenesisRosBridge:
    """Owns the Genesis main-thread step loop and a background rclpy executor."""

    def __init__(
        self,
        scene,
        *,
        node_name: str = "genesis_bridge",
        env_idx: int = 0,
        use_sim_time: bool = True,
        rtf_target: Optional[float] = None,
    ) -> None:
        # Lazy import -- ``import genesis_ros.node`` must not fail on boxes
        # without the ROS 2 stack installed (unit tests / CI without rclpy).
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        if not rclpy.ok():
            rclpy.init(args=None)

        self._rclpy = rclpy
        self.scene = scene
        self.env_idx = int(env_idx)
        self.node_name = node_name
        self.use_sim_time = bool(use_sim_time)

        self.node: Node = Node(node_name)
        try:
            self.node.declare_parameter("use_sim_time", self.use_sim_time)
        except Exception:
            # Already declared -- just overwrite.
            self.node.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, self.use_sim_time)]
            )

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._executor_thread = threading.Thread(
            target=self._run_executor,
            name=node_name + "-executor",
            daemon=True,
        )

        self._lock = threading.Lock()
        self._publishers: List[GenesisPublisher] = []
        self._subscribers: List[GenesisSubscriber] = []
        self._services: List[Any] = []
        self._entity_stubs: Dict[str, Any] = {}

        # Registry placeholder -- upgraded to Group 2's real EntityRegistry
        # the first time :meth:`register_entity` runs.
        self.registry: Any = _LazyEntityRegistry(self)

        # RTF tracking (Group 2 publishes the value on /genesis/rtf).
        self.rtf: float = 1.0
        self._rtf_alpha: float = 0.1
        self._last_wall_time: Optional[float] = None

        # RTF throttle. None (or <=0) means "run as fast as possible". A
        # positive float N caps wall-clock so ``wall_dt_per_tick >= scene.dt/N``
        # -- i.e. the sim advances no faster than N times real time.
        # set_rtf_target() mutates this from the rclpy executor thread;
        # python float assignment is atomic under the GIL so we do not lock.
        self.rtf_target: Optional[float] = (
            float(rtf_target) if rtf_target and rtf_target > 0 else None
        )

        # Cooperative pause / step credits. services/sim_control.py flips
        # these from its service callbacks on the rclpy executor thread;
        # _run_one_iteration observes them on the main thread to decide
        # whether to advance scene.step(). Kept as plain attributes because
        # bool / int assignment is atomic under the GIL.
        self._paused: bool = False
        self._step_remaining: int = 0

        self._shutdown: bool = False
        self._sigint_installed: bool = False

    # ---------------------------------------------------------------- registry
    def register_entity(
        self,
        entity,
        *,
        name: str,
        urdf_xml: Optional[str] = None,
        is_mobile_base: bool = False,
        **kw: Any,
    ) -> Any:
        """Register ``entity`` under ``name``.

        Always stores a :class:`types.SimpleNamespace` stub so callers can look
        up ``bridge._entity_stubs[name]`` even when Group 2's real
        ``EntityRegistry`` is not yet importable. If it *is* importable we
        upgrade :attr:`self.registry` in place and replay the stubs into it.
        """
        stub = types.SimpleNamespace(
            name=name,
            gs_entity=entity,
            urdf_xml=urdf_xml,
            is_mobile_base=is_mobile_base,
            extras=dict(kw),
        )
        with self._lock:
            self._entity_stubs[name] = stub

        try:
            from .entity_registry import EntityRegistry  # type: ignore
        except Exception:
            return stub

        if not isinstance(self.registry, EntityRegistry):
            real = EntityRegistry(bridge=self)
            for prior_name, prior_stub in self._entity_stubs.items():
                try:
                    real.register(
                        prior_stub.gs_entity,
                        name=prior_name,
                        urdf_xml=prior_stub.urdf_xml,
                        is_mobile_base=prior_stub.is_mobile_base,
                        **prior_stub.extras,
                    )
                except Exception:
                    pass
            self.registry = real
            return real.get(name) if hasattr(real, "get") else stub

        try:
            return self.registry.register(
                entity,
                name=name,
                urdf_xml=urdf_xml,
                is_mobile_base=is_mobile_base,
                **kw,
            )
        except Exception:
            return stub

    # ----------------------------------------------------------- plugin mgmt
    def register_publisher(self, p: GenesisPublisher) -> None:
        with self._lock:
            self._publishers.append(p)

    def register_subscriber(self, s: GenesisSubscriber) -> None:
        with self._lock:
            self._subscribers.append(s)

    def register_service(self, cb) -> None:
        """Hold a reference to a service / action server so it stays alive
        for the lifetime of the bridge."""
        with self._lock:
            self._services.append(cb)

    # ---------------------------------------------------------- RTF control
    def set_rtf_target(self, value: Optional[float]) -> None:
        """Set the real-time-factor cap. ``None`` / ``<= 0`` disables it."""
        if value is None:
            self.rtf_target = None
            return
        try:
            v = float(value)
        except (TypeError, ValueError):
            return
        self.rtf_target = v if v > 0.0 else None

    # -------------------------------------------------------------- sim time
    def current_sim_time(self):
        """Return the current simulator time as builtin_interfaces.msg.Time."""
        nanoseconds = int(round(float(self.scene.t) * float(self.scene.dt) * 1e9))
        return ros_time_from_ns(nanoseconds)

    # -------------------------------------------------------------- main loop
    def spin(self) -> None:
        """Run the bridge until shutdown."""
        self._install_sigint_handler()
        if not self._executor_thread.is_alive():
            self._executor_thread.start()

        try:
            while self._rclpy.ok() and not self._shutdown:
                self._run_one_iteration()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Idempotent graceful teardown."""
        already = self._shutdown and not self._executor_thread.is_alive()
        self._shutdown = True
        if already:
            return
        try:
            self._executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass

    # ---------------------------------------------------------- internal glue
    def _run_executor(self) -> None:
        try:
            self._executor.spin()
        except Exception:
            # Don't crash the main thread -- the main loop checks rclpy.ok().
            pass

    def _install_sigint_handler(self) -> None:
        if self._sigint_installed:
            return
        try:
            signal.signal(signal.SIGINT, self._handle_sigint)
            self._sigint_installed = True
        except ValueError:
            # Not on the interpreter's main thread. Let the caller's handler win.
            pass

    def _handle_sigint(self, signum, frame) -> None:  # noqa: D401
        self._shutdown = True

    def _run_one_iteration(self) -> None:
        """Execute exactly one simulator tick. Exposed for tests."""
        wall_start = time.perf_counter()

        # Pause gate. If paused and no step credits queued, sleep briefly so
        # we do not busy-loop the main thread, then return without advancing
        # sim time -- /clock stays frozen at the current stamp, which is
        # exactly what clients expect from /genesis/pause.
        if self._paused and self._step_remaining <= 0:
            time.sleep(0.005)
            return
        if self._paused and self._step_remaining > 0:
            # Consume exactly one step credit per iteration.
            self._step_remaining -= 1

        with self._lock:
            subs = list(self._subscribers)
            pubs = list(self._publishers)
        for sub in subs:
            try:
                sub.apply(self.scene)
            except Exception as exc:
                self.node.get_logger().warning(
                    "subscriber " + type(sub).__name__
                    + " apply() failed: " + repr(exc)
                )

        self.scene.step()

        stamp = self.current_sim_time()
        for pub in pubs:
            try:
                pub.step(stamp)
            except Exception as exc:
                self.node.get_logger().warning(
                    "publisher " + type(pub).__name__
                    + " step() failed: " + repr(exc)
                )

        wall_end = time.perf_counter()
        wall_dt = wall_end - wall_start
        sim_dt = float(self.scene.dt)

        # RTF throttle. Sleep just enough so this iteration's wall time
        # matches (scene.dt / rtf_target). If the sim already ran slower
        # than that budget, the sleep is 0.
        target = self.rtf_target
        if target and target > 0.0 and sim_dt > 0.0:
            budget = sim_dt / target
            leftover = budget - wall_dt
            if leftover > 0.0:
                time.sleep(leftover)
                wall_end = time.perf_counter()
                wall_dt = wall_end - wall_start

        if sim_dt > 0.0 and wall_dt > 0.0:
            sample = wall_dt / sim_dt
            instantaneous = 1.0 / sample if sample > 0 else 1.0
            self.rtf = (
                (1.0 - self._rtf_alpha) * self.rtf
                + self._rtf_alpha * instantaneous
            )
        self._last_wall_time = wall_end


class _LazyEntityRegistry:
    """Minimal dict-like stand-in used until Group 2 ships EntityRegistry."""

    def __init__(self, bridge: "GenesisRosBridge") -> None:
        self._bridge = bridge

    def __getitem__(self, name: str) -> Any:
        return self._bridge._entity_stubs[name]

    def __contains__(self, name: object) -> bool:
        return name in self._bridge._entity_stubs

    def __iter__(self):
        return iter(self._bridge._entity_stubs)

    def __len__(self) -> int:
        return len(self._bridge._entity_stubs)

    def get(self, name: str, default: Any = None) -> Any:
        return self._bridge._entity_stubs.get(name, default)

    def items(self):
        return self._bridge._entity_stubs.items()

    def values(self):
        return self._bridge._entity_stubs.values()

    def keys(self):
        return self._bridge._entity_stubs.keys()


__all__ = [
    "GenesisPublisher",
    "GenesisSubscriber",
    "GenesisRosBridge",
]
