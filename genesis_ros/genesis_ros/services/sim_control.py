"""``SimControlService`` -- pause / resume / step / reset the Genesis main loop.

This module registers four services that external ROS 2 clients use to drive
the ``GenesisRosBridge`` step loop:

* ``/genesis/pause``   -- stop advancing ``scene.step()`` ticks.
* ``/genesis/resume``  -- continue ticking.
* ``/genesis/step``    -- advance ``n_steps`` ticks while paused (cooperative).
* ``/genesis/reset``   -- call ``scene.reset()`` if the scene exposes one.

We *prefer* ``simulation_interfaces`` (REP-2015) service types so clients and
rqt tooling light up for free, and fall back to ``std_srvs/Trigger`` +
``std_srvs/SetBool`` when that package is absent at runtime.

NOTE: relies on ``bridge._paused`` and ``bridge._step_remaining`` which
Group 1 may or may not expose yet. If absent, we lazy-attach them here with
``setattr(bridge, '_paused', False)`` fallbacks. A grep of
``genesis_ros/node.py`` at time of writing shows neither name present, so the
lazy-attach path is exercised in the current tree.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from rclpy.node import Node  # noqa: F401
    from std_msgs.msg import Float64 as _Float64
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    _Float64 = None  # type: ignore[assignment]

import threading

from genesis_ros.node import GenesisSubscriber  # noqa: F401  -- shared pattern
from genesis_ros.qos import STATE_QOS  # noqa: F401
from genesis_ros import conversions as conv  # noqa: F401


# ------------------------------------------------------------- type resolution
def _resolve_service_types():
    """Return ``(pause_t, resume_t, step_t, reset_t, backend)``.

    ``backend`` is either ``"simulation_interfaces"`` or ``"std_srvs"``.
    """
    try:
        # REP-2015 community package.
        from simulation_interfaces.srv import StepSimulation  # type: ignore
        try:
            from simulation_interfaces.srv import ResetSimulation  # type: ignore
        except ImportError:
            from std_srvs.srv import Trigger as ResetSimulation  # type: ignore
        from std_srvs.srv import Trigger as _Trigger
        return _Trigger, _Trigger, StepSimulation, ResetSimulation, "simulation_interfaces"
    except ImportError:
        from std_srvs.srv import Trigger
        return Trigger, Trigger, Trigger, Trigger, "std_srvs"


class SimControlService:
    """Holds the four simulation-control service handles for a bridge.

    Unlike publishers / subscribers, this class does not participate in the
    main-thread flush cycle -- each service callback simply flips a flag on
    the bridge that ``GenesisRosBridge`` observes before it calls
    ``scene.step()``.
    """

    def __init__(self, node, scene, bridge):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "SimControlService requires rclpy -- the ROS 2 runtime was "
                "not importable."
            )
        self.node = node
        self.scene = scene
        self.bridge = bridge

        # Lazy-attach the cooperative bits Group 1's node.py does not define
        # yet. Once it does, ``setattr`` with an existing attribute is a
        # no-op at the observable level (the attribute already exists).
        if not hasattr(bridge, "_paused"):
            setattr(bridge, "_paused", False)
        if not hasattr(bridge, "_step_remaining"):
            setattr(bridge, "_step_remaining", 0)

        self._lock = threading.Lock()

        pause_t, resume_t, step_t, reset_t, backend = _resolve_service_types()
        self._backend = backend

        self._pause_srv = node.create_service(
            pause_t, "/genesis/pause", self._on_pause
        )
        self._resume_srv = node.create_service(
            resume_t, "/genesis/resume", self._on_resume
        )
        self._step_srv = node.create_service(
            step_t, "/genesis/step", self._on_step
        )
        self._reset_srv = node.create_service(
            reset_t, "/genesis/reset", self._on_reset
        )
        # /genesis/set_rtf: publish a std_msgs/Float64 to cap the real-time
        # factor. Non-positive values disable the cap (run as fast as
        # possible). Using a topic rather than a service matches the
        # existing /genesis/rtf Float64 state publisher for symmetry.
        if _Float64 is not None:
            self._set_rtf_sub = node.create_subscription(
                _Float64, "/genesis/set_rtf", self._on_set_rtf, 10
            )

    # ---------------------------------------------------------- helpers
    @staticmethod
    def _set_response(response, *, success=True, message=""):
        """Populate success / message fields if the response carries them."""
        if hasattr(response, "success"):
            try:
                response.success = bool(success)
            except Exception:
                pass
        if hasattr(response, "message"):
            try:
                response.message = str(message)
            except Exception:
                pass
        return response

    # ---------------------------------------------------------- callbacks
    def _on_pause(self, request, response):
        with self._lock:
            setattr(self.bridge, "_paused", True)
        return self._set_response(response, success=True, message="paused")

    def _on_resume(self, request, response):
        with self._lock:
            setattr(self.bridge, "_paused", False)
        return self._set_response(response, success=True, message="resumed")

    def _on_step(self, request, response):
        # Trigger fallback carries no fields; StepSimulation has ``n_steps``
        # (or ``steps`` in older revisions). Default to 1.
        n = getattr(request, "n_steps", None)
        if n is None:
            n = getattr(request, "steps", None)
        try:
            n_int = int(n) if n is not None else 1
        except Exception:
            n_int = 1
        if n_int < 1:
            n_int = 1

        with self._lock:
            current = getattr(self.bridge, "_step_remaining", 0) or 0
            setattr(self.bridge, "_step_remaining", int(current) + n_int)
        return self._set_response(
            response,
            success=True,
            message="queued " + str(n_int) + " step(s)",
        )

    def _on_set_rtf(self, msg) -> None:
        value = float(getattr(msg, "data", 0.0) or 0.0)
        setter = getattr(self.bridge, "set_rtf_target", None)
        if callable(setter):
            setter(value if value > 0.0 else None)
        else:
            setattr(self.bridge, "rtf_target", value if value > 0.0 else None)

    def _on_reset(self, request, response):
        scene = self.scene
        if hasattr(scene, "reset"):
            try:
                scene.reset()
                return self._set_response(response, success=True, message="reset")
            except Exception as exc:
                self._warn("SimControlService reset failed: " + repr(exc))
                return self._set_response(
                    response, success=False, message="reset failed: " + repr(exc)
                )
        self._warn(
            "SimControlService: scene has no reset() method -- ignoring "
            "/genesis/reset request."
        )
        return self._set_response(
            response, success=False, message="scene.reset() not available"
        )

    # ---------------------------------------------------------- diagnostics
    def _warn(self, message):
        logger = getattr(self.node, "get_logger", None)
        if callable(logger):
            try:
                logger().warning(message)
                return
            except Exception:
                pass


__all__ = ["SimControlService"]
