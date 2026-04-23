"""Clock and RTF publishers for the Genesis<->ROS 2 bridge.

Two classes live here:

* :class:`ClockPublisher` -- publishes ``rosgraph_msgs/msg/Clock`` on
  ``/clock`` so downstream ROS 2 nodes with ``use_sim_time=True`` drive
  their timers off simulator time. Supports counter-based decimation so
  ``/clock`` does not need to be published at every solver tick.
* :class:`RealTimeFactorPublisher` -- publishes a throttled
  ``std_msgs/msg/Float64`` on ``/genesis/rtf`` sourced from the bridge's
  RTF tracker (or any caller-supplied ``rtf_provider`` callable).

Both import ``rclpy`` lazily so the module remains importable on machines
without the ROS 2 stack installed (e.g. unit tests / CI).
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised on ROS-less boxes
    _ROS_AVAILABLE = False

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import CLOCK_QOS


class ClockPublisher(GenesisPublisher):
    """Publish simulator time on ``/clock`` at a configurable decimation."""

    def __init__(self, node, scene, registry, cfg=None):
        super().__init__(node, scene, registry, cfg)
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "ClockPublisher requires the ROS 2 Python stack (rclpy, "
                "rosgraph_msgs) but neither is importable."
            )
        # Publish every N-th tick; 1 = every tick.
        decimation = int(self.cfg.get("clock_decimation", 1) or 1)
        self._decimation: int = max(1, decimation)
        self._tick: int = 0
        self._pub = self.node.create_publisher(Clock, "/clock", CLOCK_QOS)

    def step(self, sim_time) -> None:  # noqa: D401
        self._tick += 1
        if (self._tick % self._decimation) != 0:
            return
        msg = Clock()
        msg.clock = sim_time
        self._pub.publish(msg)


class RealTimeFactorPublisher(GenesisPublisher):
    """Publish the current real-time factor on ``/genesis/rtf``.

    The value comes from ``cfg['rtf_provider']`` (a zero-argument callable
    returning a float) when provided, otherwise defaults to a constant 1.0
    so the module is usable in isolation. The publisher is throttled to
    ``cfg['rate_hz']`` (default 1 Hz) using wall-free sim-time stamping:
    the throttle looks at successive ``sim_time`` stamps so it behaves the
    same at any RTF.
    """

    def __init__(self, node, scene, registry, cfg=None):
        super().__init__(node, scene, registry, cfg)
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "RealTimeFactorPublisher requires rclpy / std_msgs but "
                "neither is importable."
            )
        self._rate_hz: float = float(self.cfg.get("rate_hz", 1.0) or 1.0)
        if self._rate_hz <= 0.0:
            self._rate_hz = 1.0
        self._period_ns: int = int(round(1e9 / self._rate_hz))
        provider = self.cfg.get("rtf_provider")
        self._provider = provider if callable(provider) else (lambda: 1.0)
        self._last_stamp_ns: int = -1

        # Default reliable, depth-10 QoS per the integration plan.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.node.create_publisher(Float64, "/genesis/rtf", qos)

    def step(self, sim_time) -> None:  # noqa: D401
        # ``sim_time`` is builtin_interfaces.msg.Time -- normalise to ns.
        now_ns = int(sim_time.sec) * 1_000_000_000 + int(sim_time.nanosec)
        if self._last_stamp_ns >= 0 and (now_ns - self._last_stamp_ns) < self._period_ns:
            return
        self._last_stamp_ns = now_ns

        try:
            value = float(self._provider())
        except Exception:
            value = 1.0
        msg = Float64()
        msg.data = value
        self._pub.publish(msg)


__all__ = [
    "ClockPublisher",
    "RealTimeFactorPublisher",
]
