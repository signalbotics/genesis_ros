"""Proximity publisher -- sensor_msgs/Range per ProximitySensor.

Owned by Group 5. Genesis ``ProximitySensor`` exposes ``max_range`` (no
``min_range``/``fov``); we publish ``sensor_msgs/Range`` with ``min_range=0``
and a configurable ``fov`` (``cfg['fov']`` or 0.1 rad default).
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from std_msgs.msg import Header  # noqa: F401
    from sensor_msgs.msg import Range
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - ROS 2 not installed
    _ROS_AVAILABLE = False

import numpy as np  # noqa: F401  (kept for parity with template)

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import SENSOR_QOS
from genesis_ros import conversions as conv

try:  # pragma: no cover
    from genesis.engine.sensors.proximity import ProximitySensor as _ProximitySensor
except Exception:  # pragma: no cover
    _ProximitySensor = None


def _is_proximity_sensor(sensor) -> bool:
    if _ProximitySensor is not None and isinstance(sensor, _ProximitySensor):
        return True
    return type(sensor).__name__ == "ProximitySensor"


class ProximityPublisher(GenesisPublisher):
    """Publish ``sensor_msgs/Range`` for every ProximitySensor on record."""

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "ProximityPublisher requires the ROS 2 Python stack"
                " (rclpy, sensor_msgs)."
            )
        super().__init__(node, scene, registry, cfg)
        self._env_idx = int(self.cfg.get("env_idx", 0))
        self._fov = float(self.cfg.get("fov", 0.1))
        self._pubs = {}

    def _iter_proximity_sensors(self):
        for record_name, record in self.registry.items():
            sensors = getattr(record, "sensors", None)
            if not sensors:
                continue
            try:
                iterator = sensors.items()
            except AttributeError:
                iterator = (
                    (getattr(s, "name", "prox%d" % i), s)
                    for i, s in enumerate(sensors)
                )
            for sensor_name, sensor in iterator:
                if _is_proximity_sensor(sensor):
                    yield record_name, record, sensor_name, sensor

    def _get_pub(self, record_name, sensor_name):
        key = (record_name, sensor_name)
        pub = self._pubs.get(key)
        if pub is None:
            topic = "/" + record_name + "/" + sensor_name
            pub = self.node.create_publisher(Range, topic, SENSOR_QOS)
            self._pubs[key] = pub
        return pub

    def step(self, sim_time) -> None:
        for record_name, record, sensor_name, sensor in self._iter_proximity_sensors():
            try:
                distances = sensor.read()
            except Exception as exc:
                self.node.get_logger().warning(
                    "ProximityPublisher: " + record_name + "/" + sensor_name
                    + " read() failed: " + repr(exc)
                )
                continue

            arr = conv._as_numpy(
                conv.select_env(distances, self._env_idx)
            ).reshape(-1)
            if arr.size == 0:
                continue
            d = float(arr[0])

            opts = getattr(sensor, "_options", None)
            max_range = getattr(opts, "max_range", None)
            if max_range is None:
                max_range = getattr(sensor, "max_range", 10.0)
            try:
                max_range = float(max_range)
            except Exception:
                max_range = 10.0

            msg = Range()
            msg.header.stamp = sim_time
            msg.header.frame_id = record_name + "/" + sensor_name
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = self._fov
            msg.min_range = 0.0
            msg.max_range = max_range
            msg.range = d

            self._get_pub(record_name, sensor_name).publish(msg)


__all__ = ["ProximityPublisher"]
