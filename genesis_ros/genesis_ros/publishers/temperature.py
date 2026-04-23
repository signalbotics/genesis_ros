"""Temperature publisher -- sensor_msgs/Temperature per TemperatureGridSensor.

Owned by Group 5. Genesis does not expose a point ``Temperature`` sensor;
only ``TemperatureGridSensor`` returning a 3D grid. We publish the grid mean
as ``sensor_msgs/Temperature``.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from std_msgs.msg import Header  # noqa: F401
    from sensor_msgs.msg import Temperature
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - ROS 2 not installed
    _ROS_AVAILABLE = False

import numpy as np

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import SENSOR_QOS
from genesis_ros import conversions as conv

try:  # pragma: no cover
    from genesis.engine.sensors.temperature import (
        TemperatureGridSensor as _TemperatureGridSensor,
    )
except Exception:  # pragma: no cover
    _TemperatureGridSensor = None


def _is_temperature_sensor(sensor) -> bool:
    if (
        _TemperatureGridSensor is not None
        and isinstance(sensor, _TemperatureGridSensor)
    ):
        return True
    return type(sensor).__name__ == "TemperatureGridSensor"


class TemperaturePublisher(GenesisPublisher):
    """Publish ``sensor_msgs/Temperature`` for each TemperatureGridSensor.

    The scalar ``temperature`` field is populated from ``np.mean(grid)``; a
    ``variance`` is inferred from ``sensor._options.noise**2`` when that
    attribute exists.
    """

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "TemperaturePublisher requires the ROS 2 Python stack"
                " (rclpy, sensor_msgs)."
            )
        super().__init__(node, scene, registry, cfg)
        self._env_idx = int(self.cfg.get("env_idx", 0))
        self._pubs = {}

    def _iter_temperature_sensors(self):
        for record_name, record in self.registry.items():
            sensors = getattr(record, "sensors", None)
            if not sensors:
                continue
            try:
                iterator = sensors.items()
            except AttributeError:
                iterator = (
                    (getattr(s, "name", "temp%d" % i), s)
                    for i, s in enumerate(sensors)
                )
            for sensor_name, sensor in iterator:
                if _is_temperature_sensor(sensor):
                    yield record_name, record, sensor_name, sensor

    def _get_pub(self, record_name, sensor_name):
        key = (record_name, sensor_name)
        pub = self._pubs.get(key)
        if pub is None:
            topic = "/" + record_name + "/" + sensor_name
            pub = self.node.create_publisher(Temperature, topic, SENSOR_QOS)
            self._pubs[key] = pub
        return pub

    def step(self, sim_time) -> None:
        for record_name, record, sensor_name, sensor in self._iter_temperature_sensors():
            try:
                grid = sensor.read()
            except Exception as exc:
                self.node.get_logger().warning(
                    "TemperaturePublisher: " + record_name + "/" + sensor_name
                    + " read() failed: " + repr(exc)
                )
                continue

            grid_np = conv._as_numpy(conv.select_env(grid, self._env_idx))
            if grid_np.size == 0:
                continue
            temp = float(np.mean(grid_np))

            try:
                noise = getattr(
                    getattr(sensor, "_options", None), "noise", 0.0
                )
                var = float(noise) ** 2
            except (AttributeError, TypeError):
                var = 0.0

            msg = Temperature()
            msg.header.stamp = sim_time
            msg.header.frame_id = record_name + "/" + sensor_name
            msg.temperature = temp
            msg.variance = var
            self._get_pub(record_name, sensor_name).publish(msg)


__all__ = ["TemperaturePublisher"]
