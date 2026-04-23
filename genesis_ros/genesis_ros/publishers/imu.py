"""IMU publisher -- sensor_msgs/Imu (+ optional MagneticField).

Owned by Group 5. Bridges Genesis ``IMUSensor`` readings onto ROS 2.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from std_msgs.msg import Header  # noqa: F401
    from sensor_msgs.msg import Imu, MagneticField
    from geometry_msgs.msg import Vector3
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - ROS 2 not installed
    _ROS_AVAILABLE = False

import numpy as np

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import SENSOR_QOS
from genesis_ros import conversions as conv

try:  # pragma: no cover - only importable when Genesis is installed
    from genesis.engine.sensors.imu import IMUSensor as _IMUSensor
except Exception:  # pragma: no cover
    _IMUSensor = None


_DEFAULT_COV_DIAG = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]


def _is_imu_sensor(sensor) -> bool:
    if _IMUSensor is not None and isinstance(sensor, _IMUSensor):
        return True
    return type(sensor).__name__ == "IMUSensor"


def _diag_cov(noise):
    if noise is None:
        return list(_DEFAULT_COV_DIAG)
    try:
        arr = conv._as_numpy(noise).reshape(-1)
    except Exception:
        return list(_DEFAULT_COV_DIAG)
    if arr.size < 3:
        return list(_DEFAULT_COV_DIAG)
    return [
        float(arr[0]) ** 2, 0.0, 0.0,
        0.0, float(arr[1]) ** 2, 0.0,
        0.0, 0.0, float(arr[2]) ** 2,
    ]


def _any_nonzero(attr) -> bool:
    if attr is None:
        return False
    try:
        arr = conv._as_numpy(attr).reshape(-1)
    except Exception:
        return False
    return bool(np.any(arr != 0.0))


class ImuPublisher(GenesisPublisher):
    """Publish ``sensor_msgs/Imu`` for every IMUSensor attached to a record.

    Lazily creates one publisher per sensor the first time it is seen. If
    magnetometer noise/bias is configured -- or ``cfg['publish_mag']`` is
    truthy -- also publishes ``sensor_msgs/MagneticField`` on ``.../mag``.
    """

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "ImuPublisher requires the ROS 2 Python stack (rclpy,"
                " sensor_msgs, geometry_msgs)."
            )
        super().__init__(node, scene, registry, cfg)
        self._env_idx = int(self.cfg.get("env_idx", 0))
        self._force_mag = bool(self.cfg.get("publish_mag", False))
        self._imu_pubs = {}
        self._mag_pubs = {}

    def _iter_imu_sensors(self):
        for record_name, record in self.registry.items():
            sensors = getattr(record, "sensors", None)
            if not sensors:
                continue
            try:
                iterator = sensors.items()
            except AttributeError:
                iterator = (
                    (getattr(s, "name", "imu%d" % i), s)
                    for i, s in enumerate(sensors)
                )
            for sensor_name, sensor in iterator:
                if _is_imu_sensor(sensor):
                    yield record_name, record, sensor_name, sensor

    def _get_imu_pub(self, record_name, sensor_name):
        key = (record_name, sensor_name)
        pub = self._imu_pubs.get(key)
        if pub is None:
            topic = "/" + record_name + "/" + sensor_name
            pub = self.node.create_publisher(Imu, topic, SENSOR_QOS)
            self._imu_pubs[key] = pub
        return pub

    def _get_mag_pub(self, record_name, sensor_name):
        key = (record_name, sensor_name)
        pub = self._mag_pubs.get(key)
        if pub is None:
            topic = "/" + record_name + "/" + sensor_name + "/mag"
            pub = self.node.create_publisher(MagneticField, topic, SENSOR_QOS)
            self._mag_pubs[key] = pub
        return pub

    def step(self, sim_time) -> None:
        for record_name, record, sensor_name, sensor in self._iter_imu_sensors():
            try:
                data = sensor.read()
            except Exception as exc:
                self.node.get_logger().warning(
                    "ImuPublisher: " + record_name + "/" + sensor_name
                    + " read() failed: " + repr(exc)
                )
                continue

            lin = conv._as_numpy(
                conv.select_env(data.lin_acc, self._env_idx)
            ).reshape(-1)
            ang = conv._as_numpy(
                conv.select_env(data.ang_vel, self._env_idx)
            ).reshape(-1)
            mag = getattr(data, "mag", None)

            quat_ros = None
            link = getattr(sensor, "_link", None)
            entity = getattr(record, "entity", None)
            if link is not None and entity is not None:
                try:
                    quats = entity.get_links_quat([int(link.idx_local)])
                    q = conv._as_numpy(
                        conv.select_env(quats, self._env_idx)
                    ).reshape(-1)
                    if q.size >= 4:
                        quat_ros = conv.quat_wxyz_to_ros(q[:4])
                except Exception:
                    quat_ros = None

            frame_id = record_name + "/" + sensor_name
            opts = getattr(sensor, "_options", None)

            imu_msg = Imu()
            imu_msg.header.stamp = sim_time
            imu_msg.header.frame_id = frame_id
            if quat_ros is not None:
                imu_msg.orientation = quat_ros
            imu_msg.orientation_covariance = list(_DEFAULT_COV_DIAG)
            imu_msg.angular_velocity = Vector3(
                x=float(ang[0]), y=float(ang[1]), z=float(ang[2])
            )
            gyro_noise = getattr(sensor, "gyro_noise", None)
            if gyro_noise is None:
                gyro_noise = getattr(opts, "gyro_noise", None)
            imu_msg.angular_velocity_covariance = _diag_cov(gyro_noise)
            imu_msg.linear_acceleration = Vector3(
                x=float(lin[0]), y=float(lin[1]), z=float(lin[2])
            )
            acc_noise = getattr(sensor, "acc_noise", None)
            if acc_noise is None:
                acc_noise = getattr(opts, "acc_noise", None)
            imu_msg.linear_acceleration_covariance = _diag_cov(acc_noise)

            self._get_imu_pub(record_name, sensor_name).publish(imu_msg)

            mag_noise = getattr(sensor, "mag_noise", None)
            if mag_noise is None:
                mag_noise = getattr(opts, "mag_noise", None)
            mag_bias = getattr(sensor, "mag_bias", None)
            if mag_bias is None:
                mag_bias = getattr(opts, "mag_bias", None)
            publish_mag = (
                self._force_mag
                or _any_nonzero(mag_noise)
                or _any_nonzero(mag_bias)
            )
            if publish_mag and mag is not None:
                mag_arr = conv._as_numpy(
                    conv.select_env(mag, self._env_idx)
                ).reshape(-1)
                mag_msg = MagneticField()
                mag_msg.header.stamp = sim_time
                mag_msg.header.frame_id = frame_id
                mag_msg.magnetic_field = Vector3(
                    x=float(mag_arr[0]),
                    y=float(mag_arr[1]),
                    z=float(mag_arr[2]),
                )
                mag_msg.magnetic_field_covariance = _diag_cov(mag_noise)
                self._get_mag_pub(record_name, sensor_name).publish(mag_msg)


__all__ = ["ImuPublisher"]
