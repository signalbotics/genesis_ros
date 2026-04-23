"""RaycasterPublisher — maps Genesis raycaster / depth-camera / lidar sensors to
ROS 2 ``sensor_msgs/LaserScan`` / ``PointCloud2`` / ``Range`` topics.

Pattern auto-detection picks the right message type from the sensor's configured
ray pattern. Every sensor also gets a static TF broadcast on ``/tf_static`` via
the queue on ``genesis_ros.publishers.tf.TFPublisher``.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Range
    from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
    from std_msgs.msg import Header
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

import math
import numpy as np

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import SENSOR_QOS
from genesis_ros import conversions as conv


_RAYCASTER_CLASS_NAMES = ("RaycasterSensor", "LidarSensor", "DepthCameraSensor")


def _euler_deg_zyx_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """Euler (ZYX, intrinsic) in degrees -> (w, x, y, z) quaternion."""
    r = math.radians(roll_deg) * 0.5
    p = math.radians(pitch_deg) * 0.5
    y = math.radians(yaw_deg) * 0.5
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_q = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y_q, z)


def _as_np(x):
    if x is None:
        return None
    if hasattr(x, "detach"):  # torch tensor
        return x.detach().cpu().numpy()
    return np.asarray(x)


class RaycasterPublisher(GenesisPublisher):
    """Publishes LaserScan / PointCloud2 / Range per registered Genesis raycaster."""

    def __init__(self, node, scene, registry, cfg):
        if not _ROS_AVAILABLE:
            raise RuntimeError("ROS 2 not available: cannot construct RaycasterPublisher")
        super().__init__(node, scene, registry, cfg)
        self._node = node
        self._registry = registry
        self._cfg = cfg or {}
        self._env_idx = int(self._cfg.get("env_idx", 0))
        # Keyed by (record_name, sensor_name)
        self._pubs: dict = {}
        self._kinds: dict = {}
        self._tf_emitted: set = set()

    # ---- pattern detection --------------------------------------------------

    @staticmethod
    def _detect(sensor) -> str:
        try:
            pat = sensor._options.pattern
        except AttributeError:
            try:
                pat = sensor.options.pattern
            except AttributeError:
                return "pointcloud2"
        cls = type(pat).__name__
        if cls == "SphericalPattern":
            try:
                if len(pat.angles[1]) == 1:
                    return "laserscan"
            except Exception:
                pass
            return "pointcloud2"
        if cls in ("DepthCameraPattern", "GridPattern"):
            return "pointcloud2"
        # Single-ray detection
        try:
            rs = getattr(sensor._options, "return_shape", None)
            if isinstance(rs, (tuple, list)) and math.prod(rs) == 1:
                return "range"
        except Exception:
            pass
        return "pointcloud2"

    # ---- main loop hook -----------------------------------------------------

    def step(self, sim_time) -> None:
        try:
            iter_records = list(self._registry)  # registry yields EntityRecord
        except TypeError:
            # fallback for dict-like
            iter_records = list(getattr(self._registry, "values", lambda: [])())

        for record in iter_records:
            sensors = getattr(record, "sensors", None) or {}
            entity = getattr(record, "entity", None) or getattr(record, "gs_entity", None)
            for sensor_name, sensor in sensors.items():
                if type(sensor).__name__ not in _RAYCASTER_CLASS_NAMES:
                    continue
                key = (record.name, sensor_name)
                kind = self._kinds.setdefault(key, self._detect(sensor))
                pub = self._pubs.get(key)
                if pub is None:
                    pub = self._make_pub(record.name, sensor_name, kind)
                    self._pubs[key] = pub

                try:
                    data = sensor.read()
                except Exception as exc:  # noqa: BLE001
                    self._node.get_logger().warning(f"raycaster {sensor_name} read failed: {exc}")
                    continue

                points, distances = None, None
                if isinstance(data, tuple) and len(data) == 2:
                    points, distances = data
                else:
                    # NamedTuple-ish with .points / .distances
                    points = getattr(data, "points", None)
                    distances = getattr(data, "distances", None)
                    if points is None and distances is None:
                        distances = data
                points = _as_np(points) if points is not None else None
                distances = _as_np(distances) if distances is not None else None
                points = conv.select_env(points, self._env_idx) if points is not None else None
                distances = conv.select_env(distances, self._env_idx) if distances is not None else None

                frame_id = f"{record.name}/{sensor_name}_frame"
                header = Header(stamp=sim_time, frame_id=frame_id)

                if kind == "laserscan":
                    msg = self._make_laserscan(sensor, header, distances)
                    if msg is not None:
                        pub.publish(msg)
                elif kind == "range":
                    msg = self._make_range(sensor, header, distances)
                    if msg is not None:
                        pub.publish(msg)
                else:  # pointcloud2
                    msg = self._make_pointcloud2(header, points)
                    if msg is not None:
                        pub.publish(msg)

                # One-shot static TF
                if key not in self._tf_emitted:
                    try:
                        self._emit_static_tf(record, sensor, sensor_name, sim_time)
                    except Exception as exc:  # noqa: BLE001
                        self._node.get_logger().warning(f"raycaster TF publish failed: {exc}")
                    self._tf_emitted.add(key)

    # ---- publisher factory --------------------------------------------------

    def _make_pub(self, robot: str, sensor_name: str, kind: str):
        topic_tail = {"laserscan": "scan", "pointcloud2": "points", "range": "range"}[kind]
        topic = f"/{robot}/{sensor_name}/{topic_tail}"
        msg_type = {"laserscan": LaserScan, "pointcloud2": PointCloud2, "range": Range}[kind]
        return self._node.create_publisher(msg_type, topic, SENSOR_QOS)

    # ---- message builders ---------------------------------------------------

    def _make_laserscan(self, sensor, header, distances) -> "LaserScan | None":
        if distances is None:
            return None
        try:
            pat = sensor._options.pattern
        except AttributeError:
            return None
        try:
            azim = _as_np(pat.angles[0]).reshape(-1)
            angle_min = float(azim[0])
            angle_max = float(azim[-1])
            if azim.size > 1:
                angle_increment = float((angle_max - angle_min) / (azim.size - 1))
            else:
                angle_increment = 0.0
        except Exception:
            return None

        range_min = float(getattr(sensor._options, "min_range", 0.0) or 0.0)
        range_max = float(getattr(sensor._options, "max_range", 100.0) or 100.0)

        d = _as_np(distances).reshape(-1).astype(np.float32)
        # Sanitise: NaN/Inf -> range_max + 1 (out-of-range convention)
        mask_bad = ~np.isfinite(d)
        d = np.where(mask_bad, range_max + 1.0, d)
        d = np.where(d > range_max, range_max, d)

        msg = LaserScan()
        msg.header = header
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = range_min
        msg.range_max = range_max
        msg.ranges = d.tolist()
        msg.intensities = []
        return msg

    def _make_pointcloud2(self, header, points) -> "PointCloud2 | None":
        if points is None:
            return None
        pts = _as_np(points).reshape(-1, 3).astype(np.float32)
        # Drop non-finite rows
        finite = np.all(np.isfinite(pts), axis=1)
        pts = pts[finite]
        n = int(pts.shape[0])
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.data = pts.tobytes()
        msg.is_dense = True
        return msg

    def _make_range(self, sensor, header, distances) -> "Range | None":
        if distances is None:
            return None
        arr = _as_np(distances).reshape(-1)
        if arr.size == 0:
            return None
        msg = Range()
        msg.header = header
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = float(self._cfg.get("fov", 0.1))
        msg.min_range = float(getattr(sensor._options, "min_range", 0.0) or 0.0)
        msg.max_range = float(getattr(sensor._options, "max_range", 100.0) or 100.0)
        msg.range = float(arr[0])
        return msg

    # ---- static TF ----------------------------------------------------------

    def _emit_static_tf(self, record, sensor, sensor_name, sim_time) -> None:
        try:
            pos_offset = getattr(sensor._options, "pos_offset", (0.0, 0.0, 0.0))
            euler_offset = getattr(sensor._options, "euler_offset", (0.0, 0.0, 0.0))
        except Exception:
            pos_offset = (0.0, 0.0, 0.0)
            euler_offset = (0.0, 0.0, 0.0)
        px, py, pz = [float(v) for v in pos_offset]
        w, x, y, z = _euler_deg_zyx_to_quat(*[float(v) for v in euler_offset])

        entity_idx = getattr(sensor, "entity_idx", 0)
        if entity_idx < 0:
            parent_frame = "world"
        else:
            link = getattr(sensor, "_link", None)
            link_name = getattr(link, "name", None) or record.base_link_name
            parent_frame = f"{record.name}/{link_name}"

        t = TransformStamped()
        t.header.stamp = sim_time
        t.header.frame_id = parent_frame
        t.child_frame_id = f"{record.name}/{sensor_name}_frame"
        t.transform.translation = Vector3(x=px, y=py, z=pz)
        t.transform.rotation = Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))

        try:
            from genesis_ros.publishers.tf import TFPublisher
            TFPublisher.queue_transform(t)
        except (ImportError, AttributeError):
            # TF publisher not registered — drop the transform silently; downstream
            # nodes can fall back to world-anchored cloud/scan frame_ids.
            pass
