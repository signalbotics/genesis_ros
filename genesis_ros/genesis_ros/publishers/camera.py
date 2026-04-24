"""Camera publisher for the Genesis<->ROS 2 bridge.

Owned by Group 4. Iterates the entity registry each step and publishes
sensor_msgs/Image (rgb + optional depth), sensor_msgs/CameraInfo,
optional sensor_msgs/PointCloud2, optional sensor_msgs/Image (mono16
segmentation), and a single /tf transform per camera.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

try:
    import rclpy  # noqa: F401
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import Header  # noqa: F401
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only on boxes without ROS
    _ROS_AVAILABLE = False

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE = CvBridge()
except Exception:  # pragma: no cover - cv_bridge is optional
    _CV_BRIDGE = None

import numpy as np

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import SENSOR_QOS, TF_QOS  # noqa: F401
from genesis_ros import conversions as conv


@dataclass
class _CameraEntry:
    entity_name: str
    cam_name: str
    camera: Any
    parent_link: Any
    frame_id: str
    publish_depth: bool = True
    publish_pointcloud: bool = False
    publish_segmentation: bool = False
    rate_divisor: int = 1
    counter: int = 0
    pubs: Dict[str, Any] = field(default_factory=dict)
    tf_emitted: bool = False


def _matrix_to_quat_wxyz(R):
    """Shepperd-style conversion of a 3x3 rotation matrix to (w, x, y, z).

    Avoids scipy. Branches on the largest diagonal candidate for numerical
    stability at antipodal rotations.
    """
    R = np.asarray(R, dtype=np.float64)
    m00 = float(R[0, 0]); m01 = float(R[0, 1]); m02 = float(R[0, 2])
    m10 = float(R[1, 0]); m11 = float(R[1, 1]); m12 = float(R[1, 2])
    m20 = float(R[2, 0]); m21 = float(R[2, 1]); m22 = float(R[2, 2])
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n > 0.0:
        w /= n; x /= n; y /= n; z /= n
    return (w, x, y, z)




class CameraPublisher(GenesisPublisher):
    """Publish every camera registered in the entity registry."""

    def __init__(self, node, scene, registry, cfg: Optional[dict] = None):
        if not _ROS_AVAILABLE:
            raise RuntimeError("CameraPublisher requires ROS 2 / rclpy to be importable.")
        super().__init__(node, scene, registry, cfg)
        self._entries: List[_CameraEntry] = []
        self._env_idx = int(self.cfg.get("env_idx", 0))
        self._known: Dict[str, _CameraEntry] = {}

    def step(self, sim_time) -> None:
        self._discover()
        for entry in self._entries:
            entry.counter += 1
            if entry.rate_divisor > 1 and (entry.counter % entry.rate_divisor) != 0:
                continue
            try:
                self._publish_entry(entry, sim_time)
            except Exception as exc:  # pragma: no cover
                self.node.get_logger().warning("CameraPublisher step failed for " + entry.entity_name + "/" + entry.cam_name + ": " + repr(exc))

    def _discover(self) -> None:
        registry = self.registry
        if registry is None:
            return
        try:
            records = list(registry.values()) if hasattr(registry, "values") else list(registry)
        except Exception:
            return
        for record in records:
            cameras = getattr(record, "cameras", None)
            if not cameras:
                continue
            entity_name = getattr(record, "name", None) or getattr(record, "entity_name", "robot")
            try:
                items = list(cameras.items())
            except AttributeError:
                continue
            for cam_name, value in items:
                key = str(entity_name) + "/" + str(cam_name)
                if key in self._known:
                    continue
                if isinstance(value, tuple) and len(value) >= 2:
                    camera, parent_link = value[0], value[1]
                else:
                    camera, parent_link = value, None
                frame_id = str(entity_name) + "/" + str(cam_name) + "_optical"
                entry_cfg = self.cfg.get(key, {}) if isinstance(self.cfg, dict) else {}
                entry = _CameraEntry(
                    entity_name=str(entity_name),
                    cam_name=str(cam_name),
                    camera=camera,
                    parent_link=parent_link,
                    frame_id=str(entry_cfg.get("frame_id", frame_id)),
                    publish_depth=bool(entry_cfg.get("publish_depth", True)),
                    publish_pointcloud=bool(entry_cfg.get("publish_pointcloud", False)),
                    publish_segmentation=bool(entry_cfg.get("publish_segmentation", False)),
                    rate_divisor=int(entry_cfg.get("rate_divisor", 1)),
                )
                self._known[key] = entry
                self._entries.append(entry)

    def _publish_entry(self, entry: _CameraEntry, sim_time) -> None:
        camera = entry.camera
        rendered = camera.render(rgb=True, depth=entry.publish_depth, segmentation=entry.publish_segmentation)
        rgb, depth, seg, _normal = rendered
        ns = "/" + entry.entity_name + "/" + entry.cam_name

        n_envs = int(getattr(self.scene, "n_envs", 0) or 0)

        if rgb is not None:
            rgb_np = self._slice_batch(rgb, n_envs, expected_trailing_channels=3)
            if rgb_np.dtype != np.uint8:
                rgb_np = np.clip(rgb_np, 0, 255).astype(np.uint8)
            img_msg = self._rgb_to_msg(rgb_np, entry.frame_id, sim_time)
            self._get_pub(entry, "image_raw", Image, ns + "/image_raw").publish(img_msg)

        if entry.publish_depth and depth is not None:
            depth_np = self._slice_batch(depth, n_envs, expected_trailing_channels=None)
            depth_np = np.asarray(depth_np, dtype=np.float32)
            depth_msg = self._depth_to_msg(depth_np, entry.frame_id, sim_time)
            self._get_pub(entry, "depth", Image, ns + "/depth").publish(depth_msg)

        info_msg = self._build_camera_info(camera, entry.frame_id, sim_time)
        self._get_pub(entry, "camera_info", CameraInfo, ns + "/camera_info").publish(info_msg)

        if entry.publish_pointcloud:
            try:
                points, mask = camera.render_pointcloud(world_frame=True)
                pc_msg = self._build_pointcloud(points, mask, sim_time)
                if pc_msg is not None:
                    self._get_pub(entry, "points", PointCloud2, ns + "/points").publish(pc_msg)
            except Exception as exc:  # pragma: no cover
                self.node.get_logger().warning("CameraPublisher pointcloud failed: " + repr(exc))

        if entry.publish_segmentation and seg is not None:
            seg_np = self._slice_batch(seg, n_envs, expected_trailing_channels=None)
            seg_np = np.asarray(seg_np)
            if seg_np.ndim == 3 and seg_np.shape[-1] == 1:
                seg_np = seg_np[..., 0]
            seg_u16 = seg_np.astype(np.uint16, copy=False)
            seg_msg = conv.np_to_image_msg(seg_u16, "mono16", stamp=sim_time, frame_id=entry.frame_id)
            self._get_pub(entry, "segmentation", Image, ns + "/segmentation").publish(seg_msg)

        # Emit camera TF EVERY tick, not just once: cameras attached to
        # a moving link change their world transform each step, so a
        # one-shot TF leaves point clouds / images stamped against a
        # frozen frame. Cheap (one TransformStamped per camera).
        try:
            self._emit_tf(entry, sim_time)
        except Exception as exc:  # pragma: no cover
            self.node.get_logger().warning("CameraPublisher TF emit failed: " + repr(exc))

    @staticmethod
    def _slice_batch(tensor, n_envs: int, expected_trailing_channels):
        """Return the array for ``env_idx=0`` without mangling
        non-batched camera output.

        ``conv.select_env`` blindly slices ``arr[0]`` whenever
        ``arr.ndim > 1``. For camera frames that means an unvectorised
        ``(H, W, 3)`` gets chopped to ``(W, 3)``, which cv_bridge rejects
        as 8UC1. Only slice when the scene actually has a batch dim.
        """
        arr = conv._as_numpy(tensor)
        # Multi-env case: ``arr.shape[0] == n_envs`` and arr.ndim matches
        # the unvectorised ndim + 1 (H,W,3) -> (n_envs,H,W,3).
        if n_envs > 1 and arr.ndim >= 3 and arr.shape[0] == n_envs:
            return arr[0]
        # Sometimes Genesis still returns a leading 1-dim even in the
        # non-vectorised case. Strip only that leading 1.
        if arr.ndim >= 3 and arr.shape[0] == 1:
            squeezed = arr[0]
            if expected_trailing_channels is None or (
                squeezed.ndim >= 2 and (
                    squeezed.ndim == 2
                    or squeezed.shape[-1] == expected_trailing_channels
                )
            ):
                return squeezed
        return arr

    @staticmethod
    def _rgb_to_msg(rgb_np, frame_id, sim_time):
        if _CV_BRIDGE is not None:
            msg = _CV_BRIDGE.cv2_to_imgmsg(np.ascontiguousarray(rgb_np), "rgb8")
            msg.header.stamp = sim_time
            msg.header.frame_id = frame_id
            return msg
        return conv.np_to_image_msg(rgb_np, "rgb8", stamp=sim_time, frame_id=frame_id)

    @staticmethod
    def _depth_to_msg(depth_np, frame_id, sim_time):
        if _CV_BRIDGE is not None:
            msg = _CV_BRIDGE.cv2_to_imgmsg(np.ascontiguousarray(depth_np), "32FC1")
            msg.header.stamp = sim_time
            msg.header.frame_id = frame_id
            return msg
        return conv.np_to_image_msg(depth_np, "32FC1", stamp=sim_time, frame_id=frame_id)

    @staticmethod
    def _build_camera_info(camera, frame_id, sim_time):
        info = CameraInfo()
        info.header.stamp = sim_time
        info.header.frame_id = frame_id
        try:
            w, h = int(camera.res[0]), int(camera.res[1])
        except Exception:
            w, h = 0, 0
        info.width = w
        info.height = h
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        K_arr = np.asarray(camera.intrinsics, dtype=np.float64).reshape(-1).tolist()
        if len(K_arr) < 9:
            K_arr = (K_arr + [0.0] * 9)[:9]
        else:
            K_arr = K_arr[:9]
        info.k = K_arr
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        P = K_arr[:3] + [0.0] + K_arr[3:6] + [0.0] + K_arr[6:9] + [0.0]
        info.p = P
        info.binning_x = 0
        info.binning_y = 0
        return info

    @staticmethod
    def _build_pointcloud(points, mask, sim_time):
        pts = conv._as_numpy(points) if hasattr(conv, "_as_numpy") else np.asarray(points)
        msk = conv._as_numpy(mask) if hasattr(conv, "_as_numpy") else np.asarray(mask)
        if pts.ndim == 4:
            pts = pts[0]
            if msk.ndim == 3:
                msk = msk[0]
        if pts.size == 0:
            return None
        flat_pts = pts.reshape(-1, 3)
        flat_mask = np.asarray(msk, dtype=bool).reshape(-1)
        if flat_mask.shape[0] == flat_pts.shape[0]:
            valid = flat_pts[flat_mask]
        else:
            valid = flat_pts
        valid = np.ascontiguousarray(valid, dtype=np.float32)
        n = int(valid.shape[0])
        msg = PointCloud2()
        msg.header.stamp = sim_time
        msg.header.frame_id = "world"
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
        msg.is_dense = True
        msg.data = valid.tobytes()
        return msg

    def _emit_tf(self, entry: _CameraEntry, sim_time) -> None:
        T = np.asarray(entry.camera.transform, dtype=np.float64).reshape(4, 4)
        translation = T[:3, 3]
        R = T[:3, :3]
        w, x, y, z = _matrix_to_quat_wxyz(R)

        t = TransformStamped()
        t.header.stamp = sim_time
        t.header.frame_id = "world"
        t.child_frame_id = entry.frame_id
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        t.transform.rotation.w = float(w)
        t.transform.rotation.x = float(x)
        t.transform.rotation.y = float(y)
        t.transform.rotation.z = float(z)

        try:
            from genesis_ros.publishers.tf import TFPublisher  # type: ignore
            TFPublisher.queue_transform(t)
        except (ImportError, AttributeError):
            pass

    def _get_pub(self, entry: _CameraEntry, key: str, msg_type, topic: str):
        pub = entry.pubs.get(key)
        if pub is None:
            pub = self.node.create_publisher(msg_type, topic, SENSOR_QOS)
            entry.pubs[key] = pub
        return pub


__all__ = ["CameraPublisher"]
