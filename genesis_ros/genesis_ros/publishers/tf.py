"""TF publishers: dynamic /tf and one-shot /tf_static.

TFPublisher walks the entity registry every rate_divisor ticks and
emits a tf2_msgs/TFMessage with every (parent_link -> link) edge plus
any transforms queued via queue_transform().
"""
from __future__ import annotations

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header
    from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Point, Twist, Pose
    from tf2_msgs.msg import TFMessage
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

import math
import threading
import xml.etree.ElementTree as ET
from typing import Any, List, Optional

import numpy as np

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import STATE_QOS, TF_QOS, TF_STATIC_QOS
from genesis_ros import conversions as conv


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quat_inv(q):
    w, x, y, z = q
    n2 = w * w + x * x + y * y + z * z
    if n2 <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    inv_n = 1.0 / n2
    return (w * inv_n, -x * inv_n, -y * inv_n, -z * inv_n)


def _quat_rotate(q, v):
    qv = (0.0, float(v[0]), float(v[1]), float(v[2]))
    q_inv = _quat_inv(q)
    t = _quat_mul(q, qv)
    r = _quat_mul(t, q_inv)
    return (r[1], r[2], r[3])


def _rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def _normalize_quat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def _make_tf(stamp, parent_frame, child_frame, pos, quat_wxyz):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame
    msg.transform.translation = Vector3(
        x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
    )
    w, x, y, z = quat_wxyz
    msg.transform.rotation = Quaternion(
        x=float(x), y=float(y), z=float(z), w=float(w)
    )
    return msg


def _iter_registry_records(registry):
    if registry is None:
        return []
    for attr in ("records", "values", "items"):
        fn = getattr(registry, attr, None)
        if fn is None:
            continue
        try:
            out = fn()
        except TypeError:
            continue
        result = []
        for entry in out:
            if isinstance(entry, tuple) and len(entry) == 2:
                result.append(entry[1])
            else:
                result.append(entry)
        return result
    try:
        return list(registry)
    except TypeError:
        return []


def _record_entity(record):
    for attr in ("entity", "gs_entity"):
        ent = getattr(record, attr, None)
        if ent is not None:
            return ent
    return None


def _record_name(record):
    return str(getattr(record, "name", ""))


def _record_links(record):
    links = getattr(record, "links", None)
    if links is not None:
        return list(links)
    return []


def _record_urdf_root(record):
    root = getattr(record, "urdf_root", None)
    if root is not None:
        return root
    urdf_xml = getattr(record, "urdf_xml", None)
    if not urdf_xml:
        return None
    try:
        return ET.fromstring(urdf_xml)
    except ET.ParseError:
        return None


def _record_is_mobile_base(record):
    return bool(getattr(record, "is_mobile_base", False))


class TFPublisher(GenesisPublisher):
    """Publish dynamic /tf frames computed from Genesis link poses."""

    _instances: List["TFPublisher"] = []

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError("TFPublisher requires rclpy / ROS 2 to be installed")
        super().__init__(node, scene, registry, cfg)
        self.rate_divisor = int(self.cfg.get("rate_divisor", 1))
        if self.rate_divisor < 1:
            self.rate_divisor = 1
        self._counter = 0
        self.env_idx = int(self.cfg.get("env_idx", 0))

        self._pub = self.node.create_publisher(TFMessage, "/tf", TF_QOS)
        self._extra_queue: List[Any] = []
        self._queue_lock = threading.Lock()
        TFPublisher._instances.append(self)

    @classmethod
    def queue_transform(cls, t):
        if not cls._instances:
            return
        inst = cls._instances[0]
        with inst._queue_lock:
            inst._extra_queue.append(t)


    def step(self, sim_time):
        self._counter += 1
        if (self._counter % self.rate_divisor) != 0:
            return

        transforms: List[Any] = []
        for record in _iter_registry_records(self.registry):
            entity = _record_entity(record)
            if entity is None:
                continue

            links = _record_links(record)
            if not links:
                continue

            try:
                positions_all = entity.get_links_pos()
                quats_all = entity.get_links_quat()
            except Exception:
                continue

            positions = conv.select_env(positions_all, self.env_idx)
            quats = conv.select_env(quats_all, self.env_idx)
            positions = np.asarray(positions).reshape(-1, 3)
            quats = np.asarray(quats).reshape(-1, 4)

            link_info = []
            for idx, link in enumerate(links):
                link_name = getattr(link, "name", None) or ("link_" + str(idx))
                link_idx = getattr(link, "idx", idx)
                parent_name = getattr(link, "parent_name", None)
                if link_idx is None or link_idx >= positions.shape[0]:
                    continue
                pos = positions[link_idx]
                quat = tuple(float(v) for v in quats[link_idx])
                link_info.append((link_name, parent_name, pos, quat))

            name_to_world = {info[0]: (info[2], info[3]) for info in link_info}

            record_name = _record_name(record)
            for link_name, parent_name, pos_w, quat_w in link_info:
                if parent_name is None:
                    continue
                parent = name_to_world.get(parent_name)
                if parent is None:
                    continue
                parent_pos, parent_quat = parent
                dp = (
                    float(pos_w[0]) - float(parent_pos[0]),
                    float(pos_w[1]) - float(parent_pos[1]),
                    float(pos_w[2]) - float(parent_pos[2]),
                )
                inv_parent_q = _quat_inv(parent_quat)
                local_pos = _quat_rotate(inv_parent_q, dp)
                local_quat = _normalize_quat(_quat_mul(inv_parent_q, quat_w))

                tf_msg = _make_tf(
                    sim_time,
                    parent_frame=record_name + "/" + str(parent_name),
                    child_frame=record_name + "/" + str(link_name),
                    pos=local_pos,
                    quat_wxyz=local_quat,
                )
                transforms.append(tf_msg)

        with self._queue_lock:
            if self._extra_queue:
                for extra in self._extra_queue:
                    try:
                        if extra.header.stamp.sec == 0 and extra.header.stamp.nanosec == 0:
                            extra.header.stamp = sim_time
                    except AttributeError:
                        pass
                    transforms.append(extra)
                self._extra_queue.clear()

        if not transforms:
            return
        msg = TFMessage()
        msg.transforms = transforms
        self._pub.publish(msg)


class TFStaticPublisher(GenesisPublisher):
    """Publish URDF fixed joints + world->odom links exactly once."""

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "TFStaticPublisher requires rclpy / ROS 2 to be installed"
            )
        super().__init__(node, scene, registry, cfg)
        self._pub = self.node.create_publisher(
            TFMessage, "/tf_static", TF_STATIC_QOS
        )
        self._published = False


    def _collect_fixed_from_urdf(self, stamp, record_name, root):
        tfs = []
        if root is None:
            return tfs
        for joint in root.iter("joint"):
            if joint.get("type") != "fixed":
                continue
            parent_elem = joint.find("parent")
            child_elem = joint.find("child")
            if parent_elem is None or child_elem is None:
                continue
            parent_frame = parent_elem.get("link")
            child_frame = child_elem.get("link")
            if not parent_frame or not child_frame:
                continue

            origin = joint.find("origin")
            xyz = (0.0, 0.0, 0.0)
            rpy = (0.0, 0.0, 0.0)
            if origin is not None:
                raw_xyz = origin.get("xyz")
                raw_rpy = origin.get("rpy")
                if raw_xyz:
                    parts = raw_xyz.split()
                    if len(parts) == 3:
                        try:
                            xyz = tuple(float(p) for p in parts)
                        except ValueError:
                            xyz = (0.0, 0.0, 0.0)
                if raw_rpy:
                    parts = raw_rpy.split()
                    if len(parts) == 3:
                        try:
                            rpy = tuple(float(p) for p in parts)
                        except ValueError:
                            rpy = (0.0, 0.0, 0.0)

            quat = _rpy_to_quat(rpy[0], rpy[1], rpy[2])
            tfs.append(
                _make_tf(
                    stamp,
                    parent_frame=record_name + "/" + parent_frame,
                    child_frame=record_name + "/" + child_frame,
                    pos=xyz,
                    quat_wxyz=quat,
                )
            )
        return tfs


    def step(self, sim_time):
        if self._published:
            return

        transforms = []
        for record in _iter_registry_records(self.registry):
            record_name = _record_name(record)
            root = _record_urdf_root(record)
            transforms.extend(
                self._collect_fixed_from_urdf(sim_time, record_name, root)
            )
            if _record_is_mobile_base(record):
                # Mobile base: odom is the dynamic frame, world->odom is
                # the identity static root.
                transforms.append(
                    _make_tf(
                        sim_time,
                        parent_frame="world",
                        child_frame=record_name + "/odom",
                        pos=(0.0, 0.0, 0.0),
                        quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                    )
                )
            else:
                # Fixed-base: anchor the robot's root link directly to
                # world using the entity's current pose. Without this the
                # whole record_name/* frame subtree is orphaned and RViz
                # (Fixed Frame: world) shows nothing.
                base_tf = self._world_to_base_link(sim_time, record)
                if base_tf is not None:
                    transforms.append(base_tf)

        msg = TFMessage()
        msg.transforms = transforms
        self._pub.publish(msg)
        self._published = True

    def _world_to_base_link(self, sim_time, record):
        """Build ``world -> <record_name>/<base_link>`` from the entity's
        current base pose. Returns ``None`` if the entity does not
        expose get_pos / get_quat yet."""
        entity = getattr(record, "entity", None) or getattr(
            record, "gs_entity", None
        )
        if entity is None:
            return None
        base_link_name = (
            getattr(record, "base_link_name", None)
            or self._first_link_name(record)
        )
        if not base_link_name:
            return None

        pos = quat = None
        try:
            if hasattr(entity, "get_pos"):
                pos = conv._as_numpy(entity.get_pos()).reshape(-1)
            if hasattr(entity, "get_quat"):
                quat = conv._as_numpy(entity.get_quat()).reshape(-1)
        except Exception:
            return None

        if pos is None or quat is None or pos.size < 3 or quat.size < 4:
            return None

        # conv.select_env in case n_envs > 1; _as_numpy has flattened out
        # scene-level batch dims, but entity-level batch slicing still
        # matters when used.
        env_idx = int(self.cfg.get("env_idx", 0)) if isinstance(self.cfg, dict) else 0
        if pos.ndim > 1 or pos.size > 3:
            try:
                pos = np.asarray(conv.select_env(pos, env_idx)).reshape(-1)
                quat = np.asarray(conv.select_env(quat, env_idx)).reshape(-1)
            except Exception:
                pos = pos.reshape(-1)[:3]
                quat = quat.reshape(-1)[:4]

        pos_tuple = (float(pos[0]), float(pos[1]), float(pos[2]))
        quat_tuple = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        return _make_tf(
            sim_time,
            parent_frame="world",
            child_frame=_record_name(record) + "/" + str(base_link_name),
            pos=pos_tuple,
            quat_wxyz=quat_tuple,
        )

    @staticmethod
    def _first_link_name(record):
        links = getattr(record, "links", None) or ()
        for link in links:
            name = getattr(link, "name", None)
            if name:
                return str(name)
        return None


__all__ = ["TFPublisher", "TFStaticPublisher"]
