"""nav_msgs/Odometry publisher for mobile bases.

Emits /{name}/odom for every registry record flagged is_mobile_base=True
and queues the matching odom -> base_link TransformStamped onto the shared
TFPublisher so robot_state_publisher consumers see a consistent TF tree.
"""
from __future__ import annotations

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header
    from geometry_msgs.msg import (
        TransformStamped,
        Quaternion,
        Vector3,
        Point,
        Twist,
        Pose,
    )
    from nav_msgs.msg import Odometry
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

from typing import Any, Dict, List, Optional

import numpy as np

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import STATE_QOS, TF_QOS, TF_STATIC_QOS
from genesis_ros import conversions as conv


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


_ZERO_COV = [0.0] * 36


class OdomPublisher(GenesisPublisher):
    """Publish Odometry for every registry record with is_mobile_base."""

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "OdomPublisher requires rclpy / ROS 2 to be installed"
            )
        super().__init__(node, scene, registry, cfg)
        self.env_idx: int = int(self.cfg.get("env_idx", 0))
        self._publishers: Dict[str, Any] = {}

    def _get_publisher(self, entity_name: str):
        pub = self._publishers.get(entity_name)
        if pub is not None:
            return pub
        topic = "/" + entity_name + "/odom"
        pub = self.node.create_publisher(Odometry, topic, STATE_QOS)
        self._publishers[entity_name] = pub
        return pub


    def step(self, sim_time) -> None:
        for record in _iter_registry_records(self.registry):
            if not bool(getattr(record, "is_mobile_base", False)):
                continue
            name = str(getattr(record, "name", ""))
            if not name:
                continue
            entity = _record_entity(record)
            if entity is None:
                continue

            try:
                pos_all = entity.get_pos()
                quat_all = entity.get_quat()
                lin_all = entity.get_vel()
                ang_all = entity.get_links_ang([0])
            except Exception:
                continue

            pos = np.asarray(conv.select_env(pos_all, self.env_idx)).reshape(-1)
            quat = np.asarray(conv.select_env(quat_all, self.env_idx)).reshape(-1)
            lin = np.asarray(conv.select_env(lin_all, self.env_idx)).reshape(-1)
            ang = np.asarray(conv.select_env(ang_all, self.env_idx)).reshape(-1)

            if pos.shape[0] < 3 or quat.shape[0] < 4:
                continue
            if lin.shape[0] < 3:
                lin = np.concatenate([lin, np.zeros(3 - lin.shape[0])])
            if ang.shape[0] < 3:
                ang = np.concatenate([ang, np.zeros(3 - ang.shape[0])])

            msg = Odometry()
            msg.header.stamp = sim_time
            msg.header.frame_id = name + "/odom"
            msg.child_frame_id = name + "/base_link"

            pose = Pose()
            pose.position = Point(
                x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
            )
            pose.orientation = conv.quat_wxyz_to_ros(quat)
            msg.pose.pose = pose
            msg.pose.covariance = list(_ZERO_COV)

            twist = Twist()
            twist.linear = Vector3(
                x=float(lin[0]), y=float(lin[1]), z=float(lin[2])
            )
            twist.angular = Vector3(
                x=float(ang[0]), y=float(ang[1]), z=float(ang[2])
            )
            msg.twist.twist = twist
            msg.twist.covariance = list(_ZERO_COV)

            self._get_publisher(name).publish(msg)

            try:
                from genesis_ros.publishers.tf import TFPublisher

                tf_msg = TransformStamped()
                tf_msg.header.stamp = sim_time
                tf_msg.header.frame_id = name + "/odom"
                tf_msg.child_frame_id = name + "/base_link"
                tf_msg.transform.translation = Vector3(
                    x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
                )
                tf_msg.transform.rotation = conv.quat_wxyz_to_ros(quat)
                TFPublisher.queue_transform(tf_msg)
            except (ImportError, AttributeError):
                pass


__all__ = ["OdomPublisher"]
