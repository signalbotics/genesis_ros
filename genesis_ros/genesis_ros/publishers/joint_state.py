"""Per-entity sensor_msgs/JointState publisher.

Each registered entity gets a dedicated /{name}/joint_states publisher
lazily created on first publish. The publisher walks the joint list at
rate_hz (decimated to the nearest scene tick) and emits current position /
velocity / effort vectors sliced to the selected environment.

Controllers driving a robot from ROS command topics can opt out with
JointStatePublisher.suppress_for to prevent double-publishing.
"""
from __future__ import annotations

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header
    from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Point, Twist, Pose
    from sensor_msgs.msg import JointState
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

import threading
from typing import Any, Dict, List, Optional, Set

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


class JointStatePublisher(GenesisPublisher):
    """Publish /{entity}/joint_states for every registered articulated entity."""

    _suppressed_entities: Set[str] = set()
    _suppressed_lock = threading.Lock()

    @classmethod
    def suppress_for(cls, name: str) -> None:
        with cls._suppressed_lock:
            cls._suppressed_entities.add(str(name))

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "JointStatePublisher requires rclpy / ROS 2 to be installed"
            )
        super().__init__(node, scene, registry, cfg)

        rate_hz = float(self.cfg.get("rate_hz", 100.0))
        self.rate_hz = rate_hz
        scene_dt = float(getattr(scene, "dt", 0.01) or 0.01)
        if rate_hz <= 0.0 or scene_dt <= 0.0:
            self.decimation = 1
        else:
            self.decimation = max(1, int(round(1.0 / (rate_hz * scene_dt))))
        self.env_idx: int = int(self.cfg.get("env_idx", 0))

        self._publishers: Dict[str, Any] = {}
        self._counter: int = 0

    def _get_publisher(self, entity_name: str):
        pub = self._publishers.get(entity_name)
        if pub is not None:
            return pub
        topic = "/" + entity_name + "/joint_states"
        pub = self.node.create_publisher(JointState, topic, STATE_QOS)
        self._publishers[entity_name] = pub
        return pub


    def step(self, sim_time) -> None:
        self._counter += 1
        if (self._counter % self.decimation) != 0:
            return

        with JointStatePublisher._suppressed_lock:
            suppressed = set(JointStatePublisher._suppressed_entities)

        for record in _iter_registry_records(self.registry):
            name = str(getattr(record, "name", ""))
            if not name or name in suppressed:
                continue
            entity = _record_entity(record)
            if entity is None:
                continue

            joints = list(getattr(record, "joints", None) or [])
            if not joints:
                continue

            try:
                qpos_all = entity.get_qpos()
                vel_all = entity.get_dofs_velocity()
                eff_all = entity.get_dofs_control_force()
            except Exception:
                continue

            qpos = np.asarray(conv.select_env(qpos_all, self.env_idx)).reshape(-1)
            vel = np.asarray(conv.select_env(vel_all, self.env_idx)).reshape(-1)
            eff = np.asarray(conv.select_env(eff_all, self.env_idx)).reshape(-1)

            names: List[str] = []
            positions: List[float] = []
            velocities: List[float] = []
            efforts: List[float] = []
            for joint in joints:
                jname = getattr(joint, "name", None)
                if not jname:
                    continue
                dof_idx = getattr(joint, "dof_idx", None)
                if dof_idx is None:
                    continue
                try:
                    dof_idx_int = int(dof_idx)
                except (TypeError, ValueError):
                    continue
                names.append(str(jname))
                positions.append(
                    float(qpos[dof_idx_int]) if dof_idx_int < qpos.shape[0] else 0.0
                )
                velocities.append(
                    float(vel[dof_idx_int]) if dof_idx_int < vel.shape[0] else 0.0
                )
                efforts.append(
                    float(eff[dof_idx_int]) if dof_idx_int < eff.shape[0] else 0.0
                )

            if not names:
                continue

            msg = JointState()
            msg.header.stamp = sim_time
            msg.name = names
            msg.position = positions
            msg.velocity = velocities
            msg.effort = efforts

            self._get_publisher(name).publish(msg)


__all__ = ["JointStatePublisher"]
