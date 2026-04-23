"""Ground-truth pose publisher -- world-frame ``geometry_msgs/PoseStamped``.

Distinct from :mod:`genesis_ros.publishers.tf` so Nav2 / SLAM users can
compare their estimated pose against the simulator's ground truth
without having to parse ``/tf``. Opt-in per entity: pass
``ground_truth=True`` to ``bridge.register_entity`` or add the entity
name to the ``entities`` list in the publisher ``cfg``.

Topic: ``/{robot}/pose_ground_truth`` (STATE_QOS).
Rate: ``cfg['rate_hz']`` (default 50 Hz). Throttled on ``sim_time``, so
behaviour is stable at any RTF.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from geometry_msgs.msg import Point, PoseStamped
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

from typing import Any, Dict, Optional, Sequence

import numpy as np

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import STATE_QOS
from genesis_ros import conversions as conv


def _iter_registry_records(registry):
    if registry is None:
        return []
    fn = getattr(registry, "values", None)
    if callable(fn):
        try:
            return list(fn())
        except TypeError:
            pass
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


def _record_flag(record, key: str) -> bool:
    if hasattr(record, key):
        try:
            return bool(getattr(record, key))
        except Exception:
            return False
    extras = getattr(record, "extras", None)
    if isinstance(extras, dict):
        return bool(extras.get(key, False))
    return False


class GroundTruthPosePublisher(GenesisPublisher):
    """Publish ground-truth world-frame pose for opt-in entities."""

    def __init__(self, node, scene, registry, cfg: Optional[dict] = None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "GroundTruthPosePublisher requires rclpy / geometry_msgs"
            )
        super().__init__(node, scene, registry, cfg)
        self.env_idx: int = int(self.cfg.get("env_idx", 0))
        self.world_frame: str = str(self.cfg.get("world_frame", "world"))

        rate_hz = float(self.cfg.get("rate_hz", 50.0) or 50.0)
        if rate_hz <= 0.0:
            rate_hz = 50.0
        self._period_ns: int = int(round(1e9 / rate_hz))
        self._last_stamp_ns: Dict[str, int] = {}

        # cfg-level allowlist. Entities not named here still qualify if
        # their registry record carries ground_truth=True.
        entities = self.cfg.get("entities", ())
        self._opt_in: set = {str(e) for e in (entities or ())}

        self._publishers: Dict[str, Any] = {}

    # -------------------------------------------------------- plumbing
    def _get_publisher(self, entity_name: str):
        pub = self._publishers.get(entity_name)
        if pub is not None:
            return pub
        topic = "/" + entity_name + "/pose_ground_truth"
        pub = self.node.create_publisher(PoseStamped, topic, STATE_QOS)
        self._publishers[entity_name] = pub
        return pub

    def _enabled(self, name: str, record) -> bool:
        if name in self._opt_in:
            return True
        return _record_flag(record, "ground_truth")

    # -------------------------------------------------------- main loop
    def step(self, sim_time) -> None:
        now_ns = int(sim_time.sec) * 1_000_000_000 + int(sim_time.nanosec)
        for record in _iter_registry_records(self.registry):
            name = str(getattr(record, "name", "") or "")
            if not name or not self._enabled(name, record):
                continue
            last = self._last_stamp_ns.get(name, -self._period_ns - 1)
            if (now_ns - last) < self._period_ns:
                continue
            entity = _record_entity(record)
            if entity is None:
                continue

            try:
                pos_all = entity.get_pos()
                quat_all = entity.get_quat()
            except Exception:
                continue

            pos = np.asarray(conv.select_env(pos_all, self.env_idx)).reshape(-1)
            quat = np.asarray(conv.select_env(quat_all, self.env_idx)).reshape(-1)
            if pos.shape[0] < 3 or quat.shape[0] < 4:
                continue

            msg = PoseStamped()
            msg.header.stamp = sim_time
            msg.header.frame_id = self.world_frame
            msg.pose.position = Point(
                x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
            )
            msg.pose.orientation = conv.quat_wxyz_to_ros(quat)
            self._get_publisher(name).publish(msg)
            self._last_stamp_ns[name] = now_ns


__all__ = ["GroundTruthPosePublisher"]
