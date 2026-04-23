"""Topic-based ros2_control bridge for Genesis entities.

Implements the Track B side of Group 8: for every registered entity whose
URDF carries a <ros2_control> block, we stand up one ControlTopicBridge.
That object combines a publisher (joint state feed) and a subscriber (joint
commands) so a single entity only consumes one pair of topics and one lock.

This module must remain importable even when rclpy is unavailable --
parse_ros2_control is useful in offline tooling. Callers that try to
instantiate ControlTopicBridge without ROS will get an immediate ImportError.
"""
from __future__ import annotations

import threading
from typing import Any, Dict, List, Optional, Tuple

try:
    import rclpy  # noqa: F401
    from rclpy.qos import QoSProfile  # noqa: F401
    from sensor_msgs.msg import JointState

    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover
    _ROS_AVAILABLE = False

from genesis_ros.node import GenesisPublisher, GenesisSubscriber
from genesis_ros.qos import STATE_QOS
from genesis_ros import conversions as conv  # noqa: F401
from genesis_ros.control.urdf_ros2_control import (
    JointSpec,
    Ros2ControlSpec,
    parse as parse_ros2_control,
    _pick_command_interface,
)



def _as_list(value):
    """Convert an arbitrary array-like (tensor / ndarray / sequence) to list."""
    if value is None:
        return []
    tolist = getattr(value, "tolist", None)
    if tolist is not None:
        try:
            out = tolist()
            if isinstance(out, list):
                return [float(x) for x in out]
            return [float(out)]
        except Exception:
            pass
    try:
        return [float(x) for x in value]
    except TypeError:
        return [float(value)]


def _record_joints(record):
    """Return a list of joint handles from an entity record.

    Registry records from Group 2 expose .joints; the Group 1 stub exposes
    .gs_entity.joints. Accept both.
    """
    joints = getattr(record, "joints", None)
    if joints:
        return list(joints)
    gs_entity = getattr(record, "gs_entity", None) or getattr(record, "entity", None)
    if gs_entity is not None:
        return list(getattr(gs_entity, "joints", []) or [])
    return []


def _record_entity(record):
    """Return the wrapped Genesis entity from a registry record."""
    return (
        getattr(record, "gs_entity", None)
        or getattr(record, "entity", None)
        or record
    )


def _record_urdf(record):
    return getattr(record, "urdf_xml", None)


def _joint_name(joint):
    name = getattr(joint, "name", None)
    if name is None:
        return None
    return str(name)


def _joint_dof_idx(joint):
    """Best-effort extraction of the DOF index for a joint.

    Genesis rigid joints expose dof_idx_local (preferred) or
    dofs_idx_local (list form); older paths use q_start / idx.
    Returns None for fixed joints (0 DOFs).
    """
    for attr in ("dof_idx_local", "dof_start", "q_start", "idx"):
        val = getattr(joint, attr, None)
        if val is None:
            continue
        if isinstance(val, (list, tuple)):
            if len(val) == 0:
                return None
            return int(val[0])
        try:
            return int(val)
        except (TypeError, ValueError):
            continue
    dofs = getattr(joint, "dofs_idx_local", None)
    if dofs:
        try:
            return int(list(dofs)[0])
        except Exception:
            return None
    return None



class ControlTopicBridge(GenesisPublisher, GenesisSubscriber):
    def __init__(self, node, scene, registry, cfg):
        GenesisPublisher.__init__(self, node, scene, registry, cfg)
        if not _ROS_AVAILABLE:
            raise ImportError(
                "ControlTopicBridge requires rclpy + sensor_msgs."
            )
        entity_name = self.cfg.get("entity_name")
        if not entity_name:
            raise ValueError("ControlTopicBridge: cfg[entity_name] is required.")
        self._entity_name = str(entity_name)

        record = self._lookup_record(self._entity_name)
        if record is None:
            raise KeyError(
                "ControlTopicBridge: no entity registered under "
                + repr(self._entity_name)
            )
        self._record = record
        self._entity = _record_entity(record)

        urdf_xml = _record_urdf(record)
        specs = parse_ros2_control(urdf_xml or "")
        if not specs:
            raise ValueError(
                "ControlTopicBridge: entity "
                + repr(self._entity_name)
                + " has no ros2_control block in its URDF."
            )
        self._spec = specs[0]
