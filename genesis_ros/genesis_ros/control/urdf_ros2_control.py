"""Pure-Python parser for the URDF ``<ros2_control>`` element.

This module deliberately has **no** ROS 2 runtime dependencies -- it only
reads XML with the standard library so the rest of the package can import it
even on boxes without ``rclpy`` installed (unit tests, parser fuzzing, CI).

The single public entry point is :func:`parse`. It returns a list of
:class:`Ros2ControlSpec` records, one per ``<ros2_control>`` block in the
URDF. ``topic_bridge.ControlTopicBridge`` consumes the specs to wire Genesis
joints to `topic_based_ros2_control
<https://github.com/PickNikRobotics/topic_based_ros2_control>`_ topics.
"""
from __future__ import annotations

import warnings
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional


# Ordered by dispatch priority in :func:`_pick_command_interface`.
_COMMAND_INTERFACE_PRIORITY = ("position", "velocity", "effort")


@dataclass
class JointSpec:
    """Per-joint slice of a ``<ros2_control>`` block.

    Fields mirror the subset of the `ros2_control URDF schema
    <https://control.ros.org/master/doc/ros2_control/doc/ros2_control_urdf.html>`_
    that we need at runtime. ``command_interfaces`` preserves the original
    declaration order; ``topic_bridge.py`` chooses one via
    :func:`_pick_command_interface` when dispatching commands.
    """

    name: str
    command_interfaces: List[str] = field(default_factory=list)
    state_interfaces: List[str] = field(default_factory=list)
    kp: Optional[float] = None
    kv: Optional[float] = None
    min_pos: Optional[float] = None
    max_pos: Optional[float] = None
    min_vel: Optional[float] = None
    max_vel: Optional[float] = None
    min_effort: Optional[float] = None
    max_effort: Optional[float] = None
    initial_value: Optional[float] = None


@dataclass
class Ros2ControlSpec:
    """One ``<ros2_control name=... type="system">`` block."""

    name: str
    hw_plugin: str
    joints: List[JointSpec] = field(default_factory=list)


def _as_float(text):
    if text is None:
        return None
    text = str(text).strip()
    if not text:
        return None
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _param_value(elem, param_name):
    """Return the value of ``<param name=param_name>`` as text.

    Supports three styles observed in the wild:

    * ``<param name="kp">42</param>``   (canonical)
    * ``<param name="kp" value="42"/>`` (attribute-style)
    * ``<kp>42</kp>`` / ``<kp value="42"/>`` (shorthand element)

    Returns ``None`` when none of those match.
    """
    for child in elem.findall("param"):
        if child.attrib.get("name") == param_name:
            if child.text is not None and child.text.strip():
                return child.text.strip()
            if "value" in child.attrib:
                return child.attrib["value"]
    shortcut = elem.find(param_name)
    if shortcut is not None:
        if shortcut.text is not None and shortcut.text.strip():
            return shortcut.text.strip()
        if "value" in shortcut.attrib:
            return shortcut.attrib["value"]
    return None


def _pick_command_interface(joint_spec):
    """Resolve a joint's active command interface using the frozen priority.

    Returns ``None`` if the joint declared no command interfaces.
    """
    declared = list(joint_spec.command_interfaces or [])
    if not declared:
        return None
    if len(declared) > 1:
        chosen = None
        for candidate in _COMMAND_INTERFACE_PRIORITY:
            if candidate in declared:
                chosen = candidate
                break
        if chosen is None:
            chosen = declared[0]
        warnings.warn(
            "ros2_control joint "
            + repr(joint_spec.name)
            + " declared multiple command interfaces "
            + repr(declared)
            + "; using "
            + repr(chosen)
            + " (priority: position > velocity > effort).",
            RuntimeWarning,
            stacklevel=2,
        )
        return chosen
    return declared[0]


def _parse_interface_block(elem, joint_spec, kind):
    """Populate ``joint_spec`` from a ``<command_interface>`` / ``<state_interface>``.

    ``kind`` is ``"command"`` or ``"state"``. Interface-level ``<param>`` entries
    for ``min`` / ``max`` map onto ``min_pos`` / ``max_pos`` / ``min_vel`` / ... per
    the interface name.
    """
    name = elem.attrib.get("name")
    if not name:
        return

    if kind == "command":
        joint_spec.command_interfaces.append(name)
    else:
        joint_spec.state_interfaces.append(name)

    min_text = _param_value(elem, "min")
    max_text = _param_value(elem, "max")
    min_val = _as_float(min_text)
    max_val = _as_float(max_text)

    if name == "position":
        if min_val is not None:
            joint_spec.min_pos = min_val
        if max_val is not None:
            joint_spec.max_pos = max_val
    elif name == "velocity":
        if min_val is not None:
            joint_spec.min_vel = min_val
        if max_val is not None:
            joint_spec.max_vel = max_val
    elif name == "effort":
        if min_val is not None:
            joint_spec.min_effort = min_val
        if max_val is not None:
            joint_spec.max_effort = max_val

    initial = _as_float(_param_value(elem, "initial_value"))
    if initial is not None:
        joint_spec.initial_value = initial


def _parse_joint(joint_elem):
    name = joint_elem.attrib.get("name")
    if not name:
        return None

    spec = JointSpec(name=name)

    for cmd in joint_elem.findall("command_interface"):
        _parse_interface_block(cmd, spec, "command")

    for st in joint_elem.findall("state_interface"):
        _parse_interface_block(st, spec, "state")

    # Joint-level params (PD gains, initial value). The <ros2_control> spec puts
    # these directly under <joint> but we also tolerate nested
    # <param name="..."> children at the interface level (already handled).
    kp = _as_float(_param_value(joint_elem, "kp"))
    if kp is not None:
        spec.kp = kp
    kv = _as_float(_param_value(joint_elem, "kv"))
    if kv is not None:
        spec.kv = kv
    initial = _as_float(_param_value(joint_elem, "initial_value"))
    if initial is not None and spec.initial_value is None:
        spec.initial_value = initial

    return spec


def parse(urdf_xml):
    """Parse all ``<ros2_control>`` blocks from a URDF string.

    Parameters
    ----------
    urdf_xml:
        Complete URDF XML source (as returned by ``xacro`` or stored in
        ``EntityRecord.urdf_xml``).

    Returns
    -------
    list[Ros2ControlSpec]
        One spec per ``<ros2_control>`` element found. Empty list when none.
    """
    if not urdf_xml:
        return []

    try:
        root = ET.fromstring(urdf_xml)
    except ET.ParseError as exc:
        warnings.warn(
            "urdf_ros2_control.parse: XML parse error " + repr(exc),
            RuntimeWarning,
            stacklevel=2,
        )
        return []

    specs = []
    # <ros2_control> can appear at any depth (some xacro layouts nest it under
    # a <control> group). Iterate descendants to be permissive.
    for rc in root.iter("ros2_control"):
        rc_name = rc.attrib.get("name", "")

        plugin_text = "UNKNOWN"
        hw = rc.find("hardware")
        if hw is not None:
            plugin_elem = hw.find("plugin")
            if plugin_elem is not None and plugin_elem.text:
                plugin_text = plugin_elem.text.strip()

        spec = Ros2ControlSpec(name=rc_name, hw_plugin=plugin_text)

        for joint_elem in rc.findall("joint"):
            joint_spec = _parse_joint(joint_elem)
            if joint_spec is None:
                continue
            # Warn+resolve multi-command-interface ambiguity at parse time so
            # callers see one diagnostic per joint instead of per command tick.
            _pick_command_interface(joint_spec)
            spec.joints.append(joint_spec)

        specs.append(spec)

    return specs


__all__ = [
    "JointSpec",
    "Ros2ControlSpec",
    "parse",
]
