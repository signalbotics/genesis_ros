"""EntityRegistry -- pure-Python index of Genesis entities for the ROS 2 bridge.

The registry is the single source of truth for joint/link/sensor/camera
metadata that every publisher and subscriber in ``genesis_ros`` consults.
It intentionally has **no** rclpy / Genesis runtime dependencies beyond
reading attributes off the passed-in entity objects, so it can be
imported and unit tested on a box without ROS 2 installed.

See ``ROS2_INTEGRATION_PLAN.md`` -> "EntityRegistry" for the contract.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional, Tuple


# ---------------------------------------------------------------------------
# URDF joint-type string   <-   Genesis JOINT_TYPE enum mapping
# ---------------------------------------------------------------------------
# Genesis' ``JOINT_TYPE`` (``genesis/constants.py``) only distinguishes
# FIXED / REVOLUTE / PRISMATIC / SPHERICAL / FREE. URDF is richer
# (continuous, planar, floating) -- we normalise both directions so that
# every ``JointInfo.type`` is a lowercase string identical to the URDF
# spelling. ``SPHERICAL`` and ``FREE`` both map to ``"floating"`` since
# ROS tooling does not know about spherical joints.
_GS_ENUM_TO_URDF: Dict[str, str] = {
    "FIXED": "fixed",
    "REVOLUTE": "revolute",
    "PRISMATIC": "prismatic",
    "CONTINUOUS": "continuous",
    "PLANAR": "planar",
    "SPHERICAL": "floating",
    "FREE": "floating",
}


def _gs_joint_type_to_str(t: Any) -> str:
    """Coerce a Genesis ``JOINT_TYPE`` enum (or plain str/int) to URDF string."""
    if t is None:
        return "fixed"
    # Genesis enum: has ``.name`` attribute
    name = getattr(t, "name", None)
    if isinstance(name, str):
        return _GS_ENUM_TO_URDF.get(name.upper(), name.lower())
    if isinstance(t, str):
        return _GS_ENUM_TO_URDF.get(t.upper(), t.lower())
    # Integer enum fallback
    try:
        return _GS_ENUM_TO_URDF.get(str(t).upper(), str(t).lower())
    except Exception:
        return "fixed"


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------
@dataclass
class JointInfo:
    """Metadata describing a single revolute/prismatic/etc. joint."""

    name: str
    dof_idx: Optional[int]
    type: str
    parent_link: str
    child_link: str
    axis_xyz: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    limit_lower: float = 0.0
    limit_upper: float = 0.0
    effort_limit: float = 0.0
    velocity_limit: float = 0.0


@dataclass
class LinkInfo:
    """Metadata describing one rigid-body link."""

    name: str
    parent_name: Optional[str]
    local_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    local_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gs_link_idx: Optional[int] = None


@dataclass
class EntityRecord:
    """Everything the bridge knows about one registered Genesis entity."""

    name: str
    entity: Any
    urdf_xml: Optional[str] = None
    urdf_root: Optional[ET.Element] = None
    base_link_name: Optional[str] = None
    joints: List[JointInfo] = field(default_factory=list)
    links: List[LinkInfo] = field(default_factory=list)
    sensors: Dict[str, Any] = field(default_factory=dict)
    cameras: Dict[str, Tuple[Any, Optional[str]]] = field(default_factory=dict)
    is_mobile_base: bool = False
    cmd_vel_cfg: Optional[dict] = None
    ground_truth: bool = False


# ---------------------------------------------------------------------------
# URDF parsing helpers
# ---------------------------------------------------------------------------
def _parse_xyz(text: Optional[str], default: Tuple[float, float, float]
               ) -> Tuple[float, float, float]:
    if not text:
        return default
    parts = text.strip().split()
    if len(parts) != 3:
        return default
    try:
        return (float(parts[0]), float(parts[1]), float(parts[2]))
    except ValueError:
        return default


def _float_attr(element: Optional[ET.Element], attr: str, default: float) -> float:
    if element is None:
        return default
    raw = element.get(attr)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _parse_urdf(
    urdf_xml: str,
) -> Tuple[ET.Element, List[JointInfo], List[LinkInfo], Optional[str]]:
    """Walk the URDF tree and collect joints, links and base link name.

    ``fixed`` joints are recorded in ``links`` (with their parent) but
    dropped from ``joints`` so downstream consumers can iterate controllable
    DoFs without filtering.
    """
    root = ET.fromstring(urdf_xml)

    joint_infos: List[JointInfo] = []
    link_infos: List[LinkInfo] = []

    # Link name -> parent link name (via the joint connecting them).
    child_to_parent: Dict[str, str] = {}
    # Link name -> (xyz, rpy) offset of the connecting joint origin.
    child_to_origin: Dict[str, Tuple[Tuple[float, float, float], Tuple[float, float, float]]] = {}

    for joint_el in root.findall("joint"):
        jname = joint_el.get("name", "")
        jtype = (joint_el.get("type") or "fixed").lower()

        parent_el = joint_el.find("parent")
        child_el = joint_el.find("child")
        parent_link = parent_el.get("link", "") if parent_el is not None else ""
        child_link = child_el.get("link", "") if child_el is not None else ""

        axis_el = joint_el.find("axis")
        axis_xyz = _parse_xyz(axis_el.get("xyz") if axis_el is not None else None,
                              (0.0, 0.0, 1.0))

        origin_el = joint_el.find("origin")
        origin_xyz = _parse_xyz(origin_el.get("xyz") if origin_el is not None else None,
                                (0.0, 0.0, 0.0))
        origin_rpy = _parse_xyz(origin_el.get("rpy") if origin_el is not None else None,
                                (0.0, 0.0, 0.0))

        limit_el = joint_el.find("limit")
        lim_lower = _float_attr(limit_el, "lower", 0.0)
        lim_upper = _float_attr(limit_el, "upper", 0.0)
        lim_effort = _float_attr(limit_el, "effort", 0.0)
        lim_velocity = _float_attr(limit_el, "velocity", 0.0)

        if child_link:
            child_to_parent[child_link] = parent_link
            child_to_origin[child_link] = (origin_xyz, origin_rpy)

        if jtype == "fixed":
            # Still useful for TF static broadcasting, but not a DoF.
            continue

        joint_infos.append(
            JointInfo(
                name=jname,
                dof_idx=None,  # filled in by caller against the Genesis entity
                type=jtype,
                parent_link=parent_link,
                child_link=child_link,
                axis_xyz=axis_xyz,
                limit_lower=lim_lower,
                limit_upper=lim_upper,
                effort_limit=lim_effort,
                velocity_limit=lim_velocity,
            )
        )

    for link_el in root.findall("link"):
        lname = link_el.get("name", "")
        parent = child_to_parent.get(lname)
        origin_xyz, origin_rpy = child_to_origin.get(lname, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        link_infos.append(
            LinkInfo(
                name=lname,
                parent_name=parent,
                local_xyz=origin_xyz,
                local_rpy=origin_rpy,
                gs_link_idx=None,
            )
        )

    # Base link = the link that is nobody's child (first one found).
    base_link_name: Optional[str] = None
    for li in link_infos:
        if li.parent_name is None:
            base_link_name = li.name
            break
    if base_link_name is None and link_infos:
        base_link_name = link_infos[0].name

    return root, joint_infos, link_infos, base_link_name


# ---------------------------------------------------------------------------
# Genesis introspection helpers
# ---------------------------------------------------------------------------
def _joint_dof_idx(gs_joint: Any) -> Optional[int]:
    """Return the first local DoF index of a Genesis joint, or None."""
    dofs = getattr(gs_joint, "dofs_idx_local", None)
    if dofs is None:
        return None
    try:
        # ``dofs_idx_local`` is a list-like; single-DoF joints have one entry.
        return int(dofs[0])
    except (TypeError, IndexError, ValueError):
        return None


def _safe_attr(obj: Any, name: str, default: Any = None) -> Any:
    try:
        return getattr(obj, name, default)
    except Exception:
        return default


def _attach_gs_indices(record: EntityRecord) -> None:
    """Populate ``JointInfo.dof_idx`` and ``LinkInfo.gs_link_idx`` from the entity.

    Best-effort: any attribute lookup failure is silently skipped so the
    registry remains useful even if Genesis' internals change.
    """
    entity = record.entity

    # --- Joints: match by name --------------------------------------------
    gs_joints = _safe_attr(entity, "joints", None) or []
    by_name: Dict[str, Any] = {}
    for gj in gs_joints:
        jname = _safe_attr(gj, "name", None)
        if isinstance(jname, str):
            by_name[jname] = gj

    for ji in record.joints:
        gj = by_name.get(ji.name)
        if gj is None:
            continue
        idx = _joint_dof_idx(gj)
        if idx is not None:
            ji.dof_idx = idx

    # --- Links: match by name ---------------------------------------------
    gs_links = _safe_attr(entity, "links", None) or []
    link_start = int(_safe_attr(entity, "link_start", 0) or 0)
    link_name_to_idx: Dict[str, int] = {}
    for i, gl in enumerate(gs_links):
        lname = _safe_attr(gl, "name", None)
        if isinstance(lname, str):
            link_name_to_idx[lname] = i

    for li in record.links:
        idx = link_name_to_idx.get(li.name)
        if idx is not None:
            li.gs_link_idx = link_start + idx


def _enumerate_from_entity(record: EntityRecord) -> None:
    """Populate ``record.joints`` and ``record.links`` directly from the entity.

    Used when no URDF XML was supplied to :meth:`EntityRegistry.register`.
    Unknown fields (axis, limits) default to zeros.
    """
    entity = record.entity

    gs_links = _safe_attr(entity, "links", None) or []
    link_start = int(_safe_attr(entity, "link_start", 0) or 0)

    # First build a map from absolute link index -> name, so we can resolve
    # the ``parent_idx`` the Genesis link stores (which is absolute).
    abs_idx_to_name: Dict[int, str] = {}
    for i, gl in enumerate(gs_links):
        lname = _safe_attr(gl, "name", None) or "link_" + str(i)
        abs_idx_to_name[link_start + i] = lname

    for i, gl in enumerate(gs_links):
        lname = _safe_attr(gl, "name", None) or "link_" + str(i)
        parent_idx = _safe_attr(gl, "parent_idx", -1)
        try:
            parent_idx = int(parent_idx)
        except (TypeError, ValueError):
            parent_idx = -1
        parent_name = abs_idx_to_name.get(parent_idx) if parent_idx >= 0 else None
        record.links.append(
            LinkInfo(
                name=lname,
                parent_name=parent_name,
                local_xyz=(0.0, 0.0, 0.0),
                local_rpy=(0.0, 0.0, 0.0),
                gs_link_idx=link_start + i,
            )
        )

    if record.base_link_name is None:
        for li in record.links:
            if li.parent_name is None:
                record.base_link_name = li.name
                break
        if record.base_link_name is None and record.links:
            record.base_link_name = record.links[0].name

    gs_joints = _safe_attr(entity, "joints", None) or []
    for gj in gs_joints:
        jname = _safe_attr(gj, "name", None) or ""
        jtype_raw = _safe_attr(gj, "type", None)
        jtype = _gs_joint_type_to_str(jtype_raw)
        if jtype == "fixed":
            continue
        parent_link = ""
        child_link = ""
        # Genesis' RigidJoint exposes ``link`` (child) and its parent via the
        # link graph. These attributes may differ across versions; stay defensive.
        child_obj = _safe_attr(gj, "link", None) or _safe_attr(gj, "child_link", None)
        if child_obj is not None:
            child_link = _safe_attr(child_obj, "name", "") or ""
            parent_obj_idx = _safe_attr(child_obj, "parent_idx", -1)
            try:
                parent_obj_idx = int(parent_obj_idx)
            except (TypeError, ValueError):
                parent_obj_idx = -1
            if parent_obj_idx >= 0:
                parent_link = abs_idx_to_name.get(parent_obj_idx, "") or ""

        record.joints.append(
            JointInfo(
                name=jname,
                dof_idx=_joint_dof_idx(gj),
                type=jtype,
                parent_link=parent_link,
                child_link=child_link,
                axis_xyz=(0.0, 0.0, 1.0),
                limit_lower=0.0,
                limit_upper=0.0,
                effort_limit=0.0,
                velocity_limit=0.0,
            )
        )


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------
class EntityRegistry:
    """Typed index of registered Genesis entities.

    Created lazily by :class:`~genesis_ros.node.GenesisRosBridge` the first
    time anyone calls :meth:`GenesisRosBridge.register_entity`. Publishers
    and subscribers read from the registry but never write to it directly
    -- sensor/camera registration goes through the explicit methods on the
    registry so ordering is deterministic across groups.
    """

    def __init__(self, bridge: Any = None) -> None:
        self._bridge = bridge
        self._records: Dict[str, EntityRecord] = {}

    # ------------------------------------------------------------------- API
    def register(
        self,
        entity: Any,
        *,
        name: str,
        urdf_xml: Optional[str] = None,
        is_mobile_base: bool = False,
        cmd_vel_cfg: Optional[dict] = None,
        base_link: Optional[str] = None,
        ground_truth: bool = False,
        **_unused: Any,
    ) -> EntityRecord:
        """Register ``entity`` under ``name`` and return its record.

        Parameters
        ----------
        entity:
            The Genesis entity handle (typically a ``RigidEntity``).
        name:
            Unique identifier for this entity in the bridge.
        urdf_xml:
            Optional raw URDF XML. If supplied, joints/links are populated
            by parsing the URDF; otherwise they are enumerated from
            ``entity.joints`` / ``entity.links``.
        is_mobile_base:
            Tag that tells the ``cmd_vel`` subscriber this entity should be
            treated as a differential-drive / planar base.
        cmd_vel_cfg:
            Optional configuration dict forwarded to the mobile-base
            subscriber (wheel radius, track width, etc.).
        base_link:
            Optional explicit base link name. When omitted, the first URDF
            link without a parent (or the first entity link) is used.
        """
        record = EntityRecord(
            name=name,
            entity=entity,
            urdf_xml=urdf_xml,
            is_mobile_base=bool(is_mobile_base),
            cmd_vel_cfg=dict(cmd_vel_cfg) if cmd_vel_cfg else None,
            base_link_name=base_link,
            ground_truth=bool(ground_truth),
        )

        if urdf_xml:
            try:
                root, joints, links, parsed_base = _parse_urdf(urdf_xml)
                record.urdf_root = root
                record.joints = joints
                record.links = links
                if record.base_link_name is None:
                    record.base_link_name = parsed_base
                _attach_gs_indices(record)
            except ET.ParseError:
                # Fall back to entity introspection if the URDF is malformed.
                _enumerate_from_entity(record)
        else:
            _enumerate_from_entity(record)

        self._records[name] = record
        return record

    def register_sensor(self, entity_name: str, sensor_name: str, sensor: Any) -> None:
        """Attach a sensor handle to an existing entity record."""
        record = self._records.get(entity_name)
        if record is None:
            raise KeyError("unknown entity: " + repr(entity_name))
        record.sensors[sensor_name] = sensor

    def register_camera(
        self,
        entity_name: str,
        cam_name: str,
        camera: Any,
        parent_link: Optional[str] = None,
    ) -> None:
        """Attach a camera handle (and its parent link) to an entity record."""
        record = self._records.get(entity_name)
        if record is None:
            raise KeyError("unknown entity: " + repr(entity_name))
        record.cameras[cam_name] = (camera, parent_link)

    # -------------------------------------------------------- dict-like bits
    def get(self, name: str, default: Any = None) -> Any:
        return self._records.get(name, default)

    def __getitem__(self, name: str) -> EntityRecord:
        return self._records[name]

    def __contains__(self, name: object) -> bool:
        return name in self._records

    def __iter__(self) -> Iterator[EntityRecord]:
        return iter(self._records.values())

    def __len__(self) -> int:
        return len(self._records)

    @property
    def names(self) -> List[str]:
        return list(self._records.keys())


__all__ = [
    "EntityRecord",
    "EntityRegistry",
    "JointInfo",
    "LinkInfo",
]
