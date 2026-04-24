"""Shared-memory hardware bridge.

Companion to the native ``genesis_ros2_control::GenesisSystem``
``hardware_interface::SystemInterface`` plugin. Maps a ``/dev/shm``
region shared with the plugin; state written here is read by
``controller_manager``'s ``read()`` cycle, commands written by
controllers land here and are dispatched into
``entity.control_dofs_*`` before the next ``scene.step()``.

The layout mirrors ``genesis_ros2_control/include/genesis_ros2_control/
shm_protocol.h`` -- keep the two in lockstep. ABI version is checked at
open time.

Protocol: seqlock. Writer bumps seq to odd, writes payload, bumps to
even. Reader grabs seq, confirms even, copies, re-reads seq, confirms
unchanged. No locks.
"""
from __future__ import annotations

import ctypes
import mmap
import os
import threading
from typing import List, Optional, Sequence

import numpy as np

from genesis_ros.node import GenesisPublisher, GenesisSubscriber
from genesis_ros import conversions as conv


# ------------------------------------------------------------------- constants
# Keep these in sync with shm_protocol.h.
SHM_MAGIC = 0x474E5253  # 'GNRS'
SHM_VERSION = 1
SHM_MAX_JOINTS = 64
SHM_JOINT_NAME_LEN = 64
SHM_CACHE_LINE = 64

CMD_POSITION = 1 << 0
CMD_VELOCITY = 1 << 1
CMD_EFFORT = 1 << 2

SHM_PATH_PREFIX = "genesis_ros2_control."


# ------------------------------------------------------------------ C structs
class _ShmHandshake(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("version", ctypes.c_uint32),
        ("n_joints", ctypes.c_uint32),
        ("_pad0", ctypes.c_uint32),
        ("joint_names",
         (ctypes.c_char * SHM_JOINT_NAME_LEN) * SHM_MAX_JOINTS),
        ("boot_stamp_ns", ctypes.c_uint64),
        ("_pad1", ctypes.c_uint8 * SHM_CACHE_LINE),
    ]


class _ShmState(ctypes.Structure):
    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("stamp_ns", ctypes.c_uint64),
        ("position", ctypes.c_double * SHM_MAX_JOINTS),
        ("velocity", ctypes.c_double * SHM_MAX_JOINTS),
        ("effort",   ctypes.c_double * SHM_MAX_JOINTS),
        ("_pad",     ctypes.c_uint8 * SHM_CACHE_LINE),
    ]


class _ShmCommand(ctypes.Structure):
    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("cmd_mask", ctypes.c_uint32 * SHM_MAX_JOINTS),
        ("position", ctypes.c_double * SHM_MAX_JOINTS),
        ("velocity", ctypes.c_double * SHM_MAX_JOINTS),
        ("effort",   ctypes.c_double * SHM_MAX_JOINTS),
        ("_pad",     ctypes.c_uint8 * SHM_CACHE_LINE),
    ]


class _Shm(ctypes.Structure):
    _fields_ = [
        ("handshake", _ShmHandshake),
        ("state", _ShmState),
        ("command", _ShmCommand),
    ]


SHM_SIZE = ctypes.sizeof(_Shm)


def _shm_path(robot_name: str) -> str:
    return "/dev/shm/" + SHM_PATH_PREFIX + robot_name


def _pad_name(name: str) -> bytes:
    raw = name.encode("utf-8")[: SHM_JOINT_NAME_LEN - 1]
    return raw + b"\x00" * (SHM_JOINT_NAME_LEN - len(raw))


# ------------------------------------------------------------------- writer
class ShmHardwareBridge(GenesisPublisher, GenesisSubscriber):
    """Dual-role bridge: publish joint state + ingest joint commands.

    Lives in the Python process alongside ``genesis_bridge``. Registers
    with the bridge as BOTH a publisher (runs after ``scene.step()`` to
    write current state) and a subscriber (runs before ``scene.step()``
    to dispatch queued commands into the entity).

    cfg keys:
      robot     (required) entity name registered in the bridge
      joints    (optional) ordered list of joint names to expose. If
                omitted, uses the registry record's joint ordering. Must
                be <= SHM_MAX_JOINTS.
      kp, kv    (optional) PD gains applied once at startup via
                entity.set_dofs_kp / set_dofs_kv for position interface.
    """

    # Resolve ambiguous MRO: GenesisPublisher + GenesisSubscriber both
    # declare abstract methods. We implement both.
    def __init__(self, node, scene, registry, cfg=None):
        GenesisPublisher.__init__(self, node, scene, registry, cfg)
        self._lock = threading.Lock()

        self._robot_name = str(self.cfg.get("robot") or "")
        if not self._robot_name:
            raise ValueError("ShmHardwareBridge: cfg['robot'] is required")

        record = registry.get(self._robot_name) if registry is not None else None
        if record is None:
            raise KeyError(
                "ShmHardwareBridge: unknown entity "
                + repr(self._robot_name)
            )
        self._record = record
        self._entity = getattr(record, "entity", None) or getattr(
            record, "gs_entity", None
        )
        if self._entity is None:
            raise RuntimeError(
                "ShmHardwareBridge: entity handle missing on record"
            )

        self._joint_names: List[str] = self._resolve_joint_names(record)
        self._dof_indices: List[int] = self._resolve_dof_indices(
            record, self._joint_names
        )
        if len(self._joint_names) > SHM_MAX_JOINTS:
            raise ValueError(
                "ShmHardwareBridge: too many joints ("
                + str(len(self._joint_names))
                + " > " + str(SHM_MAX_JOINTS) + ")"
            )

        self._n_joints = len(self._joint_names)
        self._shm_path = _shm_path(self._robot_name)
        self._shm_fd, self._shm_view, self._shm = self._open_shm()

        self._write_handshake()
        self._apply_gains()

    # ------------------------------------------------------- joint discovery
    @staticmethod
    def _resolve_joint_names(record) -> List[str]:
        joints = getattr(record, "joints", None) or []
        names = []
        for j in joints:
            # Skip fixed joints -- they have no DoF / can't be controlled.
            jtype = getattr(j, "type", None)
            if jtype in ("fixed", None):
                # Some joint objects don't carry a type; keep them only if
                # they expose a dof_idx.
                if getattr(j, "dof_idx", None) is None:
                    continue
            name = getattr(j, "name", None)
            if name:
                names.append(str(name))
        return names

    @staticmethod
    def _resolve_dof_indices(record, joint_names) -> List[int]:
        joints = {getattr(j, "name", ""): j for j in (getattr(record, "joints", None) or [])}
        out = []
        for n in joint_names:
            j = joints.get(n)
            idx = getattr(j, "dof_idx", None) if j is not None else None
            if idx is None:
                # Fall back to position in joint list; the rigid solver
                # gives a 1:1 mapping between joint list and DoF list for
                # single-DoF (revolute/prismatic) chains.
                idx = len(out)
            out.append(int(idx))
        return out

    # ------------------------------------------------------------ shm plumbing
    def _open_shm(self):
        path = self._shm_path
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            os.ftruncate(fd, SHM_SIZE)
        except OSError:
            # Someone else sized it; trust them if size already matches.
            if os.fstat(fd).st_size < SHM_SIZE:
                os.close(fd)
                raise
        buf = mmap.mmap(
            fd, SHM_SIZE, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE
        )
        shm = _Shm.from_buffer(buf)
        return fd, buf, shm

    def _write_handshake(self):
        import time as _t
        # Zero the whole region so any stale data from a prior run does
        # not leak through.
        ctypes.memset(ctypes.addressof(self._shm), 0, SHM_SIZE)
        self._shm.handshake.magic = SHM_MAGIC
        self._shm.handshake.version = SHM_VERSION
        self._shm.handshake.n_joints = self._n_joints
        for i, name in enumerate(self._joint_names):
            raw = _pad_name(name)
            ctypes.memmove(
                ctypes.addressof(self._shm.handshake.joint_names[i]),
                raw,
                SHM_JOINT_NAME_LEN,
            )
        self._shm.handshake.boot_stamp_ns = int(_t.time_ns())

    def _apply_gains(self):
        kp = self.cfg.get("kp")
        kv = self.cfg.get("kv")
        if kp is not None and hasattr(self._entity, "set_dofs_kp"):
            try:
                self._entity.set_dofs_kp(np.asarray(kp, dtype=np.float64))
            except Exception:
                pass
        if kv is not None and hasattr(self._entity, "set_dofs_kv"):
            try:
                self._entity.set_dofs_kv(np.asarray(kv, dtype=np.float64))
            except Exception:
                pass

    # ------------------------------------------------------------ publishing
    def step(self, sim_time) -> None:
        """Write current joint state into shm (called after scene.step())."""
        entity = self._entity
        try:
            q = conv._as_numpy(entity.get_qpos()).reshape(-1)
            v = conv._as_numpy(entity.get_dofs_velocity()).reshape(-1)
        except Exception:
            return
        # Effort is optional and may not be available on every entity.
        try:
            tau = conv._as_numpy(entity.get_dofs_control_force()).reshape(-1)
        except Exception:
            tau = np.zeros_like(v)

        stamp_ns = int(sim_time.sec) * 1_000_000_000 + int(sim_time.nanosec)

        st = self._shm.state
        # seqlock write: odd -> write -> even.
        st.seq += 1
        st.stamp_ns = stamp_ns
        for i, dof in enumerate(self._dof_indices):
            if dof < q.size:
                st.position[i] = float(q[dof])
            if dof < v.size:
                st.velocity[i] = float(v[dof])
            if dof < tau.size:
                st.effort[i] = float(tau[dof])
        st.seq += 1

    # ------------------------------------------------------------ subscribing
    def apply(self, scene) -> None:
        """Dispatch any queued commands into the entity before
        scene.step() runs."""
        cmd = self._shm.command
        # Snapshot seq, copy out, confirm unchanged (standard seqlock read).
        s1 = cmd.seq
        if (s1 & 1) != 0:
            return  # write in progress; skip this cycle
        masks = list(cmd.cmd_mask[: self._n_joints])
        pos_cmd = list(cmd.position[: self._n_joints])
        vel_cmd = list(cmd.velocity[: self._n_joints])
        eff_cmd = list(cmd.effort[: self._n_joints])
        s2 = cmd.seq
        if s1 != s2:
            return  # torn read; plugin will re-publish next cycle

        entity = self._entity
        # Group by interface so we do one control_dofs_* call per mode.
        pos_dofs, pos_vals = [], []
        vel_dofs, vel_vals = [], []
        eff_dofs, eff_vals = [], []
        for i, mask in enumerate(masks):
            if mask == 0:
                continue
            dof = self._dof_indices[i]
            if mask & CMD_POSITION:
                pos_dofs.append(dof); pos_vals.append(pos_cmd[i])
            if mask & CMD_VELOCITY:
                vel_dofs.append(dof); vel_vals.append(vel_cmd[i])
            if mask & CMD_EFFORT:
                eff_dofs.append(dof); eff_vals.append(eff_cmd[i])

        if pos_dofs and hasattr(entity, "control_dofs_position"):
            try:
                entity.control_dofs_position(np.asarray(pos_vals), pos_dofs)
            except Exception:
                pass
        if vel_dofs and hasattr(entity, "control_dofs_velocity"):
            try:
                entity.control_dofs_velocity(np.asarray(vel_vals), vel_dofs)
            except Exception:
                pass
        if eff_dofs and hasattr(entity, "control_dofs_force"):
            try:
                entity.control_dofs_force(np.asarray(eff_vals), eff_dofs)
            except Exception:
                pass

    # ---------------------------------------------------------------- close
    def close(self) -> None:
        try:
            del self._shm
        except Exception:
            pass
        try:
            self._shm_view.close()
        except Exception:
            pass
        try:
            os.close(self._shm_fd)
        except Exception:
            pass


def register_shm_bridge(
    bridge,
    robot: str,
    joints: Optional[Sequence[str]] = None,
    kp: Optional[Sequence[float]] = None,
    kv: Optional[Sequence[float]] = None,
) -> ShmHardwareBridge:
    """Convenience wrapper: build the hardware bridge and register it as
    both a publisher (post-step state write) and subscriber (pre-step
    command ingest)."""
    cfg = {"robot": robot}
    if joints is not None:
        cfg["joints"] = list(joints)
    if kp is not None:
        cfg["kp"] = list(kp)
    if kv is not None:
        cfg["kv"] = list(kv)
    shm = ShmHardwareBridge(bridge.node, bridge.scene, bridge.registry, cfg)
    bridge.register_publisher(shm)
    bridge.register_subscriber(shm)
    return shm


__all__ = [
    "ShmHardwareBridge",
    "register_shm_bridge",
    "SHM_MAGIC",
    "SHM_VERSION",
    "SHM_MAX_JOINTS",
    "SHM_JOINT_NAME_LEN",
    "SHM_SIZE",
    "CMD_POSITION",
    "CMD_VELOCITY",
    "CMD_EFFORT",
]
