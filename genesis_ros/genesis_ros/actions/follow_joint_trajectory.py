"""``FollowJointTrajectoryAction`` -- control_msgs FollowJointTrajectory server.

One action server is created per registered entity with a non-empty ``joints``
list. Goals are validated against that list, linearly interpolated to the
bridge step rate, staged into a lock-guarded per-entity buffer, and flushed on
the main thread in :meth:`apply` via
``entity.control_dofs_position_velocity`` (see ``rigid_entity.py:3606``).

Feedback is published at 10 Hz; goals complete when all trajectory points have
been consumed *and* the current joint positions lie within ``goal_tolerance``.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from rclpy.node import Node  # noqa: F401
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from control_msgs.action import FollowJointTrajectory
    from builtin_interfaces.msg import Time as _RosTime  # noqa: F401
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

import threading
import time

from genesis_ros.node import GenesisSubscriber
from genesis_ros.qos import STATE_QOS  # noqa: F401
from genesis_ros import conversions as conv  # noqa: F401


def _duration_to_sec(duration) -> float:
    """Accept builtin_interfaces/Duration or rclpy Duration; return float sec."""
    if duration is None:
        return 0.0
    if hasattr(duration, "sec") and hasattr(duration, "nanosec"):
        return float(duration.sec) + float(duration.nanosec) * 1e-9
    if hasattr(duration, "nanoseconds"):
        return float(duration.nanoseconds) * 1e-9
    try:
        return float(duration)
    except Exception:
        return 0.0


def _linear_interp(t, p_prev, p_next, v_prev, v_next, t_prev, t_next):
    """Linear position + velocity interpolation within a single segment."""
    span = t_next - t_prev
    if span <= 0.0:
        return list(p_next), list(v_next)
    alpha = (t - t_prev) / span
    if alpha < 0.0:
        alpha = 0.0
    elif alpha > 1.0:
        alpha = 1.0
    pos = [float(p_prev[i]) + alpha * (float(p_next[i]) - float(p_prev[i])) for i in range(len(p_prev))]
    if v_prev and v_next and len(v_prev) == len(v_next) == len(p_prev):
        vel = [float(v_prev[i]) + alpha * (float(v_next[i]) - float(v_prev[i])) for i in range(len(v_prev))]
    else:
        vel = [(float(p_next[i]) - float(p_prev[i])) / span for i in range(len(p_prev))]
    return pos, vel


class FollowJointTrajectoryAction(GenesisSubscriber):
    """ActionServer bank for FollowJointTrajectory goals."""

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "FollowJointTrajectoryAction requires rclpy / control_msgs -- "
                "the ROS 2 runtime was not importable."
            )
        super().__init__(node, scene, registry, cfg)

        self._lock = threading.Lock()
        # name -> {"position": [...], "velocity": [...], "dof_idx": [...]}
        self._pending: dict = {}
        self._servers: dict = {}
        # name -> active goal metadata for bookkeeping (start wall time etc).
        self._active: dict = {}

        records = self._iter_records(registry)
        for record in records:
            joints = list(getattr(record, "joints", []) or [])
            if not joints:
                continue
            name = getattr(record, "name", None)
            if not name:
                continue
            topic = "/" + name + "/follow_joint_trajectory"
            server = ActionServer(
                node,
                FollowJointTrajectory,
                topic,
                execute_callback=self._make_execute(name),
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            )
            self._servers[name] = server

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _iter_records(registry):
        if hasattr(registry, "values"):
            try:
                return list(registry.values())
            except Exception:
                pass
        try:
            return list(iter(registry))
        except TypeError:
            return []

    def _registry_get(self, name):
        reg = self.registry
        if reg is None:
            return None
        get = getattr(reg, "get", None)
        if callable(get):
            try:
                return get(name)
            except Exception:
                return None
        try:
            return reg[name]
        except Exception:
            return None

    def _warn(self, message):
        logger = getattr(self.node, "get_logger", None)
        if callable(logger):
            try:
                logger().warning(message)
                return
            except Exception:
                pass

    # -------------------------------------------------------- action plumbing
    def _goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_callback(self, cancel_request):
        return CancelResponse.ACCEPT

    def _make_execute(self, name):
        async def _execute(goal_handle):
            return await self._execute(name, goal_handle)
        return _execute

    async def _execute(self, name, goal_handle):
        record = self._registry_get(name)
        result = FollowJointTrajectory.Result()

        if record is None:
            self._warn("FollowJointTrajectory: no record for " + name)
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        joint_records = list(getattr(record, "joints", []) or [])
        joint_name_to_dof = {j.name: j.dof_idx for j in joint_records}

        goal = goal_handle.request
        trajectory = goal.trajectory
        goal_names = list(trajectory.joint_names)

        missing = [n for n in goal_names if n not in joint_name_to_dof]
        if missing:
            self._warn(
                "FollowJointTrajectory: unknown joints on " + name
                + ": " + repr(missing)
            )
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        points = list(trajectory.points)
        if not points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        dof_idx = [joint_name_to_dof[n] for n in goal_names]
        tolerances = list(goal.goal_tolerance or [])
        tol_by_name = {}
        for tol in tolerances:
            tol_by_name[tol.name] = float(tol.position) if tol.position > 0 else 1e-3

        seg_times = [_duration_to_sec(p.time_from_start) for p in points]
        # Ensure monotonic non-decreasing to avoid divide-by-zero below.
        for i in range(1, len(seg_times)):
            if seg_times[i] <= seg_times[i - 1]:
                seg_times[i] = seg_times[i - 1] + 1e-3

        start_wall = time.monotonic()
        final_time = seg_times[-1]
        last_feedback_wall = 0.0

        entity = getattr(record, "entity", None) or getattr(record, "gs_entity", None)

        with self._lock:
            self._active[name] = {"start_wall": start_wall, "final_time": final_time}

        import asyncio

        try:
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    with self._lock:
                        self._pending.pop(name, None)
                        self._active.pop(name, None)
                    return result

                now = time.monotonic() - start_wall

                # Find the segment that contains ``now``.
                idx = 0
                for i in range(len(seg_times) - 1):
                    if now < seg_times[i + 1]:
                        idx = i
                        break
                else:
                    idx = len(points) - 1

                if now >= final_time:
                    pos = list(points[-1].positions)
                    vel = list(points[-1].velocities or [0.0] * len(pos))
                else:
                    p_prev = list(points[idx].positions)
                    p_next = list(points[min(idx + 1, len(points) - 1)].positions)
                    v_prev = list(points[idx].velocities or [])
                    v_next = list(points[min(idx + 1, len(points) - 1)].velocities or [])
                    t_prev = seg_times[idx]
                    t_next = seg_times[min(idx + 1, len(points) - 1)]
                    pos, vel = _linear_interp(now, p_prev, p_next, v_prev, v_next, t_prev, t_next)

                with self._lock:
                    self._pending[name] = {
                        "position": pos,
                        "velocity": vel,
                        "dof_idx": dof_idx,
                    }

                # Feedback at ~10 Hz.
                wall_now = time.monotonic()
                if wall_now - last_feedback_wall > 0.1:
                    last_feedback_wall = wall_now
                    fb = FollowJointTrajectory.Feedback()
                    fb.joint_names = goal_names
                    fb.desired.positions = [float(v) for v in pos]
                    fb.desired.velocities = [float(v) for v in vel]
                    current_positions = self._read_current_positions(entity, dof_idx)
                    if current_positions is not None:
                        fb.actual.positions = current_positions
                        fb.error.positions = [
                            float(pos[i]) - current_positions[i] for i in range(len(pos))
                        ]
                    try:
                        goal_handle.publish_feedback(fb)
                    except Exception:
                        pass

                if now >= final_time:
                    current_positions = self._read_current_positions(entity, dof_idx)
                    if current_positions is not None:
                        ok = True
                        for i, jname in enumerate(goal_names):
                            tol = tol_by_name.get(jname, 1e-2)
                            if abs(current_positions[i] - float(points[-1].positions[i])) > tol:
                                ok = False
                                break
                        if ok:
                            break
                    else:
                        break

                await asyncio.sleep(0.02)
        finally:
            with self._lock:
                self._pending.pop(name, None)
                self._active.pop(name, None)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def _read_current_positions(self, entity, dof_idx):
        if entity is None or not hasattr(entity, "get_qpos"):
            return None
        try:
            qpos = conv._as_numpy(entity.get_qpos()).reshape(-1)
            return [float(qpos[i]) for i in dof_idx]
        except Exception:
            return None

    # ---------------------------------------------------------- main-thread
    def apply(self, scene):
        with self._lock:
            commands = list(self._pending.items())

        if not commands:
            return

        for name, cmd in commands:
            record = self._registry_get(name)
            if record is None:
                continue
            entity = getattr(record, "entity", None) or getattr(record, "gs_entity", None)
            if entity is None:
                continue
            try:
                entity.control_dofs_position_velocity(
                    position=cmd["position"],
                    velocity=cmd["velocity"],
                    dofs_idx_local=cmd["dof_idx"],
                )
            except Exception as exc:
                self._warn(
                    "FollowJointTrajectory apply() failed for " + repr(name)
                    + ": " + repr(exc)
                )


__all__ = ["FollowJointTrajectoryAction"]
