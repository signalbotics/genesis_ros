"""``CmdVelSubscriber`` -- feed ``geometry_msgs/Twist`` into Genesis mobile bases.

One subscriber instance owns the full set of ``/{robot}/cmd_vel`` topics for
every entity in the registry whose ``cmd_vel_cfg`` is populated (typically set
when ``register_entity(..., is_mobile_base=True)`` is called).

Two control modes are supported, selected via ``cmd_vel_cfg['mode']``:

* ``'free_base'``   -- the mobile base is a free-flying body with 6 base DOFs.
                       We write the full linear+angular twist straight to those
                       DOFs with :meth:`set_dofs_velocity`.
* ``'diff_drive'``  -- a differential-drive wheeled base. We run the standard
                       diff-drive IK to recover left/right wheel angular
                       velocities and command them with
                       :meth:`control_dofs_velocity`.

Incoming Twist messages arrive on the rclpy executor thread; we stage them in
a lock-guarded dict and flush on the main thread in :meth:`apply`, matching the
threading rule in ``ROS2_INTEGRATION_PLAN.md``.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from rclpy.node import Node  # noqa: F401
    from geometry_msgs.msg import Twist
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

import threading

from genesis_ros.node import GenesisSubscriber
from genesis_ros.qos import STATE_QOS
from genesis_ros import conversions as conv  # noqa: F401


class CmdVelSubscriber(GenesisSubscriber):
    """Stage /cmd_vel twist commands and flush them on the next main-thread tick."""

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "CmdVelSubscriber requires rclpy / geometry_msgs -- the ROS 2 "
                "runtime was not importable."
            )
        super().__init__(node, scene, registry, cfg)

        self._staged: dict = {}
        self._lock = threading.Lock()
        self._subscriptions: list = []

        # Iterate the registry and hook one subscription per mobile base.
        records = self._iter_records(registry)
        for record in records:
            cmd_vel_cfg = getattr(record, "cmd_vel_cfg", None)
            if not cmd_vel_cfg:
                continue
            name = getattr(record, "name", None)
            if not name:
                continue
            topic = "/" + name + "/cmd_vel"
            sub = node.create_subscription(
                Twist,
                topic,
                self._make_callback(name),
                STATE_QOS,
            )
            self._subscriptions.append(sub)

    # ---------------------------------------------------------- registry helper
    @staticmethod
    def _iter_records(registry):
        """Yield records from either an :class:`EntityRegistry` or the lazy
        dict-like stand-in in :mod:`genesis_ros.node`.
        """
        if hasattr(registry, "values"):
            try:
                return list(registry.values())
            except Exception:
                pass
        try:
            return list(iter(registry))
        except TypeError:
            return []

    # -------------------------------------------------------------- callback
    def _make_callback(self, name):
        # Bind ``name`` into the closure; the executor may deliver messages
        # from multiple subscriptions to arbitrary threads.
        def _cb(msg, name=name):
            with self._lock:
                self._staged[name] = msg
        return _cb

    # --------------------------------------------------------------- apply
    def apply(self, scene):
        with self._lock:
            pending = self._staged
            self._staged = {}

        if not pending:
            return

        for name, twist in pending.items():
            record = self._registry_get(name)
            if record is None:
                continue
            cfg = getattr(record, "cmd_vel_cfg", None)
            if not cfg:
                continue
            entity = getattr(record, "entity", None) or getattr(record, "gs_entity", None)
            if entity is None:
                continue

            mode = cfg.get("mode", "diff_drive")
            v = float(twist.linear.x)
            w = float(twist.angular.z)

            if mode == "free_base":
                try:
                    entity.set_dofs_velocity(
                        velocity=[
                            float(twist.linear.x),
                            float(twist.linear.y),
                            float(twist.linear.z),
                            float(twist.angular.x),
                            float(twist.angular.y),
                            float(twist.angular.z),
                        ],
                        dofs_idx_local=cfg["base_dof_idx"],
                    )
                except Exception as exc:
                    self._warn(
                        "CmdVelSubscriber free_base set_dofs_velocity failed "
                        "for " + repr(name) + ": " + repr(exc)
                    )
            elif mode == "diff_drive":
                r = float(cfg["wheel_radius"])
                b = float(cfg["wheel_base"])
                v_l = (v - 0.5 * w * b) / r
                v_r = (v + 0.5 * w * b) / r
                try:
                    joints = list(getattr(record, "joints", []) or [])
                    left_dof = next(
                        j.dof_idx for j in joints if j.name == cfg["wheel_joint_left"]
                    )
                    right_dof = next(
                        j.dof_idx for j in joints if j.name == cfg["wheel_joint_right"]
                    )
                except StopIteration:
                    self._warn(
                        "CmdVelSubscriber diff_drive: wheel joint not found on "
                        + repr(name)
                    )
                    continue
                try:
                    entity.control_dofs_velocity(
                        velocity=[v_l, v_r],
                        dofs_idx_local=[left_dof, right_dof],
                    )
                except Exception as exc:
                    self._warn(
                        "CmdVelSubscriber diff_drive control_dofs_velocity "
                        "failed for " + repr(name) + ": " + repr(exc)
                    )
            else:
                self._warn(
                    "CmdVelSubscriber unknown mode " + repr(mode)
                    + " for entity " + repr(name)
                )

    # -------------------------------------------------------------- helpers
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


__all__ = ["CmdVelSubscriber"]
