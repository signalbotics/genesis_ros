"""Contact publisher -- per-link WrenchStamped plus optional detail stream.

Owned by Group 5. Reads ``entity.get_links_net_contact_force()`` for every
registered record and publishes a ``geometry_msgs/WrenchStamped`` per link.
Optionally emits a ``diagnostic_msgs/DiagnosticArray`` summary derived from
``entity.get_contacts()``.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from std_msgs.msg import Header  # noqa: F401
    from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
    from diagnostic_msgs.msg import (
        DiagnosticArray,
        DiagnosticStatus,
        KeyValue,
    )
    _ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - ROS 2 not installed
    _ROS_AVAILABLE = False

import numpy as np

from genesis_ros.node import GenesisPublisher
from genesis_ros.qos import SENSOR_QOS
from genesis_ros import conversions as conv


def _as_str(val) -> str:
    try:
        arr = conv._as_numpy(val).reshape(-1)
        if arr.size == 1:
            return str(arr.item())
        return str(arr.tolist())
    except Exception:
        return str(val)


class ContactPublisher(GenesisPublisher):
    """Publish per-link contact wrenches and optional detailed contact list.

    * ``/{record}/contacts/{link_name}`` -- ``geometry_msgs/WrenchStamped``
      (one publisher per link, lazily created).
    * If ``cfg['publish_detail']`` is truthy, also publishes
      ``/{record}/contacts_detail`` as ``diagnostic_msgs/DiagnosticArray``.
    """

    def __init__(self, node, scene, registry, cfg=None):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "ContactPublisher requires the ROS 2 Python stack"
                " (rclpy, geometry_msgs, diagnostic_msgs)."
            )
        super().__init__(node, scene, registry, cfg)
        self._env_idx = int(self.cfg.get("env_idx", 0))
        self._publish_detail = bool(self.cfg.get("publish_detail", False))
        self._wrench_pubs = {}   # (record_name, link_name) -> publisher
        self._detail_pubs = {}   # record_name -> publisher

    def _get_wrench_pub(self, record_name, link_name):
        key = (record_name, link_name)
        pub = self._wrench_pubs.get(key)
        if pub is None:
            topic = "/" + record_name + "/contacts/" + link_name
            pub = self.node.create_publisher(
                WrenchStamped, topic, SENSOR_QOS
            )
            self._wrench_pubs[key] = pub
        return pub

    def _get_detail_pub(self, record_name):
        pub = self._detail_pubs.get(record_name)
        if pub is None:
            topic = "/" + record_name + "/contacts_detail"
            pub = self.node.create_publisher(
                DiagnosticArray, topic, SENSOR_QOS
            )
            self._detail_pubs[record_name] = pub
        return pub

    def _link_name(self, link, fallback_idx: int) -> str:
        name = getattr(link, "name", None)
        if name:
            return str(name)
        return "link_" + str(fallback_idx)

    def step(self, sim_time) -> None:
        for record_name, record in self.registry.items():
            entity = getattr(record, "entity", None) or getattr(
                record, "gs_entity", None
            )
            if entity is None:
                continue

            try:
                forces = entity.get_links_net_contact_force()
            except Exception as exc:
                self.node.get_logger().warning(
                    "ContactPublisher: " + record_name
                    + " get_links_net_contact_force() failed: " + repr(exc)
                )
                forces = None

            if forces is not None:
                f_arr = conv._as_numpy(conv.select_env(forces, self._env_idx))
                if f_arr.ndim == 1 and f_arr.shape[-1] == 3:
                    f_arr = f_arr.reshape(1, 3)
                elif f_arr.ndim >= 2 and f_arr.shape[-1] == 3:
                    f_arr = f_arr.reshape(-1, 3)
                else:
                    f_arr = None

                links = getattr(record, "links", None)
                if links is None:
                    links = getattr(entity, "links", None) or []

                if f_arr is not None:
                    n = f_arr.shape[0]
                    for i in range(n):
                        link = links[i] if i < len(links) else None
                        link_name = self._link_name(link, i)
                        frame_id = record_name + "/" + link_name
                        msg = WrenchStamped()
                        msg.header.stamp = sim_time
                        msg.header.frame_id = frame_id
                        msg.wrench = Wrench(
                            force=Vector3(
                                x=float(f_arr[i, 0]),
                                y=float(f_arr[i, 1]),
                                z=float(f_arr[i, 2]),
                            ),
                            torque=Vector3(x=0.0, y=0.0, z=0.0),
                        )
                        self._get_wrench_pub(record_name, link_name).publish(msg)

            if not self._publish_detail:
                continue

            try:
                contacts = entity.get_contacts()
            except Exception as exc:
                self.node.get_logger().warning(
                    "ContactPublisher: " + record_name
                    + " get_contacts() failed: " + repr(exc)
                )
                continue

            positions = contacts.get("position")
            force_a = contacts.get("force_a")
            force_b = contacts.get("force_b")
            link_a = contacts.get("link_a")
            link_b = contacts.get("link_b")

            def _sel(x):
                if x is None:
                    return None
                return conv._as_numpy(conv.select_env(x, self._env_idx))

            pos = _sel(positions)
            fa = _sel(force_a)
            fb = _sel(force_b)
            la = _sel(link_a)
            lb = _sel(link_b)

            n = 0
            for arr in (pos, fa, la, lb):
                if arr is not None:
                    n = max(n, arr.shape[0] if arr.ndim >= 1 else 1)

            array_msg = DiagnosticArray()
            array_msg.header.stamp = sim_time
            array_msg.header.frame_id = record_name
            statuses = []
            for i in range(n):
                kv = []
                if la is not None and i < la.shape[0]:
                    kv.append(KeyValue(key="link_a", value=_as_str(la[i])))
                if lb is not None and i < lb.shape[0]:
                    kv.append(KeyValue(key="link_b", value=_as_str(lb[i])))
                if pos is not None and i < pos.shape[0]:
                    p = pos[i].reshape(-1)
                    kv.append(KeyValue(key="pos_x", value=str(float(p[0]))))
                    kv.append(KeyValue(key="pos_y", value=str(float(p[1]))))
                    kv.append(KeyValue(key="pos_z", value=str(float(p[2]))))
                if fa is not None and i < fa.shape[0]:
                    fv = fa[i].reshape(-1)
                    kv.append(KeyValue(key="force_x", value=str(float(fv[0]))))
                    kv.append(KeyValue(key="force_y", value=str(float(fv[1]))))
                    kv.append(KeyValue(key="force_z", value=str(float(fv[2]))))
                if fb is not None and i < fb.shape[0]:
                    fv = fb[i].reshape(-1)
                    kv.append(
                        KeyValue(key="force_b_x", value=str(float(fv[0])))
                    )
                    kv.append(
                        KeyValue(key="force_b_y", value=str(float(fv[1])))
                    )
                    kv.append(
                        KeyValue(key="force_b_z", value=str(float(fv[2])))
                    )
                status = DiagnosticStatus()
                status.level = DiagnosticStatus.OK
                status.name = record_name + "/contact_" + str(i)
                status.message = "contact"
                status.hardware_id = record_name
                status.values = kv
                statuses.append(status)
            array_msg.status = statuses
            self._get_detail_pub(record_name).publish(array_msg)


__all__ = ["ContactPublisher"]
