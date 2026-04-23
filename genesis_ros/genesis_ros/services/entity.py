"""EntityService -- spawn/delete/get/set entity state over ROS 2.

Four service endpoints:

* /genesis/spawn_urdf       -- add a URDF-backed entity to the scene.
* /genesis/delete_entity    -- mark an entity tombstoned (Genesis has no real delete).
* /genesis/set_entity_state -- overwrite pose / qpos on an entity.
* /genesis/get_entity_state -- return pose / qpos.

We prefer simulation_interfaces (REP-2015) service definitions. When absent we
fall back to std_srvs/Trigger and accept a JSON-encoded argument string through
a node parameter (<service_name>.args).

TODO: switch to simulation_interfaces exclusively once REP-2015 ships final.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from rclpy.node import Node  # noqa: F401
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

import json
import threading

from genesis_ros.node import GenesisSubscriber  # noqa: F401
from genesis_ros.qos import STATE_QOS  # noqa: F401
from genesis_ros import conversions as conv  # noqa: F401


def _resolve_service_types():
    """Return a dict of service types per endpoint and a backend tag."""
    backend = "simulation_interfaces"
    spawn_t = delete_t = set_t = get_t = None

    try:
        from simulation_interfaces.srv import SpawnEntity as spawn_t
    except ImportError:
        spawn_t = None
    try:
        from simulation_interfaces.srv import DeleteEntity as delete_t
    except ImportError:
        delete_t = None
    try:
        from simulation_interfaces.srv import SetEntityState as set_t
    except ImportError:
        set_t = None
    try:
        from simulation_interfaces.srv import GetEntityState as get_t
    except ImportError:
        get_t = None

    if spawn_t is None or delete_t is None or set_t is None or get_t is None:
        from std_srvs.srv import Trigger
        spawn_t = spawn_t or Trigger
        delete_t = delete_t or Trigger
        set_t = set_t or Trigger
        get_t = get_t or Trigger
        backend = "mixed"
    return {
        "spawn": spawn_t,
        "delete": delete_t,
        "set": set_t,
        "get": get_t,
    }, backend


class EntityService:
    """Holds the four entity management service handles."""

    def __init__(self, node, scene, registry, bridge):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "EntityService requires rclpy -- the ROS 2 runtime was not "
                "importable."
            )
        self.node = node
        self.scene = scene
        self.registry = registry
        self.bridge = bridge
        self._lock = threading.Lock()

        types, backend = _resolve_service_types()
        self._types = types
        self._backend = backend

        try:
            for key in ("spawn_urdf", "delete_entity", "set_entity_state", "get_entity_state"):
                param_name = key + ".args"
                if not node.has_parameter(param_name):
                    try:
                        node.declare_parameter(param_name, "")
                    except Exception:
                        pass
        except Exception:
            pass

        self._spawn_srv = node.create_service(
            types["spawn"], "/genesis/spawn_urdf", self._on_spawn
        )
        self._delete_srv = node.create_service(
            types["delete"], "/genesis/delete_entity", self._on_delete
        )
        self._set_srv = node.create_service(
            types["set"], "/genesis/set_entity_state", self._on_set
        )
        self._get_srv = node.create_service(
            types["get"], "/genesis/get_entity_state", self._on_get
        )

    @staticmethod
    def _set_response(response, *, success=True, message=""):
        if hasattr(response, "success"):
            try:
                response.success = bool(success)
            except Exception:
                pass
        if hasattr(response, "message"):
            try:
                response.message = str(message)
            except Exception:
                pass
        return response

    @staticmethod
    def _extract_pose(req, payload):
        pos = None
        quat = None
        fixed = False
        pose = getattr(req, "pose", None)
        if pose is not None:
            p = getattr(pose, "position", None)
            o = getattr(pose, "orientation", None)
            if p is not None:
                pos = [float(p.x), float(p.y), float(p.z)]
            if o is not None:
                quat = [float(o.w), float(o.x), float(o.y), float(o.z)]
        if pos is None and "pose" in payload:
            raw = payload["pose"]
            if isinstance(raw, (list, tuple)) and len(raw) >= 6:
                pos = [float(raw[0]), float(raw[1]), float(raw[2])]
        if quat is None and "quat" in payload:
            raw = payload["quat"]
            if isinstance(raw, (list, tuple)) and len(raw) >= 4:
                quat = [float(v) for v in raw[:4]]
        if hasattr(req, "fixed"):
            fixed = bool(getattr(req, "fixed"))
        elif "fixed" in payload:
            fixed = bool(payload["fixed"])
        return pos, quat, fixed

    def _read_payload(self, request, param_key):
        raw = None
        for attr in ("args", "data", "json"):
            if hasattr(request, attr):
                candidate = getattr(request, attr)
                if isinstance(candidate, str) and candidate:
                    raw = candidate
                    break
        if raw is None:
            try:
                val = self.node.get_parameter(param_key + ".args").value
            except Exception:
                val = None
            if isinstance(val, str) and val:
                raw = val
        if raw is None:
            return {}
        try:
            return json.loads(raw)
        except Exception:
            return {}

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

    def _record_entity(self, record):
        return getattr(record, "entity", None) or getattr(record, "gs_entity", None)

    def _on_spawn(self, request, response):
        payload = self._read_payload(request, "spawn_urdf")

        name = getattr(request, "name", None) or payload.get("name")
        urdf_xml = getattr(request, "urdf_xml", None) or payload.get("urdf_xml")
        urdf_path = getattr(request, "urdf_path", None) or payload.get("urdf_path")
        pos, quat, fixed = self._extract_pose(request, payload)

        if not urdf_xml and not urdf_path:
            return self._set_response(
                response,
                success=False,
                message="spawn_urdf: either urdf_xml or urdf_path is required",
            )

        scene = self.scene
        if getattr(scene, "_built", False):
            return self._set_response(
                response, success=False, message="deferred until reset"
            )

        try:
            import genesis as gs
            morph_kwargs = {}
            if urdf_path:
                morph_kwargs["file"] = urdf_path
            if urdf_xml:
                morph_kwargs["xml"] = urdf_xml
            if pos is not None:
                morph_kwargs["pos"] = pos
            if quat is not None:
                morph_kwargs["quat"] = quat
            if fixed:
                morph_kwargs["fixed"] = True
            morph = gs.morphs.URDF(**morph_kwargs)
            entity = scene.add_entity(morph)
        except Exception as exc:
            self._warn("spawn_urdf failed: " + repr(exc))
            return self._set_response(
                response, success=False, message="spawn failed: " + repr(exc)
            )

        try:
            if name and hasattr(self.bridge, "register_entity"):
                self.bridge.register_entity(
                    entity,
                    name=name,
                    urdf_xml=urdf_xml,
                )
        except Exception as exc:
            self._warn("spawn_urdf registry insert failed: " + repr(exc))

        return self._set_response(response, success=True, message="spawned")

    def _on_delete(self, request, response):
        payload = self._read_payload(request, "delete_entity")
        name = getattr(request, "name", None) or payload.get("name")
        if not name:
            return self._set_response(
                response, success=False, message="delete_entity: name required"
            )

        records = getattr(self.registry, "_records", None)
        if isinstance(records, dict) and name in records:
            try:
                setattr(records[name], "deleted", True)
                return self._set_response(
                    response, success=True, message="tombstoned " + name
                )
            except Exception as exc:
                return self._set_response(
                    response,
                    success=False,
                    message="tombstone failed: " + repr(exc),
                )

        record = self._registry_get(name)
        if record is None:
            return self._set_response(
                response, success=False, message="unknown entity: " + name
            )
        try:
            setattr(record, "deleted", True)
        except Exception as exc:
            return self._set_response(
                response, success=False, message="tombstone failed: " + repr(exc)
            )
        return self._set_response(
            response, success=True, message="tombstoned " + name
        )

    def _on_set(self, request, response):
        payload = self._read_payload(request, "set_entity_state")
        name = getattr(request, "name", None) or payload.get("name")
        if not name:
            return self._set_response(
                response, success=False, message="set_entity_state: name required"
            )

        record = self._registry_get(name)
        if record is None:
            return self._set_response(
                response, success=False, message="unknown entity: " + name
            )
        entity = self._record_entity(record)
        if entity is None:
            return self._set_response(
                response, success=False, message="entity handle missing on record"
            )

        pos, quat, _fixed = self._extract_pose(request, payload)
        qs = None
        if hasattr(request, "joint_positions"):
            raw_q = list(getattr(request, "joint_positions") or [])
            if raw_q:
                qs = [float(q) for q in raw_q]
        if qs is None and "qpos" in payload:
            qs = [float(q) for q in payload["qpos"]]

        try:
            if pos is not None and hasattr(entity, "set_pos"):
                entity.set_pos(pos)
            if quat is not None and hasattr(entity, "set_quat"):
                entity.set_quat(quat)
            if qs is not None and hasattr(entity, "set_dofs_position"):
                entity.set_dofs_position(qs)
        except Exception as exc:
            self._warn("set_entity_state failed: " + repr(exc))
            return self._set_response(
                response, success=False, message="set failed: " + repr(exc)
            )

        return self._set_response(response, success=True, message="state updated")

    def _on_get(self, request, response):
        payload = self._read_payload(request, "get_entity_state")
        name = getattr(request, "name", None) or payload.get("name")
        if not name:
            return self._set_response(
                response, success=False, message="get_entity_state: name required"
            )

        record = self._registry_get(name)
        if record is None:
            return self._set_response(
                response, success=False, message="unknown entity: " + name
            )
        entity = self._record_entity(record)
        if entity is None:
            return self._set_response(
                response, success=False, message="entity handle missing on record"
            )

        pos = quat = qs = None
        try:
            if hasattr(entity, "get_pos"):
                pos = conv._as_numpy(entity.get_pos()).reshape(-1).tolist()
            if hasattr(entity, "get_quat"):
                quat = conv._as_numpy(entity.get_quat()).reshape(-1).tolist()
            if hasattr(entity, "get_qpos"):
                qs = conv._as_numpy(entity.get_qpos()).reshape(-1).tolist()
        except Exception as exc:
            self._warn("get_entity_state failed: " + repr(exc))
            return self._set_response(
                response, success=False, message="get failed: " + repr(exc)
            )

        if pos is not None and hasattr(response, "pose"):
            pose = response.pose
            position = getattr(pose, "position", None)
            if position is not None and len(pos) >= 3:
                try:
                    position.x = float(pos[0])
                    position.y = float(pos[1])
                    position.z = float(pos[2])
                except Exception:
                    pass
            orientation = getattr(pose, "orientation", None)
            if orientation is not None and quat is not None and len(quat) >= 4:
                try:
                    orientation.w = float(quat[0])
                    orientation.x = float(quat[1])
                    orientation.y = float(quat[2])
                    orientation.z = float(quat[3])
                except Exception:
                    pass
        if qs is not None and hasattr(response, "joint_positions"):
            try:
                response.joint_positions = [float(q) for q in qs]
            except Exception:
                pass

        message = json.dumps({"pos": pos, "quat": quat, "qpos": qs})
        return self._set_response(response, success=True, message=message)

    def _warn(self, message):
        logger = getattr(self.node, "get_logger", None)
        if callable(logger):
            try:
                logger().warning(message)
                return
            except Exception:
                pass


__all__ = ["EntityService"]
