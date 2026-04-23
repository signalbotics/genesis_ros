"""Physics-property services for the Genesis<->ROS 2 bridge.

Exposes two bridge-level services that mirror what Gazebo offers via
``gz_sim`` / ``gz topic``:

* ``/genesis/set_gravity``             -- write the gravity vector.
* ``/genesis/set_physics_properties``  -- read/write gravity + report
  current dt / substeps (both are immutable post-``scene.build`` in
  Genesis today; we surface them in the response for completeness).

Both services accept a JSON-encoded argument payload via
``std_srvs/Trigger`` since ``simulation_interfaces`` does not define
equivalents yet. Clients call them like:

.. code-block:: bash

   ros2 service call /genesis/set_gravity std_srvs/srv/Trigger '{}' \\
       --args '{"gravity": [0, 0, -9.81]}'
   ros2 service call /genesis/set_physics_properties std_srvs/srv/Trigger '{}'

The ``--args`` payload is read from the matching node parameter
(``set_gravity.args`` / ``set_physics_properties.args``) when the
service request itself carries no string field -- same contract as
``EntityService``.
"""
from __future__ import annotations

try:
    import rclpy  # noqa: F401
    from rclpy.node import Node  # noqa: F401
    from std_srvs.srv import Trigger as _Trigger
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    _Trigger = None  # type: ignore[assignment]

import json
import threading


class PhysicsService:
    """Hold service handles for gravity + physics-property queries."""

    def __init__(self, node, scene, bridge):
        if not _ROS_AVAILABLE:
            raise RuntimeError(
                "PhysicsService requires rclpy / std_srvs -- ROS 2 runtime "
                "was not importable."
            )
        self.node = node
        self.scene = scene
        self.bridge = bridge
        self._lock = threading.Lock()

        for key in ("set_gravity", "set_physics_properties"):
            param_name = key + ".args"
            try:
                if not node.has_parameter(param_name):
                    node.declare_parameter(param_name, "")
            except Exception:
                pass

        self._set_gravity_srv = node.create_service(
            _Trigger, "/genesis/set_gravity", self._on_set_gravity
        )
        self._set_props_srv = node.create_service(
            _Trigger, "/genesis/set_physics_properties", self._on_set_props
        )

    # ---------------------------------------------------------- helpers
    @staticmethod
    def _set_response(response, *, success=True, message=""):
        if hasattr(response, "success"):
            response.success = bool(success)
        if hasattr(response, "message"):
            response.message = str(message)
        return response

    def _read_payload(self, request, param_key):
        raw = None
        for attr in ("args", "data", "json"):
            candidate = getattr(request, attr, None)
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
        if not raw:
            return {}
        try:
            return json.loads(raw)
        except Exception:
            return {}

    def _current_state(self):
        sim = getattr(self.scene, "sim", None)
        gravity = None
        dt = substeps = None
        if sim is not None:
            try:
                g = sim.gravity
                gravity = [float(g[0]), float(g[1]), float(g[2])]
            except Exception:
                gravity = None
            dt = getattr(sim, "dt", None)
            substeps = getattr(sim, "substeps", None)
        return {
            "gravity": gravity,
            "dt": float(dt) if dt is not None else None,
            "substeps": int(substeps) if substeps is not None else None,
        }

    # ---------------------------------------------------------- callbacks
    def _on_set_gravity(self, request, response):
        payload = self._read_payload(request, "set_gravity")
        gravity = payload.get("gravity")
        if gravity is None:
            return self._set_response(
                response,
                success=False,
                message="set_gravity: missing 'gravity' field in JSON args "
                "(expected [gx, gy, gz])",
            )
        if not isinstance(gravity, (list, tuple)) or len(gravity) != 3:
            return self._set_response(
                response,
                success=False,
                message="set_gravity: 'gravity' must be a 3-element list",
            )
        try:
            vec = [float(v) for v in gravity]
        except (TypeError, ValueError) as exc:
            return self._set_response(
                response,
                success=False,
                message="set_gravity: numeric parse failed: " + repr(exc),
            )
        sim = getattr(self.scene, "sim", None)
        setter = getattr(sim, "set_gravity", None) if sim is not None else None
        if not callable(setter):
            return self._set_response(
                response,
                success=False,
                message="set_gravity: scene.sim has no set_gravity() method",
            )
        with self._lock:
            try:
                setter(vec)
            except Exception as exc:
                return self._set_response(
                    response,
                    success=False,
                    message="set_gravity failed: " + repr(exc),
                )
        return self._set_response(
            response,
            success=True,
            message=json.dumps({"gravity": vec}),
        )

    def _on_set_props(self, request, response):
        payload = self._read_payload(request, "set_physics_properties")
        state = self._current_state()
        warnings: list[str] = []

        gravity = payload.get("gravity")
        if gravity is not None:
            if not isinstance(gravity, (list, tuple)) or len(gravity) != 3:
                return self._set_response(
                    response,
                    success=False,
                    message="'gravity' must be a 3-element list",
                )
            try:
                vec = [float(v) for v in gravity]
            except (TypeError, ValueError) as exc:
                return self._set_response(
                    response,
                    success=False,
                    message="gravity parse failed: " + repr(exc),
                )
            sim = getattr(self.scene, "sim", None)
            setter = getattr(sim, "set_gravity", None) if sim is not None else None
            if not callable(setter):
                return self._set_response(
                    response,
                    success=False,
                    message="scene.sim has no set_gravity() method",
                )
            with self._lock:
                try:
                    setter(vec)
                    state["gravity"] = vec
                except Exception as exc:
                    return self._set_response(
                        response,
                        success=False,
                        message="set_gravity failed: " + repr(exc),
                    )

        # dt / substeps are immutable post-build in Genesis; surface a
        # warning if the caller tried to change them so behaviour is
        # visible instead of silently ignored.
        if "dt" in payload and payload["dt"] != state["dt"]:
            warnings.append(
                "dt is immutable post-scene.build; ignored (current="
                + str(state["dt"]) + ")"
            )
        if "substeps" in payload and payload["substeps"] != state["substeps"]:
            warnings.append(
                "substeps is immutable post-scene.build; ignored (current="
                + str(state["substeps"]) + ")"
            )

        body = dict(state)
        if warnings:
            body["warnings"] = warnings
        return self._set_response(
            response, success=True, message=json.dumps(body)
        )


__all__ = ["PhysicsService"]
