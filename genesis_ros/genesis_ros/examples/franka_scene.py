"""Standalone Franka demo scene for the Genesis <-> ROS 2 bridge.

Spawns a Panda manipulator on a ground plane, opens the GUI viewer, and
publishes ``/clock``, ``/tf``, ``/tf_static``, and
``/franka/joint_states``. Intended to replace ``ros2 run genesis_ros
genesis_bridge`` when you want to see a real robot in the scene.

Run via either of:

* ``ros2 run genesis_ros franka_demo``
* ``python3 -m genesis_ros.examples.franka_scene``

Toggle the viewer off with ``GENESIS_HEADLESS=1`` (handy for CI).
"""
from __future__ import annotations

import os
import sys

import genesis as gs

from genesis_ros import GenesisRosBridge
from genesis_ros.publishers.clock import ClockPublisher, RealTimeFactorPublisher
from genesis_ros.publishers.tf import TFPublisher, TFStaticPublisher
from genesis_ros.publishers.joint_state import JointStatePublisher
from genesis_ros.control.shm_bridge import register_shm_bridge


# Prefer the system asset location from the genesis-world-assets .deb,
# then the in-repo copy for dev checkouts.
_URDF_CANDIDATES = (
    "/opt/genesis/assets/urdf/panda_bullet/panda.urdf",
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "genesis",
        "assets",
        "urdf",
        "panda_bullet",
        "panda.urdf",
    ),
)


def _resolve_urdf() -> str:
    for path in _URDF_CANDIDATES:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError(
        "panda.urdf not found in any of: " + ", ".join(_URDF_CANDIDATES)
    )


def main(argv=None):
    urdf_path = _resolve_urdf()
    show_viewer = os.environ.get("GENESIS_HEADLESS", "0") != "1"

    try:
        gs.init()
    except Exception:
        pass

    viewer_fps_env = os.environ.get("GENESIS_VIEWER_FPS", "").strip()
    viewer_fps = int(viewer_fps_env) if viewer_fps_env.isdigit() else None
    scene = gs.Scene(
        show_viewer=show_viewer,
        viewer_options=gs.options.ViewerOptions(max_FPS=viewer_fps),
    )
    scene.add_entity(gs.morphs.Plane())
    with open(urdf_path, "r") as fh:
        urdf_xml = fh.read()
    franka = scene.add_entity(gs.morphs.URDF(file=urdf_path, pos=(0.0, 0.0, 0.0), fixed=True))
    scene.build()

    bridge = GenesisRosBridge(scene, node_name="genesis_bridge", use_sim_time=True)
    bridge.register_entity(franka, name="franka", urdf_xml=urdf_xml)

    for publisher_cls in (
        ClockPublisher,
        RealTimeFactorPublisher,
        TFPublisher,
        TFStaticPublisher,
        JointStatePublisher,
    ):
        bridge.register_publisher(
            publisher_cls(bridge.node, scene, bridge.registry, {})
        )

    # Native shared-memory hardware bridge for genesis_ros2_control/
    # GenesisSystem. Always wired on -- if no controller_manager is
    # running, the shm region just sits there harmlessly. Opt out with
    # GENESIS_DISABLE_SHM=1 for boxes where /dev/shm is restricted.
    if os.environ.get("GENESIS_DISABLE_SHM", "0") != "1":
        try:
            register_shm_bridge(bridge, robot="franka")
        except Exception as exc:
            bridge.node.get_logger().warning(
                "shm hardware bridge not started: " + repr(exc)
            )

    bridge.spin()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
