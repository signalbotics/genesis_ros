"""Standalone KUKA LBR iiwa demo scene for the Genesis <-> ROS 2 bridge.

Spawns a fixed-base KUKA iiwa 7-DoF arm and publishes ``/clock``,
``/tf``, ``/tf_static``, ``/kuka/joint_states``.

Run via::

    ros2 run genesis_ros kuka_demo
    # or
    python3 -m genesis_ros.examples.kuka_scene

Set ``GENESIS_HEADLESS=1`` to suppress the viewer.
"""
from __future__ import annotations

import os
import sys

import genesis as gs

from genesis_ros import GenesisRosBridge
from genesis_ros.publishers.clock import ClockPublisher, RealTimeFactorPublisher
from genesis_ros.publishers.tf import TFPublisher, TFStaticPublisher
from genesis_ros.publishers.joint_state import JointStatePublisher


_URDF_CANDIDATES = (
    "/opt/genesis/assets/urdf/kuka_iiwa/model.urdf",
    os.path.join(
        os.path.dirname(__file__),
        "..", "..", "..",
        "genesis", "assets", "urdf", "kuka_iiwa", "model.urdf",
    ),
)


def _resolve_urdf() -> str:
    for path in _URDF_CANDIDATES:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError(
        "kuka_iiwa/model.urdf not found in any of: " + ", ".join(_URDF_CANDIDATES)
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
    kuka = scene.add_entity(
        gs.morphs.URDF(file=urdf_path, pos=(0.0, 0.0, 0.0), fixed=True)
    )
    scene.build()

    bridge = GenesisRosBridge(scene, node_name="genesis_bridge", use_sim_time=True)
    bridge.register_entity(kuka, name="kuka", urdf_xml=urdf_xml)

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

    bridge.spin()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
