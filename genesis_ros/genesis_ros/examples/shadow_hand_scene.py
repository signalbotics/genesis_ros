"""Standalone Shadow Hand demo scene for the Genesis <-> ROS 2 bridge.

Spawns a fixed-base Shadow Dexterous Hand and publishes ``/clock``,
``/tf``, ``/tf_static``, ``/shadow_hand/joint_states``.

Run via::

    ros2 run genesis_ros shadow_hand_demo
    # or
    python3 -m genesis_ros.examples.shadow_hand_scene

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
    "/opt/genesis/assets/urdf/shadow_hand/shadow_hand.urdf",
    os.path.join(
        os.path.dirname(__file__),
        "..", "..", "..",
        "genesis", "assets", "urdf", "shadow_hand", "shadow_hand.urdf",
    ),
)


def _resolve_urdf() -> str:
    for path in _URDF_CANDIDATES:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError(
        "shadow_hand.urdf not found in any of: " + ", ".join(_URDF_CANDIDATES)
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
    # Mount the hand palm-up on a virtual wrist at table height so all
    # five fingers are visible in the default viewer / RViz view.
    hand = scene.add_entity(
        gs.morphs.URDF(file=urdf_path, pos=(0.0, 0.0, 0.5), fixed=True)
    )
    scene.build()

    bridge = GenesisRosBridge(scene, node_name="genesis_bridge", use_sim_time=True)
    bridge.register_entity(hand, name="shadow_hand", urdf_xml=urdf_xml)

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
