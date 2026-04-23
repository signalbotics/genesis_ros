"""Standalone differential-drive demo scene for the Genesis <-> ROS 2 bridge.

Genesis doesn't ship a TurtleBot URDF, so this falls back to a simple box
chassis when none is found. Subscribes ``/turtlebot/cmd_vel`` and drives
wheel joints (only applied when a real URDF is present — the box fallback
has no wheels).

Run via::

    ros2 run genesis_ros turtlebot_demo
    # or with your own URDF
    TURTLEBOT_URDF=/path/to/turtlebot.urdf ros2 run genesis_ros turtlebot_demo

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
from genesis_ros.publishers.odom import OdomPublisher
from genesis_ros.subscribers.cmd_vel import CmdVelSubscriber


_CMD_VEL_CFG = {
    "mode": "diff_drive",
    "wheel_joint_left": "left_wheel",
    "wheel_joint_right": "right_wheel",
    "wheel_radius": 0.033,
    "wheel_base": 0.16,
}


def main(argv=None):
    urdf_env = os.environ.get("TURTLEBOT_URDF", "")
    urdf_path = urdf_env if os.path.isfile(urdf_env) else ""
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

    urdf_xml = ""
    if urdf_path:
        with open(urdf_path, "r") as fh:
            urdf_xml = fh.read()
        entity = scene.add_entity(gs.morphs.URDF(file=urdf_path, pos=(0.0, 0.0, 0.05)))
    else:
        entity = scene.add_entity(
            gs.morphs.Box(size=(0.3, 0.3, 0.1), pos=(0.0, 0.0, 0.05))
        )

    scene.build()

    bridge = GenesisRosBridge(scene, node_name="genesis_bridge", use_sim_time=True)
    bridge.register_entity(
        entity,
        name="turtlebot",
        urdf_xml=urdf_xml,
        is_mobile_base=True,
        cmd_vel_cfg=_CMD_VEL_CFG,
    )

    for publisher_cls in (
        ClockPublisher,
        RealTimeFactorPublisher,
        TFPublisher,
        TFStaticPublisher,
        JointStatePublisher,
        OdomPublisher,
    ):
        bridge.register_publisher(
            publisher_cls(bridge.node, scene, bridge.registry, {})
        )

    if urdf_xml:
        bridge.register_subscriber(
            CmdVelSubscriber(
                bridge.node, scene, bridge.registry, {"cmd_vel_cfg": _CMD_VEL_CFG}
            )
        )

    bridge.spin()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
