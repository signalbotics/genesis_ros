"""Standalone Crazyflie drone demo scene for the Genesis <-> ROS 2 bridge.

Spawns a Crazyflie 2.x (cf2x) quadrotor hovering above a plane and
publishes ``/clock``, ``/tf``, ``/tf_static``, ``/drone/joint_states``,
``/drone/odom``, plus a ground-truth pose on
``/drone/pose_ground_truth`` since drones are typically consumed by SLAM
/ VIO pipelines that need a reference.

Run via::

    ros2 run genesis_ros drone_demo
    # or
    python3 -m genesis_ros.examples.drone_scene

Set ``GENESIS_HEADLESS=1`` to suppress the viewer. Use
``DRONE_URDF=<path>`` to pick a different quadrotor URDF (``cf2p``,
``racer`` ship with Genesis).
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
from genesis_ros.publishers.ground_truth import GroundTruthPosePublisher


_DEFAULT_URDF = "cf2x.urdf"
_URDF_CANDIDATES = (
    "/opt/genesis/assets/urdf/drones/" + _DEFAULT_URDF,
    os.path.join(
        os.path.dirname(__file__),
        "..", "..", "..",
        "genesis", "assets", "urdf", "drones", _DEFAULT_URDF,
    ),
)


def _resolve_urdf() -> str:
    override = os.environ.get("DRONE_URDF", "").strip()
    if override and os.path.isfile(override):
        return os.path.abspath(override)
    for path in _URDF_CANDIDATES:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError(
        "drone URDF not found. Set DRONE_URDF or install genesis-world-assets."
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
        viewer_options=gs.options.ViewerOptions(
            max_FPS=viewer_fps,
            camera_pos=(2.5, 1.5, 1.5),
            camera_lookat=(0.0, 0.0, 0.5),
        ),
    )
    scene.add_entity(gs.morphs.Plane())
    with open(urdf_path, "r") as fh:
        urdf_xml = fh.read()
    drone = scene.add_entity(
        gs.morphs.URDF(file=urdf_path, pos=(0.0, 0.0, 0.5))
    )
    scene.build()

    bridge = GenesisRosBridge(scene, node_name="genesis_bridge", use_sim_time=True)
    bridge.register_entity(
        drone, name="drone", urdf_xml=urdf_xml,
        is_mobile_base=True, ground_truth=True,
    )

    for publisher_cls in (
        ClockPublisher,
        RealTimeFactorPublisher,
        TFPublisher,
        TFStaticPublisher,
        JointStatePublisher,
        OdomPublisher,
        GroundTruthPosePublisher,
    ):
        bridge.register_publisher(
            publisher_cls(bridge.node, scene, bridge.registry, {})
        )

    bridge.spin()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
