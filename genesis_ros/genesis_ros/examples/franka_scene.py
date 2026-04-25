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

import numpy as np

import genesis as gs

from genesis_ros import GenesisRosBridge
from genesis_ros.publishers.clock import ClockPublisher, RealTimeFactorPublisher
from genesis_ros.publishers.tf import TFPublisher, TFStaticPublisher
from genesis_ros.publishers.joint_state import JointStatePublisher
from genesis_ros.control.shm_bridge import register_shm_bridge


# Panda "ready" pose: arm in a relaxed bent configuration with the
# gripper open. Gazebo's gz_ros2_control demo uses essentially the same
# values; using them here means the arm holds against gravity from t=0
# rather than collapsing while we wait for controller_manager to come up.
_PANDA_READY_QPOS = np.array(
    [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04],
    dtype=np.float32,
)
_PANDA_KP = np.array(
    [4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100], dtype=np.float32,
)
_PANDA_KV = np.array(
    [450, 450, 350, 350, 200, 200, 200, 10, 10], dtype=np.float32,
)


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

    # Hold the ready pose against gravity from the first tick. Without
    # this the arm collapses for the ~1-2 s before controller_manager
    # activates the joint_trajectory_controller. Same idea as Gazebo's
    # JointPositionController initial-pose / hold mode.
    n_dofs = int(franka.n_dofs)
    qpos = _PANDA_READY_QPOS[: n_dofs]
    kp   = _PANDA_KP[: n_dofs]
    kv   = _PANDA_KV[: n_dofs]
    dofs = tuple(range(n_dofs))
    franka.set_dofs_kp(kp)
    franka.set_dofs_kv(kv)
    franka.set_qpos(qpos)
    franka.control_dofs_position(qpos, dofs)

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
