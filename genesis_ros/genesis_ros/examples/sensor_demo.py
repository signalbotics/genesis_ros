"""Sensor-heavy demo scene -- exercises every publisher in the bridge.

A fixed Franka arm sits in a small "kitchen" built from primitives
(walls, a workbench, a fridge, a pillar, a tabletop ball and box) and
carries a full sensor suite:

* Wrist RGB + depth camera -> ``/franka/wrist_cam/{image_raw,depth,
  camera_info,points}``.
* End-effector IMU -> ``/franka/imu``.
* End-effector proximity probe -> ``/franka/ee_proximity``.
* Base-mounted spherical lidar -> ``/franka/lidar/points`` (or
  ``/franka/lidar/scan`` for single-ring patterns).
* Per-link contact wrenches -> ``/franka/contacts/<link>``.
* Ground-truth pose -> ``/franka/pose_ground_truth``.
* TF / tf_static / joint_states / clock / rtf as usual.

Run:

    ros2 run genesis_ros sensor_demo
    # or
    python3 -m genesis_ros.examples.sensor_demo

Env knobs: ``GENESIS_HEADLESS=1`` suppresses the viewer;
``GENESIS_VIEWER_FPS=<n>`` caps it at N Hz.
"""
from __future__ import annotations

import math
import os
import sys
import threading

import numpy as np

import genesis as gs

from genesis_ros import GenesisRosBridge
from genesis_ros.publishers.clock import ClockPublisher, RealTimeFactorPublisher
from genesis_ros.publishers.tf import TFPublisher, TFStaticPublisher
from genesis_ros.publishers.joint_state import JointStatePublisher
from genesis_ros.publishers.camera import CameraPublisher
from genesis_ros.publishers.imu import ImuPublisher
from genesis_ros.publishers.contact import ContactPublisher
from genesis_ros.publishers.proximity import ProximityPublisher
from genesis_ros.publishers.raycaster import RaycasterPublisher
from genesis_ros.publishers.ground_truth import GroundTruthPosePublisher
from genesis_ros.services.sim_control import SimControlService
from genesis_ros.services.physics import PhysicsService


_URDF_CANDIDATES = (
    "/opt/genesis/assets/urdf/panda_bullet/panda.urdf",
    os.path.join(
        os.path.dirname(__file__),
        "..", "..", "..",
        "genesis", "assets", "urdf", "panda_bullet", "panda.urdf",
    ),
)


def _resolve_urdf() -> str:
    for path in _URDF_CANDIDATES:
        if os.path.isfile(path):
            return os.path.abspath(path)
    raise FileNotFoundError(
        "panda.urdf not found in any of: " + ", ".join(_URDF_CANDIDATES)
    )


# -- scene furniture ---------------------------------------------------------

def _build_kitchen(scene: "gs.Scene") -> None:
    """Box/cylinder clutter around the robot so the sensors have content."""
    # Floor.
    scene.add_entity(gs.morphs.Plane())

    # 4 walls framing a 4x4 m room centred on the origin.
    WALL_H = 1.8
    WALL_T = 0.05
    ROOM = 2.0
    for x, y, sx, sy in (
        (0.0,  ROOM, 2 * ROOM + WALL_T, WALL_T),  # +Y wall
        (0.0, -ROOM, 2 * ROOM + WALL_T, WALL_T),  # -Y wall
        ( ROOM, 0.0, WALL_T, 2 * ROOM + WALL_T),  # +X wall
        (-ROOM, 0.0, WALL_T, 2 * ROOM + WALL_T),  # -X wall
    ):
        scene.add_entity(
            gs.morphs.Box(
                pos=(x, y, WALL_H / 2.0),
                size=(sx, sy, WALL_H),
                fixed=True,
            )
        )

    # Workbench in front of the arm.
    scene.add_entity(
        gs.morphs.Box(
            pos=(0.7, 0.0, 0.3),
            size=(0.6, 1.0, 0.6),
            fixed=True,
        )
    )

    # "Fridge" to the right.
    scene.add_entity(
        gs.morphs.Box(
            pos=(0.8, 1.2, 0.9),
            size=(0.7, 0.7, 1.8),
            fixed=True,
        )
    )

    # "Cabinet" to the left.
    scene.add_entity(
        gs.morphs.Box(
            pos=(0.0, -1.4, 0.6),
            size=(1.2, 0.5, 1.2),
            fixed=True,
        )
    )

    # Pillar behind the arm.
    scene.add_entity(
        gs.morphs.Cylinder(
            pos=(-1.2, 0.8, 0.75),
            height=1.5,
            radius=0.12,
            fixed=True,
        )
    )

    # Free-floating props on the workbench so contact sensors light up.
    scene.add_entity(
        gs.morphs.Sphere(pos=(0.65, -0.15, 0.75), radius=0.04)
    )
    scene.add_entity(
        gs.morphs.Box(pos=(0.65, 0.15, 0.75), size=(0.06, 0.06, 0.06))
    )


# -- IK sweep thread ---------------------------------------------------------

def _start_sweep(scene: "gs.Scene", franka, end_effector, motors_dof, stop_event):
    """Background thread: drive the EE in a slow circle via IK.

    Runs on the main thread? No -- scene.control_dofs_position is safe to
    call outside the step loop because it only stages commands for the
    next step. We run the IK + command on a daemon thread so the demo
    keeps moving without blocking bridge.spin().
    """

    def _loop():
        center = np.array([0.55, 0.0, 0.55], dtype=np.float32)
        radius = 0.12
        omega = 0.6  # rad/s

        t0 = None
        import time as _t
        try:
            while not stop_event.is_set():
                if t0 is None:
                    t0 = _t.perf_counter()
                t = _t.perf_counter() - t0
                target = center + np.array(
                    [math.cos(omega * t), math.sin(omega * t), 0.0],
                    dtype=np.float32,
                ) * radius
                try:
                    qpos = franka.inverse_kinematics(
                        link=end_effector,
                        pos=target,
                        quat=np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float32),
                    )
                    franka.control_dofs_position(qpos[:-2], motors_dof)
                except Exception:
                    # IK can fail transiently near singularities; skip.
                    pass
                _t.sleep(0.02)
        except Exception:
            pass

    thread = threading.Thread(
        target=_loop,
        name="sensor-demo-sweep",
        daemon=True,
    )
    thread.start()
    return thread


# -- main -------------------------------------------------------------------

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
            camera_pos=(3.0, -2.5, 2.0),
            camera_lookat=(0.3, 0.0, 0.5),
        ),
    )

    _build_kitchen(scene)

    with open(urdf_path, "r") as fh:
        urdf_xml = fh.read()
    franka = scene.add_entity(
        gs.morphs.URDF(file=urdf_path, pos=(0.0, 0.0, 0.0), fixed=True)
    )
    end_effector = franka.get_link("hand")

    # --- sensors --------------------------------------------------------
    imu = scene.add_sensor(
        gs.sensors.IMU(
            entity_idx=franka.idx,
            link_idx_local=end_effector.idx_local,
            pos_offset=(0.0, 0.0, 0.05),
            acc_noise=(0.02, 0.02, 0.02),
            gyro_noise=(0.005, 0.005, 0.005),
            delay=0.005,
        )
    )

    proximity = scene.add_sensor(
        gs.sensors.Proximity(
            entity_idx=franka.idx,
            link_idx_local=end_effector.idx_local,
            probe_local_pos=[(0.0, 0.0, 0.08)],
            # Track every link in the scene (all rigid geoms) so the
            # probe reports the distance to the nearest surface.
            track_link_idx=tuple(
                range(1, int(scene.sim.rigid_solver.n_links))
            ),
            max_range=2.0,
        )
    )

    lidar = scene.add_sensor(
        gs.sensors.Lidar(
            entity_idx=franka.idx,
            link_idx_local=0,          # base link
            pos_offset=(0.0, 0.0, 1.2),
            pattern=gs.sensors.SphericalPattern(),
            max_range=5.0,
            return_world_frame=False,
        )
    )

    # --- camera ---------------------------------------------------------
    wrist_cam = scene.add_camera(
        res=(320, 240),
        pos=(0.0, 0.0, 0.8),
        lookat=(0.6, 0.0, 0.3),
        fov=60,
        GUI=False,
    )
    # Attach to the EE so it follows the arm around the scene.
    # 4x4 offset: point forward (+x in EE frame) with a small downward tilt.
    offset_T = np.eye(4, dtype=np.float32)
    offset_T[:3, 3] = (0.08, 0.0, 0.02)
    wrist_cam.attach(end_effector, offset_T)

    # --- build + bridge -------------------------------------------------
    scene.build()

    bridge = GenesisRosBridge(scene, node_name="genesis_bridge", use_sim_time=True)
    bridge.register_entity(
        franka, name="franka", urdf_xml=urdf_xml, ground_truth=True
    )
    bridge.registry.register_sensor("franka", "imu", imu)
    bridge.registry.register_sensor("franka", "ee_proximity", proximity)
    bridge.registry.register_sensor("franka", "lidar", lidar)
    bridge.registry.register_camera(
        "franka", "wrist_cam", wrist_cam, parent_link="hand"
    )

    for publisher_cls, cfg in (
        (ClockPublisher,            {}),
        (RealTimeFactorPublisher,   {"rtf_provider": lambda: bridge.rtf}),
        (TFPublisher,               {}),
        (TFStaticPublisher,         {}),
        (JointStatePublisher,       {}),
        (ImuPublisher,              {}),
        (ContactPublisher,          {}),
        (ProximityPublisher,        {}),
        (RaycasterPublisher,        {}),
        (CameraPublisher,           {"publish_depth": True, "publish_pointcloud": True}),
        (GroundTruthPosePublisher,  {"rate_hz": 30.0}),
    ):
        bridge.register_publisher(
            publisher_cls(bridge.node, scene, bridge.registry, cfg)
        )
    bridge.register_service(SimControlService(bridge.node, scene, bridge))
    bridge.register_service(PhysicsService(bridge.node, scene, bridge))

    motors_dof = (0, 1, 2, 3, 4, 5, 6)
    franka.set_dofs_kp(np.array(
        [4500.0, 4500.0, 3500.0, 3500.0, 2000.0, 2000.0, 2000.0, 100.0, 100.0]
    ))
    franka.set_dofs_kv(np.array(
        [450.0, 450.0, 350.0, 350.0, 200.0, 200.0, 200.0, 10.0, 10.0]
    ))

    stop = threading.Event()
    sweep_thread = _start_sweep(scene, franka, end_effector, motors_dof, stop)

    try:
        bridge.spin()
    finally:
        stop.set()
        sweep_thread.join(timeout=1.0)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
