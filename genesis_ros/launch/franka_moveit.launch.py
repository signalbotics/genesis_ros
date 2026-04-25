"""MoveIt 2 layered on top of franka.launch.py.

Includes franka.launch.py (Genesis scene + ros2_control + RViz) and
adds the move_group node. The included franka launch is told to load
the MoveIt-aware RViz config so the MotionPlanning panel comes up in
the same RViz window -- no second RViz process.

Usage:
    ros2 launch genesis_ros franka_moveit.launch.py

Requires (apt):
    ros-jazzy-moveit-ros-move-group
    ros-jazzy-moveit-planners-ompl
    ros-jazzy-moveit-simple-controller-manager
    ros-jazzy-moveit-ros-visualization
"""
from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None


def _pkg_share() -> str:
    if get_package_share_directory is not None:
        try:
            return get_package_share_directory("genesis_ros")
        except Exception:
            pass
    return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def _load_yaml(path: str) -> dict:
    import yaml
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _build_robot_description(base_urdf: str, xacro_path: str) -> str:
    """Same xacro render franka.launch.py uses, so move_group sees an
    identical URDF."""
    import xacro
    doc = xacro.process_file(
        xacro_path,
        mappings={
            "base_urdf": base_urdf,
            "hardware": "shm",
            "robot_name": "franka",
            "commands_topic": "/franka/joint_commands",
            "states_topic": "/franka/joint_states",
        },
    )
    urdf = doc.toprettyxml(indent="  ")
    meshes_root = os.path.join(os.path.dirname(base_urdf), "meshes")
    if os.path.isdir(meshes_root):
        urdf = urdf.replace("package://meshes/", "file://" + meshes_root + "/")
    return urdf


def _move_group(context, *_a, **_kw):
    pkg = _pkg_share()
    moveit_dir = os.path.join(pkg, "config", "moveit")
    base_urdf = LaunchConfiguration("urdf_path").perform(context)
    xacro_path = os.path.join(pkg, "urdf", "franka_panda.urdf.xacro")

    with open(os.path.join(moveit_dir, "panda.srdf"), "r") as f:
        srdf = f.read()
    kinematics    = _load_yaml(os.path.join(moveit_dir, "kinematics.yaml"))
    joint_limits  = _load_yaml(os.path.join(moveit_dir, "joint_limits.yaml"))
    moveit_ctrls  = _load_yaml(os.path.join(moveit_dir, "moveit_controllers.yaml"))
    ompl_planning = _load_yaml(os.path.join(moveit_dir, "ompl_planning.yaml"))
    robot_description = _build_robot_description(base_urdf, xacro_path)

    return [Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        namespace="franka",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": srdf},
            {"robot_description_kinematics": kinematics},
            {"robot_description_planning": joint_limits},
            moveit_ctrls,
            {
                "planning_pipelines": {
                    "pipeline_names": ["ompl"],
                    "default_planning_pipeline": "ompl",
                }
            },
            {"ompl": ompl_planning},
            {"use_sim_time": True},
            # ----- planning_scene_monitor publishing flags -----
            # Without these, MoveIt's monitored_planning_scene topic is
            # VOLATILE and only ticks on scene change; RViz subscribes
            # after the publisher's last frame and stays at
            # "No Planning Scene Loaded" forever. Forcing the monitor
            # to periodically (re)publish + emit geometry/state/tf
            # updates makes the topic carry a steady scene that any
            # late RViz can pick up.
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            {"planning_scene_monitor_options": {
                "name": "planning_scene_monitor",
                "robot_description": "robot_description",
                "joint_state_topic": "/franka/joint_states",
                "publish_planning_scene": True,
                "publish_planning_scene_hz": 30.0,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            }},
            {"publish_planning_scene": True},
            {"publish_planning_scene_hz": 30.0},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
        ],
    )]


def generate_launch_description():
    pkg = _pkg_share()
    franka_launch = os.path.join(pkg, "launch", "franka.launch.py")
    moveit_rviz = os.path.join(pkg, "config", "rviz", "franka_moveit.rviz")

    args = [
        DeclareLaunchArgument(
            "urdf_path",
            default_value="/opt/genesis/assets/urdf/panda_bullet/panda.urdf",
            description="Absolute path to the base panda URDF.",
        ),
    ]

    # Include franka.launch.py and tell it to use the MoveIt RViz
    # preset so the MotionPlanning panel comes up in franka.launch.py's
    # RViz process. Don't pass rviz:=false -- we want exactly that
    # RViz, just with a different config.
    franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(franka_launch),
        launch_arguments={
            "urdf_path": LaunchConfiguration("urdf_path"),
            "rviz_config": moveit_rviz,
        }.items(),
    )

    return LaunchDescription(args + [franka, OpaqueFunction(function=_move_group)])
