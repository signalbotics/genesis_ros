"""Generic Genesis <-> ROS 2 bridge launch.

Starts the ``genesis_bridge`` node and (optionally) ``robot_state_publisher``
loading a URDF/xacro from ``robot_description_path``. RViz can be started by
passing ``rviz:=true``.

Arguments
---------
robot_description_path  Absolute path to a URDF or xacro file.
robot_name              Namespace string used by the bridge (default: ``robot``).
use_sim_time            Whether to use ``/clock`` for time (default: ``true``).
env_idx                 Which Genesis batched env to publish (default: ``0``).
rviz                    Whether to launch RViz (default: ``false``).
rviz_config             Optional absolute path to an ``.rviz`` config.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_state_publisher(context, *args, **kwargs):
    """Resolve ``robot_description`` from URDF or xacro file at launch time."""
    path = LaunchConfiguration("robot_description_path").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() in ("1", "true")
    )

    robot_description = ""
    if path and os.path.isfile(path):
        if path.endswith(".xacro"):
            import subprocess
            try:
                robot_description = subprocess.check_output(["xacro", path]).decode("utf-8")
            except Exception:
                with open(path, "r") as f:
                    robot_description = f.read()
        else:
            with open(path, "r") as f:
                robot_description = f.read()

    if not robot_description:
        return []

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )
    return [rsp]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "robot_description_path",
            default_value="",
            description="Absolute path to a URDF or xacro file.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="robot",
            description="ROS namespace for this robot.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Whether to use /clock for time.",
        ),
        DeclareLaunchArgument(
            "env_idx",
            default_value="0",
            description="Which Genesis batched env to publish.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Whether to launch RViz.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="Optional absolute path to an .rviz config file.",
        ),
    ]

    bridge_node = Node(
        package="genesis_ros",
        executable="genesis_bridge",
        name="genesis_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "env_idx": LaunchConfiguration("env_idx"),
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(args + [
        OpaqueFunction(function=_robot_state_publisher),
        bridge_node,
        rviz_node,
    ])
