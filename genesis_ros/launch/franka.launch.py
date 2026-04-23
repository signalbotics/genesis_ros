"""Genesis Franka Panda launch.

Starts:
- ``examples/franka_scene.py`` (Genesis scene + ``genesis_bridge`` in-process).
- ``controller_manager/ros2_control_node`` with ``config/controllers_franka.yaml``.
- Spawners: ``joint_state_broadcaster`` then ``joint_trajectory_controller``
  chained via ``RegisterEventHandler(OnProcessExit)``.
- ``robot_state_publisher`` loading the Franka URDF.
- RViz preloaded with ``config/rviz/franka.rviz``.
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - only at launch time
    get_package_share_directory = None


# Default URDF resolution order:
#   1. $GENESIS_ASSETS + subpath   (custom asset bundle)
#   2. /opt/genesis/assets/...     (genesis-world-assets .deb)
#   3. ../../../Genesis/...        (sibling Genesis source checkout)
# Override via the ``urdf_path`` launch arg.
_URDF_SUBPATH = "urdf/panda_bullet/panda.urdf"
_URDF_CANDIDATES = [
    os.path.join(os.environ.get("GENESIS_ASSETS", "/nonexistent"), _URDF_SUBPATH),
    os.path.join("/opt/genesis/assets", _URDF_SUBPATH),
    os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..",
        "Genesis", "genesis", "assets", _URDF_SUBPATH,
    )),
]
DEFAULT_FRANKA_URDF = next(
    (p for p in _URDF_CANDIDATES if os.path.isfile(p)),
    os.path.join("/opt/genesis/assets", _URDF_SUBPATH),
)


def _pkg_share():
    if get_package_share_directory is not None:
        try:
            return get_package_share_directory("genesis_ros")
        except Exception:
            pass
    # Fallback to repo layout.
    return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def _robot_state_publisher(context, *args, **kwargs):
    path = LaunchConfiguration("urdf_path").perform(context)
    robot_description = ""
    if path and os.path.isfile(path):
        with open(path, "r") as f:
            robot_description = f.read()
    if not robot_description:
        return []
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() in ("1", "true")
    )
    return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="franka",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )]


def generate_launch_description():
    pkg_share = _pkg_share()
    default_controllers = os.path.join(pkg_share, "config", "controllers_franka.yaml")
    default_rviz = os.path.join(pkg_share, "config", "rviz", "franka.rviz")
    default_scene = os.path.join(pkg_share, "examples", "franka_scene.py")

    args = [
        DeclareLaunchArgument(
            "urdf_path",
            default_value=DEFAULT_FRANKA_URDF,
            description="Absolute path to the Franka URDF.",
        ),
        DeclareLaunchArgument(
            "controllers_path",
            default_value=default_controllers,
            description="controller_manager YAML config.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="RViz config file.",
        ),
        DeclareLaunchArgument(
            "scene_path",
            default_value=default_scene,
            description="Python scene file to run.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use /clock time source.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Whether to launch RViz.",
        ),
    ]

    # Genesis scene + in-process genesis_bridge node.
    scene_proc = ExecuteProcess(
        cmd=["python3", LaunchConfiguration("scene_path")],
        name="franka_scene",
        output="screen",
        additional_env={"GENESIS_SHOW_VIEWER": os.environ.get("GENESIS_SHOW_VIEWER", "0")},
    )

    rsp_action = OpaqueFunction(function=_robot_state_publisher)

    # Controller manager (topic_based_ros2_control hardware on /franka/joint_*).
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace="franka",
        parameters=[
            LaunchConfiguration("controllers_path"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    # Spawners chained via OnProcessExit for deterministic ordering.
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="jsb_spawner",
        namespace="franka",
        arguments=["joint_state_broadcaster", "-c", "/franka/controller_manager"],
        output="screen",
    )
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="jtc_spawner",
        namespace="franka",
        arguments=["joint_trajectory_controller", "-c", "/franka/controller_manager"],
        output="screen",
    )

    jsb_after_scene = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=scene_proc,
            on_exit=[jsb_spawner],
        )
    )
    jtc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[jtc_spawner],
        )
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
        rsp_action,
        scene_proc,
        controller_manager,
        jsb_after_scene,
        jtc_after_jsb,
        rviz_node,
    ])
