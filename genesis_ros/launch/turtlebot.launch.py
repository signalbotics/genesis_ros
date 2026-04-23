"""Genesis TurtleBot diff-drive launch.

Runs ``examples/turtlebot_scene.py`` which registers a differential-drive
``/cmd_vel`` subscriber, a raycaster publishing ``/scan``, and a front camera
publishing ``/image_raw``. ``controller_manager`` loads a
``velocity_controllers/JointGroupVelocityController`` on the two wheel joints.
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
except Exception:  # pragma: no cover
    get_package_share_directory = None


# Genesis doesn't ship a TurtleBot URDF by default. Provide your own via
# ``TURTLEBOT_URDF`` env var or the ``urdf_path`` launch arg. Resolution
# order matches the other demos: env var override, /opt/genesis/assets,
# sibling Genesis checkout.
_URDF_SUBPATH = "urdf/turtlebot/turtlebot.urdf"
_URDF_CANDIDATES = [
    os.environ.get("TURTLEBOT_URDF", ""),
    os.path.join(os.environ.get("GENESIS_ASSETS", "/nonexistent"), _URDF_SUBPATH),
    os.path.join("/opt/genesis/assets", _URDF_SUBPATH),
    os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..",
        "Genesis", "genesis", "assets", _URDF_SUBPATH,
    )),
]
DEFAULT_TURTLEBOT_URDF = next(
    (p for p in _URDF_CANDIDATES if p and os.path.isfile(p)),
    os.path.join("/opt/genesis/assets", _URDF_SUBPATH),
)


def _pkg_share():
    if get_package_share_directory is not None:
        try:
            return get_package_share_directory("genesis_ros")
        except Exception:
            pass
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
        namespace="turtlebot",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )]


def generate_launch_description():
    pkg_share = _pkg_share()
    default_controllers = os.path.join(pkg_share, "config", "controllers_turtlebot.yaml")
    default_rviz = os.path.join(pkg_share, "config", "rviz", "turtlebot.rviz")
    default_scene = os.path.join(pkg_share, "examples", "turtlebot_scene.py")

    args = [
        DeclareLaunchArgument(
            "urdf_path",
            default_value=DEFAULT_TURTLEBOT_URDF,
            description="Absolute path to the TurtleBot URDF.",
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

    scene_proc = ExecuteProcess(
        cmd=["python3", LaunchConfiguration("scene_path")],
        name="turtlebot_scene",
        output="screen",
        additional_env={"GENESIS_SHOW_VIEWER": os.environ.get("GENESIS_SHOW_VIEWER", "0")},
    )

    rsp_action = OpaqueFunction(function=_robot_state_publisher)

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace="turtlebot",
        parameters=[
            LaunchConfiguration("controllers_path"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="jsb_spawner",
        namespace="turtlebot",
        arguments=["joint_state_broadcaster", "-c", "/turtlebot/controller_manager"],
        output="screen",
    )
    vel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="velocity_controller_spawner",
        namespace="turtlebot",
        arguments=["velocity_controllers", "-c", "/turtlebot/controller_manager"],
        output="screen",
    )

    jsb_after_scene = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=scene_proc,
            on_exit=[jsb_spawner],
        )
    )
    vel_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[vel_spawner],
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
        vel_after_jsb,
        rviz_node,
    ])
