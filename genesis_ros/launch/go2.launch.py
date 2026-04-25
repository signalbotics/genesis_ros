"""Genesis Unitree Go2 quadruped launch.

Runs ``examples/go2_scene.py`` which publishes IMU, per-foot contact wrenches,
``/odom``, and one head camera image stream. RViz preloaded with
``config/rviz/go2.rviz``.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None


_URDF_SUBPATH = "urdf/go2/urdf/go2.urdf"
_URDF_CANDIDATES = [
    os.path.join(os.environ.get("GENESIS_ASSETS", "/nonexistent"), _URDF_SUBPATH),
    os.path.join("/opt/genesis/assets", _URDF_SUBPATH),
    os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..",
        "Genesis", "genesis", "assets", _URDF_SUBPATH,
    )),
]
DEFAULT_GO2_URDF = next(
    (p for p in _URDF_CANDIDATES if os.path.isfile(p)),
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
        namespace="go2",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )]


def generate_launch_description():
    pkg_share = _pkg_share()
    default_rviz = os.path.join(pkg_share, "config", "rviz", "go2.rviz")
    default_scene = os.path.join(pkg_share, "examples", "go2_scene.py")

    args = [
        DeclareLaunchArgument(
            "urdf_path",
            default_value=DEFAULT_GO2_URDF,
            description="Absolute path to the Go2 URDF.",
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

    # Launched via the ``go2_demo`` console script (setup.py) because
    # example modules live under site-packages, not share/.
    scene_proc = Node(
        package="genesis_ros",
        executable="go2_demo",
        name="go2_scene",
        output="screen",
    )

    rsp_action = OpaqueFunction(function=_robot_state_publisher)

    def _rviz(context, *_a, **_kw):
        if LaunchConfiguration("rviz").perform(context).lower() not in ("1", "true"):
            return []
        from genesis_ros.launch_utils import resolve_rviz_config
        cfg = resolve_rviz_config(
            LaunchConfiguration("rviz_config").perform(context)
        )
        return [Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", cfg],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )]

    return LaunchDescription(args + [
        rsp_action,
        scene_proc,
        OpaqueFunction(function=_rviz),
    ])
