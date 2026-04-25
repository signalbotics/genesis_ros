"""Launch the sensor-heavy demo with RViz preloaded.

Starts the ``sensor_demo`` console script (Genesis scene +
``genesis_bridge`` in process), ``robot_state_publisher`` with the
panda URDF, and RViz preloaded with the sensor layout.

The panda URDF from Genesis ships ``package://meshes/visual/hand.obj``
style mesh references that RViz cannot resolve (there is no ROS package
named ``meshes``). This launch rewrites them to absolute ``file://``
URIs so the RobotModel display loads.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None


_URDF_SUBPATH = "urdf/panda_bullet/panda.urdf"
_URDF_CANDIDATES = [
    os.path.join(os.environ.get("GENESIS_ASSETS", "/nonexistent"), _URDF_SUBPATH),
    os.path.join("/opt/genesis/assets", _URDF_SUBPATH),
    os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..",
        "Genesis", "genesis", "assets", _URDF_SUBPATH,
    )),
]
DEFAULT_URDF = next(
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


def _rewrite_urdf_meshes(urdf_text: str, urdf_path: str) -> str:
    """Replace ``package://meshes/...`` with file:// URIs rooted at the
    URDF's sibling ``meshes/`` directory so RViz can resolve them."""
    meshes_root = os.path.join(os.path.dirname(urdf_path), "meshes")
    if not os.path.isdir(meshes_root):
        return urdf_text
    return urdf_text.replace("package://meshes/", "file://" + meshes_root + "/")


def _robot_state_publisher(context, *_args, **_kwargs):
    path = LaunchConfiguration("urdf_path").perform(context)
    if not path or not os.path.isfile(path):
        return []
    with open(path, "r") as fh:
        robot_description = fh.read()
    robot_description = _rewrite_urdf_meshes(robot_description, path)
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
    default_rviz = os.path.join(pkg_share, "config", "rviz", "sensor_demo.rviz")

    args = [
        DeclareLaunchArgument(
            "urdf_path",
            default_value=DEFAULT_URDF,
            description="Absolute path to the panda URDF.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz,
            description="RViz config file for the sensor demo.",
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

    # Invoke the Python module via ``ros2 run`` console script registered
    # in setup.py -- the scene is installed as a Python module, not a
    # share data file, so we cannot ``python3 <path>`` it.
    scene_node = Node(
        package="genesis_ros",
        executable="sensor_demo",
        name="sensor_demo_scene",
        output="screen",
    )

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
        OpaqueFunction(function=_robot_state_publisher),
        scene_node,
        OpaqueFunction(function=_rviz),
    ])
