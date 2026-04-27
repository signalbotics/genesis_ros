"""ament_python setup for genesis_ros."""
import os
from glob import glob
from setuptools import find_packages, setup

package_name = "genesis_ros"


def _data_files():
    """Collect launch/ and config/ recursively.

    ``launch/`` and ``config/`` may not exist yet (other groups own them). Using
    :func:`glob.glob` with ``recursive=True`` returns an empty list for missing
    roots, so the install step simply skips them in that case.
    """
    entries = [
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    # Collect launch files.
    launch_dir = "launch"
    launch_files = [
        f for f in glob(os.path.join(launch_dir, "**", "*"), recursive=True)
        if os.path.isfile(f)
    ]
    if launch_files:
        # Preserve subdirectory structure so e.g. launch/rviz/foo.rviz installs
        # under share/genesis_ros/launch/rviz/foo.rviz.
        grouped: dict[str, list[str]] = {}
        for f in launch_files:
            rel_dir = os.path.dirname(f)
            grouped.setdefault(rel_dir, []).append(f)
        for rel_dir, paths in grouped.items():
            entries.append((os.path.join("share", package_name, rel_dir), paths))

    # Collect config + urdf (xacro) files as share data so launches can
    # resolve them via get_package_share_directory("genesis_ros").
    for sub_dir in ("config", "urdf"):
        files = [
            f for f in glob(os.path.join(sub_dir, "**", "*"), recursive=True)
            if os.path.isfile(f)
        ]
        if not files:
            continue
        grouped: dict = {}
        for f in files:
            rel_dir = os.path.dirname(f)
            grouped.setdefault(rel_dir, []).append(f)
        for rel_dir, paths in grouped.items():
            entries.append((os.path.join("share", package_name, rel_dir), paths))

    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Genesis ROS Maintainers",
    maintainer_email="bruk@signalbotics.com",
    description=(
        "ROS 2 bridge for the Genesis physics simulator. In-process rclpy "
        "bridge publishing /clock, /tf, /joint_states, sensor topics and "
        "driving the sim from /cmd_vel, FollowJointTrajectory actions and "
        "sim-control services."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "genesis_bridge = genesis_ros.bridge:main",
            "franka_demo = genesis_ros.examples.franka_scene:main",
            "go2_demo = genesis_ros.examples.go2_scene:main",
            "anymal_demo = genesis_ros.examples.anymal_scene:main",
            "kuka_demo = genesis_ros.examples.kuka_scene:main",
            "shadow_hand_demo = genesis_ros.examples.shadow_hand_scene:main",
            "drone_demo = genesis_ros.examples.drone_scene:main",
            "sensor_demo = genesis_ros.examples.sensor_demo:main",
            "franka_moveit_test = genesis_ros.examples.franka_moveit_test:main",
            # Pure-Genesis RL pipelines (no ROS in the loop). Train offline
            # with these; deploy a trained policy via the matching `*_demo`
            # scene + a policy loader. Requires rsl-rl-lib>=5.0.0.
            "go2_train = genesis_ros.examples.locomotion.go2_train:main",
            "go2_eval = genesis_ros.examples.locomotion.go2_eval:main",
            "go2_backflip = genesis_ros.examples.locomotion.go2_backflip:main",
            "go2_backflip_train = genesis_ros.examples.locomotion.go2_backflip_train:main",
            "hover_train = genesis_ros.examples.drone_rl.hover_train:main",
            "hover_eval = genesis_ros.examples.drone_rl.hover_eval:main",
            "grasp_train = genesis_ros.examples.manipulation_rl.grasp_train:main",
            "grasp_eval = genesis_ros.examples.manipulation_rl.grasp_eval:main",
        ],
    },
)
