# genesis_ros

ROS 2 bridge for the [Genesis](https://github.com/Genesis-Embodied-AI/Genesis)
physics simulator — plus Debian packaging for Ubuntu 24.04 (Noble) /
ROS 2 Jazzy.

Matches the surface area of Gazebo's `gz_ros2_control` and Isaac Sim's
`isaacsim.ros2.bridge`: `/clock`, `/tf`, `/joint_states`, `/odom`, camera /
IMU / contact / lidar / range / temperature sensors, `/cmd_vel`,
`FollowJointTrajectory`, and sim-control services (pause / step / reset /
spawn / delete / get-state / set-state).

## Layout

```
genesis_ros/          the ament_python ROS 2 package
  publishers/         clock, tf, joint_state, odom, camera, imu, contact, ...
  subscribers/        cmd_vel
  services/           sim_control, entity
  actions/            follow_joint_trajectory
  control/            topic_based_ros2_control wiring
  examples/           franka_demo, go2_demo, turtlebot_demo

launch/               bridge, franka, go2, turtlebot launch files
config/               controller YAMLs + RViz presets
test/                 pytest / launch_testing

packaging/            Debian .deb build (3 packages)
  genesis-world/         engine + vendored Python 3.12 venv (with torch)
  genesis-world-assets/  URDF / meshes / textures under /opt/genesis/assets
  ros-jazzy-genesis-ros/ the ROS 2 bridge package (via bloom)
```

## Building the Debian packages

Requires Docker. The Genesis engine source lives *outside* this repo.
Point to it with `GENESIS_SRC`, or place a Genesis checkout at
`../Genesis` (sibling directory — the default).

```bash
# One command:
./packaging/build-in-docker.sh

# Or point at a non-default Genesis checkout:
GENESIS_SRC=/path/to/Genesis ./packaging/build-in-docker.sh
```

Artifacts land in `packaging/out/`. See `packaging/README.md` for the
bare-metal (no-Docker) build path and hosting via a local apt repo.

## Using the ROS 2 package

After installing the three `.deb`s:

```bash
sudo apt install ./packaging/out/genesis-world_*.deb \
                 ./packaging/out/genesis-world-assets_*.deb \
                 ./packaging/out/ros-jazzy-genesis-ros_*.deb

source /opt/ros/jazzy/setup.bash
ros2 run genesis_ros franka_demo        # Franka arm with GUI
ros2 run genesis_ros go2_demo           # Unitree Go2 quadruped with GUI
ros2 run genesis_ros turtlebot_demo     # Diff-drive base
ros2 run genesis_ros genesis_bridge     # Empty scene (just /clock)
```

Environment knobs for the demos:

| Variable | Effect |
|---|---|
| `GENESIS_HEADLESS=1` | Don't open the viewer |
| `GENESIS_VIEWER_FPS=<n>` | Cap the viewer at N Hz (default: uncapped) |
| `TURTLEBOT_URDF=/path/to/urdf` | Use your own turtlebot URDF (fallback is a box) |

## License

Apache-2.0. See `genesis_ros/package.xml` for dependency list.
