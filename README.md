# genesis_ros

ROS 2 bridge for the [Genesis](https://github.com/Genesis-Embodied-AI/Genesis)
physics simulator — plus Debian packaging for Ubuntu 24.04 (Noble) /
ROS 2 Jazzy.

Matches the surface area of Gazebo's `gz_ros2_control` and Isaac Sim's
`isaacsim.ros2.bridge`: `/clock`, `/tf`, `/joint_states`, `/odom`, camera /
IMU / contact / lidar / range / temperature sensors, `/cmd_vel`,
`FollowJointTrajectory`, and sim-control services (pause / step / reset /
spawn / delete / get-state / set-state).

## Why genesis_ros?

| You want… | Best pick |
|---|---|
| Drop a URDF in and see telemetry in 30 minutes | **genesis_ros** |
| Train RL with hundreds of parallel envs on one GPU | **genesis_ros** |
| Photorealistic cameras for VLA / vision training | Isaac Sim |
| PX4 / ArduPilot SITL out of the box | Gazebo |
| The most public examples and Stack Overflow answers | Gazebo |

**Where genesis_ros wins:**

- **Pure Python.** No SDF, no USD, no Action Graphs. If you can write a Python script, you can author a scene.
- **Same process as `rclpy`.** No IPC, no bridge process — your ROS callbacks call into the simulator directly.
- **GPU-batched physics.** `n_envs=1024` in one launch file. Roughly 10× faster than Gazebo on multi-env workloads.
- **One-line install.** Three `.deb` packages on Jazzy. No 30 GB Omniverse install.

**Where it's still young:**

- Fewer pre-made robot examples than Gazebo (Franka, Go2, ANYmal, KUKA, Shadow Hand, drone today).
- No PX4 / ArduPilot SITL bridge yet.
- Documentation is this README plus the example scenes — no community Q&A history.

If you're starting fresh and your work is heavy on physics and RL throughput, this is the fast lane. If you need a mature ecosystem of robot models and autopilots, Gazebo is the safer pick today.

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

## Reinforcement learning

Genesis ships three RL pipelines (locomotion, drone hover, grasping). They
run as plain `ros2 run` console scripts — no ROS in the training loop, just
GPU-batched Genesis with thousands of parallel envs. Train offline, then
deploy a `.pt` policy on top of the matching `*_demo` scene.

```bash
# One-time: install the trainer (not vendored in the deb)
/opt/genesis/venv/bin/pip install 'rsl-rl-lib>=5.0.0' tensordict

# Locomotion — Unitree Go2 PPO walking
ros2 run genesis_ros go2_train -B 4096 --max_iterations 1000
ros2 run genesis_ros go2_eval  -e go2-walking --ckpt 1000
ros2 run genesis_ros go2_backflip                  # advanced reward shaping

# Drone — quadrotor hover
ros2 run genesis_ros hover_train
ros2 run genesis_ros hover_eval

# Manipulation — Franka grasping
ros2 run genesis_ros grasp_train
ros2 run genesis_ros grasp_eval
```

Checkpoints land in `./logs/<exp_name>/`. Eval scripts open the viewer
and replay the policy.

## License

Apache-2.0. See `genesis_ros/package.xml` for dependency list.
