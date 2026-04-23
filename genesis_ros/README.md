# genesis_ros (ROS 2 package)

In-process ROS 2 bridge for the [Genesis](https://github.com/Genesis-Embodied-AI/Genesis)
physics simulator. `rclpy` runs in the same Python process as Genesis —
no external bridge. Surface area matches Gazebo's `gz_ros2_control` and
Isaac Sim's `isaacsim.ros2.bridge`.

- **Publishers:** `/clock`, `/genesis/rtf`, `/tf`, `/tf_static`,
  `/{robot}/joint_states`, `/{robot}/odom`, cameras
  (`Image`/`CameraInfo`/depth/`PointCloud2`/segmentation), IMU, contact
  wrenches, proximity `Range`, `Temperature`, raycaster
  (`LaserScan`/`PointCloud2`/`Range`).
- **Subscribers:** `/{robot}/cmd_vel` (free-base + differential-drive).
- **Services:** `/genesis/{pause,resume,step,reset,spawn_urdf,delete_entity,set_entity_state,get_entity_state}` —
  `simulation_interfaces` (REP-2015) types when installed, `std_srvs/Trigger`
  fallback otherwise.
- **Actions:** `/{robot}/follow_joint_trajectory`
  (`control_msgs/action/FollowJointTrajectory`).
- **`ros2_control`:** topic-based via `topic_based_ros2_control`, auto-wired
  from the URDF's `<ros2_control>` block.

## Execution model (Isaac-style, in-process)

- `rclpy` lives in the same Python process as `genesis`.
- `scene.step()` runs on the **main thread only**.
- `rclpy.executors.MultiThreadedExecutor` runs on a background daemon thread.
- Subscribers stage incoming messages into a `threading.Lock`-guarded dict;
  `bridge.spin()` flushes them before each `scene.step()`.
- Publishers' `step()` is called on the main thread right after
  `scene.step()`. Publishing itself is thread-safe in `rclpy`.

No custom message package is introduced. Standard `sensor_msgs`,
`geometry_msgs`, `nav_msgs`, `std_msgs`, `tf2_msgs`, `rosgraph_msgs`,
`control_msgs`, `std_srvs` only, plus the community `simulation_interfaces`
types fetched through `rosdep`.

---

## Install

Easiest: install the three `.deb`s built from this repo's `packaging/`
directory — see `../packaging/README.md`.

Source-tree use inside an existing ROS 2 workspace:

```bash
cd <your-ros2-ws>/src
ln -s <path-to-this-repo>/genesis_ros .
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select genesis_ros --symlink-install
source install/setup.bash
```

Prerequisites for source builds:

- ROS 2 Humble or newer (Iron / Jazzy also supported).
- A Python environment where `import genesis` succeeds (pip / conda / venv
  with `genesis==0.4.6` + its GPU stack).
- `colcon-common-extensions`, `rosdep` initialised.

The `.deb` path drops a `.pth` file that exposes `/opt/genesis/venv/lib/python3.12/site-packages`
to the system Python so `ros2 run` can import Genesis. Source builds need
you to manage that yourself.

### Source order matters

Both the Genesis Python environment **and** the ROS 2 overlay must be in
scope. Source the overlay **last**:

```bash
source /path/to/.venv/bin/activate      # Genesis env first
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash     # workspace overlay last
```

If `ros2 launch` can't find `genesis_ros`, the overlay wasn't sourced last.

---

## Runnable demos (console entry points)

After install:

```bash
ros2 run genesis_ros genesis_bridge     # empty scene — just /clock
ros2 run genesis_ros franka_demo        # Franka Panda arm + GUI
ros2 run genesis_ros go2_demo           # Unitree Go2 quadruped + GUI
ros2 run genesis_ros turtlebot_demo     # Diff-drive base (or custom URDF)
```

Environment knobs:

| Variable | Effect |
|---|---|
| `GENESIS_HEADLESS=1` | Suppress the viewer |
| `GENESIS_VIEWER_FPS=<n>` | Cap viewer at N Hz (default: uncapped) |
| `TURTLEBOT_URDF=/path/to/urdf` | Use a real TurtleBot URDF instead of the box fallback |

Launch files also ship (`launch/franka.launch.py`, `go2.launch.py`,
`turtlebot.launch.py`) for full RViz + controller_manager setups. Expected
behaviour of each:

- **franka**: full TF chain `world → franka/base_link → … → franka/panda_hand`;
  `/franka/joint_states` at physics rate; wrist camera on
  `/franka/wrist_cam/image_raw`; IMU on `/franka/imu`; controller spawner
  loads `panda_arm_controller` + `joint_state_broadcaster`.
- **go2**: 12 leg joints in `/go2/joint_states`; body IMU on `/go2/imu`;
  per-foot contacts on `/go2/contacts/{FL,FR,RL,RR}_foot`.
- **turtlebot**: 2D lidar on `/turtlebot/lidar/scan`; `/turtlebot/odom`
  updates when you publish `/turtlebot/cmd_vel`; TF chain
  `world → turtlebot/odom → turtlebot/base_link`.

---

## Topics, frames, QoS (shared conventions)

| Concern              | Default                                                              |
| -------------------- | -------------------------------------------------------------------- |
| Root node            | `genesis_bridge`                                                     |
| Clock                | `/clock`                                                             |
| RTF                  | `/genesis/rtf` (`std_msgs/Float64`)                                  |
| TF                   | `/tf`, `/tf_static`                                                  |
| Per-robot namespace  | `/{robot}` (e.g. `/franka`)                                          |
| JointState           | `/{robot}/joint_states`                                              |
| Odom                 | `/{robot}/odom`                                                      |
| Cmd vel              | `/{robot}/cmd_vel`                                                   |
| Camera               | `/{robot}/{cam}/{image_raw,depth,camera_info,points,segmentation}`   |
| IMU                  | `/{robot}/{sensor_name}`                                             |
| Lidar 2D             | `/{robot}/{sensor_name}/scan`                                        |
| Lidar 3D             | `/{robot}/{sensor_name}/points`                                      |
| Contact              | `/{robot}/contacts/{link}` (`geometry_msgs/WrenchStamped`)           |
| HW command topic     | `/{robot}/joint_commands` (`sensor_msgs/JointState`)                 |
| Base frame           | `{robot}/base_link`                                                  |
| Odom frame           | `{robot}/odom`                                                       |
| World frame          | `world`                                                              |

QoS presets in `genesis_ros.qos`:

| Preset          | Reliability   | History          | Durability       |
| --------------- | ------------- | ---------------- | ---------------- |
| `SENSOR_QOS`    | BEST_EFFORT   | KEEP_LAST(5)     | VOLATILE         |
| `CLOCK_QOS`     | RELIABLE      | KEEP_LAST(1)     | VOLATILE         |
| `TF_QOS`        | RELIABLE      | KEEP_LAST(100)   | VOLATILE         |
| `TF_STATIC_QOS` | RELIABLE      | KEEP_LAST(1)     | TRANSIENT_LOCAL  |
| `STATE_QOS`     | RELIABLE      | KEEP_LAST(10)    | VOLATILE         |

---

## Drop your own URDF

Three things needed:

1. A URDF (or xacro) for the robot.
2. A `<ros2_control>` tag inside the URDF pointing at
   `topic_based_ros2_control/TopicBasedSystem`.
3. A Python entry that builds a Genesis scene and registers the robot.

### URDF `<ros2_control>` block

```xml
<ros2_control name="panda" type="system">
  <hardware>
    <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
    <param name="joint_commands_topic">/franka/joint_commands</param>
    <param name="joint_states_topic">/franka/joint_states</param>
    <param name="sum_wrapped_joint_states">false</param>
  </hardware>
  <joint name="panda_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- repeat for panda_joint2..panda_joint7 -->
</ros2_control>
```

### Python scene + bridge wiring

```python
import genesis as gs
from genesis_ros.node import GenesisRosBridge
from genesis_ros.control.urdf_ros2_control import auto_register_control

gs.init()
scene = gs.Scene(show_viewer=False)
scene.add_entity(gs.morphs.Plane())

urdf_path = "/robots/panda.urdf"
panda = scene.add_entity(gs.morphs.URDF(file=urdf_path, fixed=True))
scene.build()

bridge = GenesisRosBridge(scene, node_name="genesis_bridge")
with open(urdf_path) as f:
    bridge.register_entity(panda, name="franka", urdf_xml=f.read())

# Parse <ros2_control> and wire up matching publishers/subscribers.
auto_register_control(bridge, bridge.registry)

bridge.spin()
```

### Controller YAML (consumed by `topic_based_ros2_control`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    panda_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

panda_arm_controller:
  ros__parameters:
    joints: [panda_joint1, panda_joint2, panda_joint3, panda_joint4,
             panda_joint5, panda_joint6, panda_joint7]
    command_interfaces: [position]
    state_interfaces:   [position, velocity]
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
```

---

## Writing a custom publisher

Subclass `GenesisPublisher`, implement `step()`, register on the bridge.
The base class owns the `rclpy.Node`, the registry handle, and the QoS
presets; your subclass converts Genesis tensors to ROS messages.

```python
from std_msgs.msg import Float32
from genesis_ros.node import GenesisPublisher, GenesisRosBridge
from genesis_ros.qos import STATE_QOS

class BatteryPublisher(GenesisPublisher):
    def __init__(self, node, scene, registry, cfg):
        super().__init__(node, scene, registry, cfg)
        ns = cfg["robot"]
        self._pub = node.create_publisher(Float32, f"/{ns}/battery", STATE_QOS)
        self._v0 = cfg.get("initial_voltage", 12.6)
        self._drain = cfg.get("drain_per_sec", 0.001)

    def step(self, sim_time):
        t = sim_time.sec + sim_time.nanosec * 1e-9
        msg = Float32()
        msg.data = max(0.0, self._v0 - self._drain * t)
        self._pub.publish(msg)

bridge.register_publisher(
    BatteryPublisher(bridge.node, scene, bridge.registry,
                     cfg={"robot": "franka", "initial_voltage": 12.6}),
)
```

Subscribers follow the same pattern: subclass `GenesisSubscriber`, stage
incoming messages in the callback, implement `apply(scene)` for the
main-thread flush. `bridge.spin()` calls `apply()` right before
`scene.step()`.

---

## Module layout

```
genesis_ros/
├── node.py                 base classes + GenesisRosBridge orchestrator
├── bridge.py               genesis_bridge console entry point
├── conversions.py          tensor ↔ ROS helpers
├── qos.py                  QoS presets
├── entity_registry.py      entity ↔ ROS namespace mapping
├── publishers/             clock, tf, joint_state, odom, camera,
│                           imu, contact, proximity, temperature, raycaster
├── subscribers/            cmd_vel
├── services/               sim_control, entity (spawn/delete/get/set)
├── actions/                follow_joint_trajectory
├── control/                URDF <ros2_control> parsing + topic HW bridge
└── examples/               franka_scene, go2_scene, turtlebot_scene
```

---

## Troubleshooting

**Sensor topics empty in RViz** — bridge publishes with `SENSOR_QOS`
(BEST_EFFORT); RViz defaults to RELIABLE. In RViz, set the display's
**Reliability Policy** to `Best Effort`, or add
`--qos-reliability best_effort` to `ros2 topic echo`.

**`use_sim_time` not propagating** — only `genesis_bridge` sets it
automatically. Every other node you launch alongside must include
`parameters=[{"use_sim_time": True}]`; `robot_state_publisher` and
`controller_manager` in particular must honour it for TF stamps to line up
with `/clock`.

**Controller spawner race** — `controller_spawner` exits with "Timed out
waiting for controller manager". The stock launch files wrap each spawner
in `RegisterEventHandler(OnProcessStart(target=bridge_process, …))`; do
the same in custom launches or raise `--controller-manager-timeout`.

**RTF drops at high topic count** — `rclpy` callbacks contend for the GIL
with `scene.step()`. Mitigations: lower `clock_decimation`, reduce camera
resolution, use `SENSOR_QOS` depth=1 for heavy topics, prefer `cv_bridge`'s
zero-copy image conversions.

**`ros2 launch` exits immediately** — typically Genesis failed to init the
GPU backend (try `GENESIS_BACKEND=cpu`), `simulation_interfaces` is missing
(run `rosdep install`), or the overlay isn't sourced last.

---

## Limitations

- **No soft-body ROS messages yet.** MPM/FEM/SPH/PBD solvers run fine in
  Genesis, but no standard ROS message type captures deformable-body state.
  A custom `genesis_msgs` package is out of scope for v1.
- **ROS 1 unsupported.** Bridge is `rclpy`-only; ROS 1 would need a
  separate `ros1_bridge` hop that we don't provide.
- **Windows unsupported.** `rclpy` and `topic_based_ros2_control` are
  first-class only on Linux.
- **Native C++ `SystemInterface` deferred.** v1 stops at
  `topic_based_ros2_control`. A native `GenesisSystem` that bypasses the
  topic hop is future work.
- **Single-environment bridge.** When `scene.n_envs > 1`, only `env_idx=0`
  is published. Parallel-env publishing is on the roadmap.
