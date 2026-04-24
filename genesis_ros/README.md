# genesis_ros (ROS 2 package)

In-process ROS 2 bridge for the [Genesis](https://github.com/Genesis-Embodied-AI/Genesis)
physics simulator. `rclpy` runs in the same Python process as Genesis —
no external bridge. Surface area matches Gazebo's `gz_ros2_control` and
Isaac Sim's `isaacsim.ros2.bridge`.

- **Publishers:** `/clock`, `/genesis/rtf`, `/tf`, `/tf_static`,
  `/{robot}/joint_states`, `/{robot}/odom`, `/{robot}/pose_ground_truth`,
  cameras (`Image`/`CameraInfo`/depth/`PointCloud2`/segmentation), IMU,
  contact wrenches, proximity `Range`, `Temperature`, raycaster
  (`LaserScan`/`PointCloud2`/`Range`).
- **Subscribers:** `/{robot}/cmd_vel` (free-base + differential-drive),
  `/genesis/set_rtf` (`std_msgs/Float64`, caps real-time factor).
- **Services:**
  `/genesis/{pause,resume,step,reset,spawn_urdf,delete_entity,set_entity_state,get_entity_state,set_gravity,set_physics_properties}` —
  `simulation_interfaces` (REP-2015) types when installed, `std_srvs/Trigger`
  fallback otherwise.
- **Actions:** `/{robot}/follow_joint_trajectory`
  (`control_msgs/action/FollowJointTrajectory`).
- **`ros2_control`:** topic-based via `topic_based_ros2_control`, auto-wired
  from the URDF's `<ros2_control>` block.
- **Bridge params (`ros2 param set /genesis_bridge <k> <v>`):** `rtf_target`
  (real-time-factor cap, `0.0` = unthrottled), `clock_decimation` (publish
  `/clock` every Nth physics tick, default `1`).

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
ros2 run genesis_ros anymal_demo        # ANYmal C quadruped + GUI
ros2 run genesis_ros kuka_demo          # KUKA LBR iiwa 7-DoF arm + GUI
ros2 run genesis_ros shadow_hand_demo   # Shadow Dexterous Hand + GUI
ros2 run genesis_ros drone_demo         # Crazyflie 2.x quadrotor + GUI
ros2 run genesis_ros sensor_demo        # Franka in a kitchen with ALL sensors live
# or with RViz preloaded:
ros2 launch genesis_ros sensor_demo.launch.py
```

Environment knobs:

| Variable | Effect |
|---|---|
| `GENESIS_HEADLESS=1` | Suppress the viewer |
| `GENESIS_VIEWER_FPS=<n>` | Cap viewer at N Hz (default: uncapped) |
| `TURTLEBOT_URDF=/path/to/urdf` | Use a real TurtleBot URDF instead of the box fallback |

Bridge CLI flags (append to `ros2 run genesis_ros genesis_bridge`):

| Flag | Effect |
|---|---|
| `--rtf-target N` | Cap wall-clock to N real-time factor (default: unthrottled) |
| `--clock-decimation N` | Publish `/clock` every Nth physics tick (default: 1) |
| `--env-idx I` | Expose env `I` when `scene.n_envs > 1` (default: 0) |
| `--no-sim-time` | Do not set `use_sim_time=True` on the bridge node |

Sim-control cookbook (works on any running `genesis_bridge`):

```bash
# Pause / resume the sim.
ros2 service call /genesis/pause   std_srvs/srv/Trigger '{}'
ros2 service call /genesis/resume  std_srvs/srv/Trigger '{}'

# Advance N steps while paused (cooperative counter).
ros2 service call /genesis/step simulation_interfaces/srv/StepSimulation \
    '{n_steps: 10}'
# (std_srvs/Trigger fallback: one step per call.)

# Cap the real-time factor at 0.5x; set to 0 to un-throttle.
ros2 topic pub --once /genesis/set_rtf std_msgs/msg/Float64 'data: 0.5'

# Change gravity at runtime.
ros2 param set /genesis_bridge set_gravity.args '{"gravity": [0, 0, -3.71]}'
ros2 service call /genesis/set_gravity std_srvs/srv/Trigger '{}'

# Spawn a URDF (entity addition is deferred until the next reset if the
# scene is already built).
ros2 launch genesis_ros spawn_entity.launch.py \
    name:=cube urdf_path:=/tmp/cube.urdf x:=0.5 z:=0.5
```

Launch files also ship (`launch/franka.launch.py`, `go2.launch.py`,
`sensor_demo.launch.py`) for full RViz + controller_manager setups.
Expected behaviour of each:

- **franka**: full TF chain `world → franka/panda_link0 → … →
  franka/panda_link7`; `/franka/joint_states` at physics rate;
  controller spawner loads `joint_trajectory_controller` +
  `joint_state_broadcaster`.
- **go2**: 12 leg joints in `/go2/joint_states`; `/go2/odom` updated
  from the free base.
- **anymal**: 12 leg joints in `/anymal/joint_states`; `/anymal/odom`
  from the free base. Publishes `/anymal/cmd_vel` for free-base push.
- **kuka**: 7-DoF iiwa, `/kuka/joint_states`.
- **shadow_hand**: 24-joint dexterous hand, `/shadow_hand/joint_states`.
- **drone**: Crazyflie 2.x, `/drone/odom` + ground-truth pose on
  `/drone/pose_ground_truth` for SLAM / VIO reference.
- **sensor_demo**: fixed Franka in a primitives-built kitchen (walls,
  workbench, fridge, pillar, tabletop props). Exercises every sensor
  publisher: wrist RGB+depth camera on `/franka/wrist_cam/*`, EE IMU on
  `/franka/imu`, EE proximity probe on `/franka/ee_proximity`, base
  spherical lidar on `/franka/lidar/points`, per-link contact wrenches
  on `/franka/contacts/*`, ground-truth pose on
  `/franka/pose_ground_truth`. The arm sweeps in a circle so every
  topic updates.

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
└── examples/               franka_scene, go2_scene, anymal_scene,
                            kuka_scene, shadow_hand_scene, drone_scene,
                            sensor_demo
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

## Known gaps vs Gazebo (`gz_ros2_control`)

These are scoped and tracked in `ROS2_INTEGRATION_PLAN.md` under Tier B / C.
Not blockers for typical ROS 2 workflows; listed up front so you hit
them before they bite.

- **Post-build `dt` / `substeps` are immutable.** `/genesis/set_physics_properties`
  can change gravity live, but the timestep and substep count are fixed
  once `scene.build()` runs. Rebuild the scene (or call `/genesis/reset`
  with a fresh config) to change them.
- **No soft-body ROS messages.** MPM / FEM / SPH / PBD solvers run fine
  in Genesis, but no standard ROS message type captures deformable-body
  state. A custom `genesis_msgs` package is out of scope for v1.
- **No native C++ `hardware_interface::SystemInterface` plugin.** v1 stops
  at community `topic_based_ros2_control`. A native `GenesisSystem` that
  bypasses the topic hop is future work (Tier C) — relevant only for
  1 kHz+ control loops where topic latency starts to matter.
- **No joint-level force/torque sensor publisher yet** (Tier B-8). Contact
  wrenches on links are supported; joint F/T needs separate kinematics.
- **No stereo-camera paired publisher** (Tier B-9). Two cameras can be
  registered separately; shared-stamp stereo pairs are follow-up.
- **`<mimic>` joint handling unverified** (Tier B-11). Should work via
  qpos, but TF / joint_states have not been runtime-tested against a URDF
  that uses `<mimic>`.
- **No SDF import** (Tier C-14). URDF / xacro only. Gazebo worlds need
  hand-translation to URDF.
- **Single-environment bridge.** When `scene.n_envs > 1`, only `env_idx`
  (default 0) is published. Parallel-env publishing is on the roadmap.
- **Gazebo plugin ABI not supported.** The `gz-sim` plugin API is its
  own ecosystem and is not in scope.
- **No buoyancy / hydrodynamics / wind / aerodynamics** — these need
  engine-level additions upstream in Genesis itself.
- **No GUI-interactive editing.** Genesis's pyrender viewer does not
  support force / teleport widgets; a viewer rewrite would be needed.

## Platform & ecosystem limits

- **ROS 1 unsupported.** Bridge is `rclpy`-only; ROS 1 would need a
  separate `ros1_bridge` hop that we don't provide.
- **Windows unsupported.** `rclpy` and `topic_based_ros2_control` are
  first-class only on Linux.
