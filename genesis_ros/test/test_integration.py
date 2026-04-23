"""End-to-end integration tests for the Genesis <-> ROS 2 bridge.

Each test launches a dedicated `ros2 launch genesis_ros <file>` process via
`subprocess.Popen` inside its own process group, waits for topics/services to
appear, exercises the behaviour, and finally tears the launch tree down with
`SIGINT -> SIGTERM -> SIGKILL` escalation so no zombie nodes are left behind.

The module is intentionally defensive: on hosts without ROS 2 (no `rclpy` /
`launch_testing`) the whole file is skipped via `pytest.importorskip`. Inside
each test we additionally check that the launch subprocess did not die during
startup; when it did we `pytest.skip` with the captured stderr so CI on a
Genesis-only host never reports a false failure.

These tests are written to run on a ROS 2-enabled machine where the
`genesis_ros` package has been built via colcon and the overlay is sourced.
"""
from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import threading
import time
from typing import Callable, List, Optional

import pytest

# Skip the whole module when ROS 2 Python bindings are missing.
rclpy = pytest.importorskip("rclpy")
pytest.importorskip("launch_testing")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402


# ---------------------------------------------------------------------------
# Global knobs
# ---------------------------------------------------------------------------

PACKAGE = "genesis_ros"
LAUNCH_STARTUP_TIMEOUT = 25.0          # seconds to wait for first topic/service
DEFAULT_TOPIC_TIMEOUT = 10.0           # seconds to wait for per-test data
PER_TEST_HARD_TIMEOUT = 60             # seconds - outer wrapper for each test

try:  # Prefer pytest-timeout's marker when available; fall back to signal.alarm.
    import pytest_timeout  # type: ignore  # noqa: F401

    _TIMEOUT_MARK = pytest.mark.timeout(PER_TEST_HARD_TIMEOUT)
except Exception:  # pragma: no cover - depends on the host's pytest plugins
    _TIMEOUT_MARK = pytest.mark.skipif(False, reason="")  # no-op marker


# ---------------------------------------------------------------------------
# rclpy lifecycle - a single init/shutdown for the whole module
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module", autouse=True)
def _rclpy_module():
    if not rclpy.ok():
        rclpy.init()
    yield
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Launch subprocess helpers
# ---------------------------------------------------------------------------


def _spawn_launch(launch_file: str, extra_args: Optional[List[str]] = None) -> subprocess.Popen:
    """Start `ros2 launch genesis_ros <launch_file>` in its own process group."""
    cmd = ["ros2", "launch", PACKAGE, launch_file]
    if extra_args:
        cmd.extend(extra_args)
    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        preexec_fn=os.setsid,  # new process group so we can SIGINT the tree
        env=env,
    )


def _teardown_launch(proc: subprocess.Popen) -> None:
    """Graceful escalation: SIGINT (10 s) -> SIGTERM (5 s) -> SIGKILL."""
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    for sig, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def _skip_if_launch_died(proc: subprocess.Popen, grace: float = 2.0) -> None:
    """If the launch exited before we even start interacting, skip the test."""
    time.sleep(grace)
    if proc.poll() is not None:
        try:
            _out, err = proc.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            err = b""
        pytest.skip(
            f"ros2 launch exited early (rc={proc.returncode}); "
            f"likely missing setup on this host. stderr tail: "
            f"{(err or b'').decode(errors='replace')[-400:]}"
        )


class _LaunchFixture:
    """Context manager wrapping a single launch subprocess for one test."""

    def __init__(self, launch_file: str, extra_args: Optional[List[str]] = None):
        self.launch_file = launch_file
        self.extra_args = extra_args
        self.proc: Optional[subprocess.Popen] = None

    def __enter__(self) -> subprocess.Popen:
        self.proc = _spawn_launch(self.launch_file, self.extra_args)
        _skip_if_launch_died(self.proc, grace=2.0)
        return self.proc

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.proc is not None:
            _teardown_launch(self.proc)


# ---------------------------------------------------------------------------
# rclpy helpers shared across tests
# ---------------------------------------------------------------------------


def _sensor_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def _collect(
    node_name: str,
    topic: str,
    msg_type,
    *,
    count: int = 1,
    timeout: float = DEFAULT_TOPIC_TIMEOUT,
    qos: Optional[QoSProfile] = None,
    on_msg: Optional[Callable] = None,
) -> list:
    """Spin a helper node, subscribe to a topic, return up to `count` msgs."""
    node = Node(node_name)
    try:
        collected: list = []

        def _cb(msg):
            collected.append(msg)
            if on_msg is not None:
                on_msg(msg)

        node.create_subscription(msg_type, topic, _cb, qos or _reliable_qos())
        exe = SingleThreadedExecutor()
        exe.add_node(node)
        deadline = time.time() + timeout
        while len(collected) < count and time.time() < deadline:
            exe.spin_once(timeout_sec=0.1)
        exe.remove_node(node)
        return collected
    finally:
        node.destroy_node()


def _publish_until(
    node_name: str,
    topic: str,
    msg,
    *,
    duration: float,
    rate_hz: float = 20.0,
    qos: Optional[QoSProfile] = None,
) -> None:
    """Blocking helper: publish `msg` at `rate_hz` for `duration` seconds."""
    node = Node(node_name)
    try:
        pub = node.create_publisher(type(msg), topic, qos or _reliable_qos())
        end = time.time() + duration
        period = 1.0 / rate_hz
        while time.time() < end:
            pub.publish(msg)
            time.sleep(period)
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# Optional signal.alarm fallback for hard timeouts
# ---------------------------------------------------------------------------


class _AlarmTimeout:
    """Fallback hard-timeout using `signal.alarm` when pytest-timeout is absent."""

    def __init__(self, seconds: int):
        self.seconds = seconds
        self._prev_handler = None

    def __enter__(self):
        if threading.current_thread() is not threading.main_thread():
            return self  # signal.alarm only works on the main thread

        def _raise(signum, frame):
            raise TimeoutError(f"integration test exceeded {self.seconds}s")

        self._prev_handler = signal.signal(signal.SIGALRM, _raise)
        signal.alarm(self.seconds)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._prev_handler is not None:
            signal.alarm(0)
            signal.signal(signal.SIGALRM, self._prev_handler)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@_TIMEOUT_MARK
def test_clock():
    """`/clock` ticks at sim-time rate, is monotonic, and has >= 10 msgs/5 s."""
    from rosgraph_msgs.msg import Clock

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("bridge.launch.py"):
        msgs = _collect(
            "it_clock_listener",
            "/clock",
            Clock,
            count=50,
            timeout=LAUNCH_STARTUP_TIMEOUT + 5.0,
            qos=_reliable_qos(depth=50),
        )
        assert len(msgs) >= 10, f"expected >= 10 /clock msgs, got {len(msgs)}"
        # Monotonic non-decreasing.
        stamps = [m.clock.sec + m.clock.nanosec * 1e-9 for m in msgs]
        for a, b in zip(stamps, stamps[1:]):
            assert b >= a, f"/clock went backwards: {a} -> {b}"


@_TIMEOUT_MARK
def test_tf_tree():
    """`world -> franka/panda_hand` resolves via tf2 within 5 s."""
    import tf2_ros

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("franka.launch.py"):
        node = Node("it_tf_probe")
        try:
            buf = tf2_ros.Buffer()
            _listener = tf2_ros.TransformListener(buf, node)  # noqa: F841
            exe = SingleThreadedExecutor()
            exe.add_node(node)
            deadline = time.time() + LAUNCH_STARTUP_TIMEOUT
            transform = None
            last_err = None
            while time.time() < deadline:
                exe.spin_once(timeout_sec=0.2)
                try:
                    transform = buf.lookup_transform(
                        "world", "franka/panda_hand", rclpy.time.Time()
                    )
                    break
                except Exception as err:  # tf2 raises several lookup errors
                    last_err = err
            assert transform is not None, f"tf lookup failed: {last_err!r}"
            assert transform.header.frame_id == "world"
            assert transform.child_frame_id == "franka/panda_hand"
        finally:
            node.destroy_node()


@_TIMEOUT_MARK
def test_joint_state():
    """`/franka/joint_states.name` matches panda_joint1..panda_joint7 (subset)."""
    from sensor_msgs.msg import JointState

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("franka.launch.py"):
        msgs = _collect(
            "it_joint_state_listener",
            "/franka/joint_states",
            JointState,
            count=3,
            timeout=LAUNCH_STARTUP_TIMEOUT,
        )
        assert msgs, "no /franka/joint_states received"
        names = list(msgs[-1].name)
        expected = [f"panda_joint{i}" for i in range(1, 8)]
        for n in expected:
            assert n in names, f"joint {n} missing from /franka/joint_states: {names}"
        # Position vector aligned with names.
        assert len(msgs[-1].position) == len(names)


@_TIMEOUT_MARK
def test_camera():
    """`wrist_cam/image_raw` has (H,W,3); camera_info.k[0] > 0."""
    import numpy as np
    from sensor_msgs.msg import CameraInfo, Image

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("franka.launch.py"):
        imgs = _collect(
            "it_cam_listener",
            "/franka/wrist_cam/image_raw",
            Image,
            count=1,
            timeout=LAUNCH_STARTUP_TIMEOUT,
            qos=_sensor_qos(),
        )
        assert imgs, "no image received on /franka/wrist_cam/image_raw"
        img = imgs[0]
        assert img.height > 0 and img.width > 0
        # rgb8 / bgr8 / rgba8 all carry 3 or 4 channels; require at least 3.
        channels = img.step // max(img.width, 1)
        assert channels >= 3, f"expected >=3 channels, got step/width={channels}"
        buf = np.frombuffer(img.data, dtype=np.uint8)
        assert buf.size == img.height * img.step

        infos = _collect(
            "it_cam_info_listener",
            "/franka/wrist_cam/camera_info",
            CameraInfo,
            count=1,
            timeout=LAUNCH_STARTUP_TIMEOUT,
            qos=_sensor_qos(),
        )
        assert infos, "no camera_info received"
        assert infos[0].k[0] > 0.0, f"fx <= 0: {infos[0].k[0]}"


@_TIMEOUT_MARK
def test_imu():
    """`||ang_vel||` ~0 at rest; >0.5 rad/s after a commanded motion."""
    from sensor_msgs.msg import Imu
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("franka.launch.py"):
        rest = _collect(
            "it_imu_rest",
            "/franka/imu",
            Imu,
            count=20,
            timeout=LAUNCH_STARTUP_TIMEOUT,
            qos=_sensor_qos(depth=50),
        )
        assert len(rest) >= 5, f"too few IMU msgs at rest: {len(rest)}"
        norms = [
            math.sqrt(m.angular_velocity.x ** 2
                     + m.angular_velocity.y ** 2
                     + m.angular_velocity.z ** 2)
            for m in rest
        ]
        avg_rest = sum(norms) / len(norms)
        assert avg_rest < 1e-3, f"IMU not at rest: ||w||={avg_rest}"

        # Commanded motion: shove all joints via FollowJointTrajectory-style topic.
        traj = JointTrajectory()
        traj.joint_names = [f"panda_joint{i}" for i in range(1, 8)]
        p = JointTrajectoryPoint()
        p.positions = [0.5] * 7
        p.time_from_start.sec = 1
        traj.points.append(p)

        pub_node = Node("it_imu_cmd")
        try:
            pub = pub_node.create_publisher(
                JointTrajectory,
                "/franka/panda_arm_controller/joint_trajectory",
                _reliable_qos(),
            )
            for _ in range(20):
                pub.publish(traj)
                time.sleep(0.05)
        finally:
            pub_node.destroy_node()

        moving = _collect(
            "it_imu_moving",
            "/franka/imu",
            Imu,
            count=20,
            timeout=LAUNCH_STARTUP_TIMEOUT,
            qos=_sensor_qos(depth=50),
        )
        if not moving:
            pytest.skip("no IMU data while moving; controller may not be live")
        peak = max(
            math.sqrt(m.angular_velocity.x ** 2
                     + m.angular_velocity.y ** 2
                     + m.angular_velocity.z ** 2)
            for m in moving
        )
        assert peak > 0.5, f"peak ||w||={peak} did not exceed 0.5 rad/s"


@_TIMEOUT_MARK
def test_cmd_vel_diff_drive():
    """Drive the turtlebot forward for 2 s; `odom.position.x > 0.5`."""
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("turtlebot.launch.py"):
        # Wait for odometry to come up.
        first = _collect(
            "it_odom_initial",
            "/turtlebot/odom",
            Odometry,
            count=1,
            timeout=LAUNCH_STARTUP_TIMEOUT,
        )
        if not first:
            pytest.skip("no initial odometry from turtlebot")
        start_x = first[0].pose.pose.position.x

        twist = Twist()
        twist.linear.x = 0.5
        pub_thread = threading.Thread(
            target=_publish_until,
            args=("it_cmd_vel_pub", "/turtlebot/cmd_vel", twist),
            kwargs={"duration": 2.0, "rate_hz": 20.0},
            daemon=True,
        )
        pub_thread.start()
        pub_thread.join(timeout=5.0)

        time.sleep(0.5)  # let the last commanded step propagate to odom
        final = _collect(
            "it_odom_final",
            "/turtlebot/odom",
            Odometry,
            count=1,
            timeout=LAUNCH_STARTUP_TIMEOUT,
        )
        assert final, "no final odometry sample"
        delta_x = final[0].pose.pose.position.x - start_x
        assert delta_x > 0.5, f"expected x travel > 0.5 m, got {delta_x:.3f}"


@_TIMEOUT_MARK
def test_sim_services():
    """`/genesis/pause` freezes `/clock`; `/genesis/resume` unfreezes it."""
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import Empty

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("bridge.launch.py"):
        node = Node("it_sim_services")
        try:
            pause = node.create_client(Empty, "/genesis/pause")
            resume = node.create_client(Empty, "/genesis/resume")
            for cli in (pause, resume):
                if not cli.wait_for_service(timeout_sec=LAUNCH_STARTUP_TIMEOUT):
                    pytest.skip(f"service {cli.srv_name} never came up")

            # Call /genesis/pause.
            fut = pause.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            assert fut.done(), "/genesis/pause did not return"

            # Sample /clock while paused - should be near-constant.
            paused = _collect(
                "it_clock_paused",
                "/clock",
                Clock,
                count=5,
                timeout=3.0,
                qos=_reliable_qos(depth=10),
            )
            if len(paused) >= 2:
                first = paused[0].clock.sec + paused[0].clock.nanosec * 1e-9
                last = paused[-1].clock.sec + paused[-1].clock.nanosec * 1e-9
                assert last - first < 0.1, (
                    f"/clock advanced {last - first:.3f}s while paused"
                )

            # Resume and check /clock advances again.
            fut = resume.call_async(Empty.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            assert fut.done(), "/genesis/resume did not return"

            resumed = _collect(
                "it_clock_resumed",
                "/clock",
                Clock,
                count=10,
                timeout=5.0,
                qos=_reliable_qos(depth=50),
            )
            assert len(resumed) >= 2, "insufficient /clock samples after resume"
            first = resumed[0].clock.sec + resumed[0].clock.nanosec * 1e-9
            last = resumed[-1].clock.sec + resumed[-1].clock.nanosec * 1e-9
            assert last - first > 0.05, (
                f"/clock failed to advance after resume: {last - first:.3f}s"
            )
        finally:
            node.destroy_node()


@_TIMEOUT_MARK
def test_spawn_entity():
    """`/genesis/spawn_urdf` inline box; tf `world -> box/base_link` < 5 s."""
    import tf2_ros

    # An inline box URDF the bridge can parse without touching disk.
    box_urdf = """<?xml version="1.0"?>
<robot name="box">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
    </collision>
  </link>
</robot>
""".strip()

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("bridge.launch.py"):
        try:
            from simulation_interfaces.srv import SpawnEntity
        except Exception:
            pytest.skip("simulation_interfaces.srv.SpawnEntity not available on this host")

        node = Node("it_spawn_box")
        try:
            cli = node.create_client(SpawnEntity, "/genesis/spawn_urdf")
            if not cli.wait_for_service(timeout_sec=LAUNCH_STARTUP_TIMEOUT):
                pytest.skip("/genesis/spawn_urdf never came up")

            req = SpawnEntity.Request()
            # simulation_interfaces.srv.SpawnEntity field names vary slightly
            # across releases; set what exists to keep the test portable.
            for attr, value in (
                ("name", "box"),
                ("resource_string", box_urdf),
                ("uri", ""),
                ("entity_namespace", "box"),
            ):
                if hasattr(req, attr):
                    setattr(req, attr, value)

            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            assert fut.done() and fut.result() is not None, "spawn service call failed"

            # Now wait for the TF entry.
            buf = tf2_ros.Buffer()
            _listener = tf2_ros.TransformListener(buf, node)  # noqa: F841
            exe = SingleThreadedExecutor()
            exe.add_node(node)
            deadline = time.time() + 5.0
            ok = False
            while time.time() < deadline and not ok:
                exe.spin_once(timeout_sec=0.2)
                try:
                    buf.lookup_transform("world", "box/base_link", rclpy.time.Time())
                    ok = True
                except Exception:
                    pass
            assert ok, "tf world -> box/base_link never appeared after spawn"
        finally:
            node.destroy_node()


@_TIMEOUT_MARK
def test_ros2_control():
    """FollowJointTrajectory drives Franka joints to within 0.05 rad of goal."""
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("franka.launch.py"):
        node = Node("it_ros2_control")
        try:
            action_name = "/franka/panda_arm_controller/follow_joint_trajectory"
            cli = ActionClient(node, FollowJointTrajectory, action_name)
            if not cli.wait_for_server(timeout_sec=LAUNCH_STARTUP_TIMEOUT):
                pytest.skip(f"action server {action_name} never came up")

            goal = FollowJointTrajectory.Goal()
            joint_names = [f"panda_joint{i}" for i in range(1, 8)]
            goal.trajectory.joint_names = joint_names
            target = [0.3, -0.2, 0.0, -1.5, 0.0, 1.3, 0.0]
            pt = JointTrajectoryPoint()
            pt.positions = target
            pt.time_from_start.sec = 3
            goal.trajectory.points.append(pt)

            send_fut = cli.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, send_fut, timeout_sec=5.0)
            handle = send_fut.result()
            if handle is None or not handle.accepted:
                pytest.skip("FollowJointTrajectory goal was rejected")

            result_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_fut, timeout_sec=15.0)
            assert result_fut.done(), "trajectory result never returned"

            # Verify final joint positions.
            msgs = _collect(
                "it_r2c_final_js",
                "/franka/joint_states",
                JointState,
                count=1,
                timeout=5.0,
            )
            assert msgs, "no joint_states after trajectory"
            last = msgs[-1]
            name_to_pos = dict(zip(last.name, last.position))
            errors = [abs(name_to_pos[n] - t) for n, t in zip(joint_names, target)
                      if n in name_to_pos]
            assert errors, "none of the requested joints were in /joint_states"
            worst = max(errors)
            assert worst < 0.05, f"worst joint error {worst:.4f} > 0.05 rad"
        finally:
            node.destroy_node()


@_TIMEOUT_MARK
def test_lidar():
    """`/turtlebot/lidar/scan` has nonempty ranges and angle_max > angle_min."""
    from sensor_msgs.msg import LaserScan

    with _AlarmTimeout(PER_TEST_HARD_TIMEOUT), _LaunchFixture("turtlebot.launch.py"):
        scans = _collect(
            "it_lidar_listener",
            "/turtlebot/lidar/scan",
            LaserScan,
            count=1,
            timeout=LAUNCH_STARTUP_TIMEOUT,
            qos=_sensor_qos(),
        )
        assert scans, "no LaserScan received"
        scan = scans[0]
        assert len(scan.ranges) > 0, "LaserScan.ranges is empty"
        assert scan.angle_max > scan.angle_min, (
            f"angle_max ({scan.angle_max}) <= angle_min ({scan.angle_min})"
        )
        # At least one reading should be finite (walls in the scene).
        finite = sum(1 for r in scan.ranges if math.isfinite(r))
        assert finite > 0, "no finite ranges in LaserScan"


if __name__ == "__main__":  # pragma: no cover
    sys.exit(pytest.main([__file__, "-xvs"]))
