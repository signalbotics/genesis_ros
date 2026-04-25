"""Headless smoke test for MoveIt against the Genesis Franka.

Connects to /franka/move_action, builds a MotionPlanRequest with
joint constraints to a target configuration, fires the goal, prints
plan + execution outcome. Run while
``ros2 launch genesis_ros franka_moveit.launch.py`` is up.

Usage:
    ros2 run genesis_ros franka_moveit_test
    ros2 run genesis_ros franka_moveit_test ready
    ros2 run genesis_ros franka_moveit_test extended

No RViz needed -- result + status print to stdout.
"""
from __future__ import annotations

import sys
from typing import List, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


# Named joint configurations, matching the SRDF's <group_state> entries.
_PRESETS = {
    "ready":    [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    "extended": [0.0,  0.0,    0.0, -0.10,  0.0, 1.571, 0.785],
    "zeros":    [0.0,  0.0,    0.0,  0.0,    0.0, 0.0,    0.0],
}

_JOINTS = (
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7",
)


def _build_goal(positions: Sequence[float]):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        Constraints,
        JointConstraint,
        MotionPlanRequest,
        PlanningOptions,
        WorkspaceParameters,
    )
    from geometry_msgs.msg import Vector3

    req = MotionPlanRequest()
    req.group_name = "panda_arm"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3

    ws = WorkspaceParameters()
    ws.header.frame_id = "world"
    ws.min_corner = Vector3(x=-2.0, y=-2.0, z=-2.0)
    ws.max_corner = Vector3(x= 2.0, y= 2.0, z= 2.0)
    req.workspace_parameters = ws

    constraints = Constraints()
    for name, target in zip(_JOINTS, positions):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(target)
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    req.goal_constraints.append(constraints)

    opts = PlanningOptions()
    opts.plan_only = False  # plan AND execute

    goal = MoveGroup.Goal()
    goal.request = req
    goal.planning_options = opts
    return goal, MoveGroup


def main(argv: List[str] | None = None) -> int:
    args = list(argv if argv is not None else sys.argv[1:])
    preset = args[0] if args else "extended"
    if preset not in _PRESETS:
        print(
            "unknown preset '" + preset + "'; available: "
            + ", ".join(_PRESETS.keys()),
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = Node("franka_moveit_test")
    goal, MoveGroup = _build_goal(_PRESETS[preset])
    client = ActionClient(node, MoveGroup, "/franka/move_action")

    node.get_logger().info("waiting for /franka/move_action...")
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("/franka/move_action did not come up in 15 s")
        rclpy.shutdown()
        return 1

    node.get_logger().info("sending goal: preset=" + preset)
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        node.get_logger().error("goal rejected")
        rclpy.shutdown()
        return 1

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    if result is None:
        node.get_logger().error("no result returned")
        rclpy.shutdown()
        return 1

    err = result.result.error_code.val
    node.get_logger().info("done. error_code=" + str(err)
        + " (1=SUCCESS, see moveit_msgs/MoveItErrorCodes for others)")
    rclpy.shutdown()
    return 0 if err == 1 else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))
