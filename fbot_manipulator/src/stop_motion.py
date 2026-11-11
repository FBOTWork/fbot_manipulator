#!/usr/bin/env python3
# coding: utf-8
"""CLI: stops the arm mid-motion WITHOUT cutting torque -- it holds at whatever
position it reaches, same as Position Control Mode always does once nothing
new is commanded.

Cancels every active goal (an all-zero action_msgs/GoalInfo means "cancel
all", per the ROS2 action spec) on:
  - /<robot_name>/arm_controller/follow_joint_trajectory (the trajectory the
    arm is actually moving along)
  - /fbot_manipulator/manipulation_task (the higher-level MTC task, so it
    doesn't just send the next stage's trajectory right after)

This does NOT touch torque_enable: never use this to confuse it with an
actual e-stop (torque off) -- that drops the arm limp under gravity, which
is never acceptable for this robot. This only stops the motion in progress;
the motors stay energized and keep holding their last commanded position.

Usage:
    ros2 run fbot_manipulator stop_motion [robot_name]

robot_name defaults to 'wx200' (this arm's xs_sdk namespace).
"""

import sys

import rclpy
from rclpy.node import Node

from action_msgs.srv import CancelGoal


def _cancel_all(node: Node, action_name: str) -> bool:
    """Sends an all-goals CancelGoal request to `action_name`'s hidden cancel service."""
    client = node.create_client(CancelGoal, f'{action_name}/_action/cancel_goal')
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().warn(f"No cancel service for '{action_name}' (not running?).")
        return False

    request = CancelGoal.Request()
    # goal_info left at its default (all-zero goal_id, zero stamp): cancel every goal.
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if not future.done() or future.result() is None:
        node.get_logger().error(f"No response canceling '{action_name}' (timed out).")
        return False

    result = future.result()
    if result.return_code == CancelGoal.Response.ERROR_NONE:
        node.get_logger().warn(
            f"Stopped '{action_name}': {len(result.goals_canceling)} goal(s) canceled."
        )
        return True
    if result.return_code == CancelGoal.Response.ERROR_REJECTED:
        node.get_logger().info(f"'{action_name}': nothing to cancel (no active goal).")
        return True
    node.get_logger().error(f"'{action_name}': cancel rejected, return_code={result.return_code}.")
    return False


def main(args=None):
    rclpy.init(args=args)

    argv = rclpy.utilities.remove_ros_args(args=sys.argv)
    robot_name = argv[1] if len(argv) > 1 else 'wx200'

    node = Node('stop_motion_cli')
    try:
        ok_arm = _cancel_all(node, f'/{robot_name}/arm_controller/follow_joint_trajectory')
        ok_task = _cancel_all(node, '/fbot_manipulator/manipulation_task')
        node.get_logger().warn(
            "Motion stop requested; torque_enable was NOT touched -- the arm should still be "
            "holding its last commanded position."
        )
        sys.exit(0 if (ok_arm or ok_task) else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
