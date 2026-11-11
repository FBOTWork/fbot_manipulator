#!/usr/bin/env python3
# coding: utf-8
"""CLI: sends the arm to its 'home' named pose.

Calls manipulator_interface_node's /fbot_manipulator/move_to_named_target
service (fbot_manipulator_msgs/srv/MoveToNamedTarget) with target_name='home'
-- the joint positions under that key in
config/interbotix_arm/manipulator_config.yaml ('poses.home', currently
[0.0, 0.0, 0.0, 0.0, 0.0]). This plans and executes a normal MoveIt motion
to there, same as any other named-target move (not an instant teleport, and
not usable while another motion/task is in progress -- stop it first with
stop_motion if needed).

Usage:
    ros2 run fbot_manipulator go_home [target_name]

target_name defaults to 'home'; pass another key from that same
'poses' section (e.g. 'Sleep') to reuse this CLI for those too.
"""

import sys

import rclpy
from rclpy.node import Node

from fbot_manipulator_msgs.srv import MoveToNamedTarget


def main(args=None):
    rclpy.init(args=args)

    argv = rclpy.utilities.remove_ros_args(args=sys.argv)
    target_name = argv[1] if len(argv) > 1 else 'home'

    node = Node('go_home_cli')
    service_name = '/fbot_manipulator/move_to_named_target'
    client = node.create_client(MoveToNamedTarget, service_name)

    try:
        if not client.wait_for_service(timeout_sec=3.0):
            node.get_logger().error(f"Service '{service_name}' not available.")
            sys.exit(1)

        request = MoveToNamedTarget.Request()
        request.target_name = target_name

        node.get_logger().info(f"Moving to named target '{target_name}'...")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)

        if not future.done() or future.result() is None:
            node.get_logger().error("No response from move_to_named_target (timed out).")
            sys.exit(1)

        result = future.result()
        if result.success:
            node.get_logger().info(f"Reached '{target_name}'.")
            sys.exit(0)
        else:
            node.get_logger().error(f"Failed: {result.message}")
            sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
