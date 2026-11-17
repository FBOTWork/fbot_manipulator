#!/usr/bin/env python3
# coding: utf-8
"""Node: stays subscribed to /emergency_button (std_msgs/Bool) and, on True, locks
the arm the same way `stop_motion` does -- cancels every active goal (an all-zero
action_msgs/GoalInfo means "cancel all", per the ROS2 action spec) on:
  - /<robot_name>/arm_controller/follow_joint_trajectory (the trajectory the
    arm is actually moving along)
  - /fbot_manipulator/manipulation_task (the higher-level MTC task, so it
    doesn't just send the next stage's trajectory right after)

This does NOT touch torque_enable: never confuse this with an actual e-stop
(torque off) -- that drops the arm limp under gravity, which is never
acceptable for this robot. This only stops the motion in progress; the motors
stay energized and keep holding their last commanded position.

Every True message re-locks the arm, so a goal sent while the button is held
down gets canceled on the next message too.

Usage:
    ros2 run fbot_manipulator estop_watchdog --ros-args -p robot_name:=wx200
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from action_msgs.srv import CancelGoal
from std_msgs.msg import Bool


class EstopWatchdog(Node):
    def __init__(self):
        super().__init__('estop_watchdog')

        self.declare_parameter('robot_name', 'wx200')
        robot_name = self.get_parameter('robot_name').value

        self._cancel_clients = {
            action_name: self.create_client(CancelGoal, f'{action_name}/_action/cancel_goal')
            for action_name in (
                f'/{robot_name}/arm_controller/follow_joint_trajectory',
                '/fbot_manipulator/manipulation_task',
            )
        }

        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/emergency_button', self._emergency_cb, qos)
        self.get_logger().info("Estop watchdog ready: subscribed to '/emergency_button'.")

    def _emergency_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.get_logger().warn(
            "Emergency button pressed: locking the arm (motion stop; torque stays on).")
        for action_name, client in self._cancel_clients.items():
            self._cancel_all(action_name, client)

    def _cancel_all(self, action_name: str, client) -> None:
        """Sends an all-goals CancelGoal request to `action_name`'s hidden cancel service."""
        if not client.service_is_ready():
            self.get_logger().warn(f"No cancel service for '{action_name}' (not running?).")
            return
        # goal_info left at its default (all-zero goal_id, zero stamp): cancel every goal.
        future = client.call_async(CancelGoal.Request())
        future.add_done_callback(lambda f, name=action_name: self._on_cancel_done(name, f))

    def _on_cancel_done(self, action_name: str, future) -> None:
        result = future.result()
        if result is None:
            self.get_logger().error(f"'{action_name}': cancel call failed.")
            return
        if result.return_code == CancelGoal.Response.ERROR_NONE:
            self.get_logger().warn(
                f"Locked '{action_name}': {len(result.goals_canceling)} goal(s) canceled.")
        elif result.return_code == CancelGoal.Response.ERROR_REJECTED:
            self.get_logger().info(f"'{action_name}': nothing to cancel (no active goal).")
        else:
            self.get_logger().error(
                f"'{action_name}': cancel rejected, return_code={result.return_code}.")


def main(args=None):
    rclpy.init(args=args)
    node = EstopWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
