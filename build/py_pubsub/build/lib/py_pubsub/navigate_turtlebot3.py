#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time


class TurtlebotNavigator(Node):
    def __init__(self):
        super().__init__('turtlebot_navigator')
        self.navigator = BasicNavigator()

        # Give time to initialize everything
        self.get_logger().info('Waiting for Nav2 to be ready...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active!')

        # Hardcoded goals (PoseStamped)
        self.goals = [
            self.create_pose(1.0, 1.0, 0.0),
            self.create_pose(0.0, 2.0, 1.57),
            self.create_pose(-1.0, 0.0, 3.14)
        ]

        self.send_goals()

    def create_pose(self, x, y, yaw):
        """Creates a PoseStamped message with the given x, y, and yaw."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        import tf_transformations
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def send_goals(self):
        for i, goal in enumerate(self.goals):
            self.get_logger().info(f'Navigating to Goal {i + 1}...')
            self.navigator.goToPose(goal)

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().info(f'Feedback: Distance remaining: {feedback.distance_remaining:.2f}')
                time.sleep(1.0)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Goal {i + 1} succeeded!')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'Goal {i + 1} was canceled.')
            elif result == TaskResult.FAILED:
                self.get_logger().error(f'Goal {i + 1} failed!')

        self.get_logger().info('All goals completed!')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotNavigator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
