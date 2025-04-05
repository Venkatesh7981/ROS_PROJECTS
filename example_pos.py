#!/usr/bin/env python3
# Author: You (adapted from Samsung Research example)
# Description: Navigate TurtleBot3 to 3 poses - original goal, back to near start, and forward from goal

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def create_pose(x, y, z=0.0, w=1.0, frame_id='map', navigator=None):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if navigator:
        pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set initial pose (home)
    initial_pose = create_pose(-2.17839, 0.15482, z=0.0, w=1.0, navigator=navigator)
    navigator.setInitialPose(initial_pose)

    # Wait until Nav2 is fully active
    navigator.waitUntilNav2Active()

    # Define three goals
    goals = [
        # Goal 1: Original goal (-0.44, 2.19)
        create_pose(-0.44, 2.19, z=0.0, w=1.0, navigator=navigator),

        # Goal 2: Slightly back near home (initial_pose.x + 0.2, initial_pose.y - 0.2)
        create_pose(1.98, -0.05, z=0.0, w=1.0, navigator=navigator),

        # Goal 3: Slightly in front of original goal (-0.44 + 0.3, 2.19)
        create_pose(0.14, -2.19, z=0.0, w=1.0, navigator=navigator)
    ]

    for i, goal in enumerate(goals):
        print(f"\n[INFO] Navigating to Goal {i + 1}: ({goal.pose.position.x}, {goal.pose.position.y})")
        navigator.goToPose(goal)

        feedback_count = 0
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and feedback_count % 5 == 0:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f"[INFO] Goal {i + 1} ETA: {eta:.1f} sec")
            feedback_count += 1

        # Check result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"[SUCCESS] Goal {i + 1} reached!")
        elif result == TaskResult.CANCELED:
            print(f"[CANCELED] Goal {i + 1} was canceled!")
        elif result == TaskResult.FAILED:
            print(f"[FAILED] Goal {i + 1} failed!")
        else:
            print(f"[ERROR] Unknown status for Goal {i + 1}")

    navigator.lifecycleShutdown()
    print("\n✅ Navigation to all 3 goals complete.")
    exit(0)


if __name__ == '__main__':
    main()

