#!/usr/bin/env python3

import rclpy
import random
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
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


def get_random_goal(navigator, x_range=(-2.0, 2.0), y_range=(-2.0, 2.0)):
    x = random.uniform(*x_range)
    y = random.uniform(*y_range)
    print(f"[INFO] New random goal: x={x:.2f}, y={y:.2f}")
    return create_pose(x, y, z=0.0, w=1.0, navigator=navigator)


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = create_pose(-2.0, 0.0, z=0.0, w=1.0, navigator=navigator)
    navigator.setInitialPose(initial_pose)

    # Wait until Nav2 is active
    navigator.waitUntilNav2Active()

    try:
        while rclpy.ok():
            goal_pose = get_random_goal(navigator)

            navigator.goToPose(goal_pose)

            i = 0
            last_eta = None
            eta_stuck_counter = 0

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    print(f"[INFO] ETA: {eta:.1f} sec")

                    # Obstacle detection heuristic
                    if last_eta is not None:
                        if abs(eta - last_eta) < 1.0:  # Little to no change in ETA
                            eta_stuck_counter += 1
                        else:
                            eta_stuck_counter = 0
                    last_eta = eta

                    if eta_stuck_counter >= 3:
                        print("[OBSTACLE DETECTED] Robot might be stuck or avoiding something!")
                        eta_stuck_counter = 0  # Reset to prevent spamming

                i += 1

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("[SUCCESS] Goal reached!")
            elif result == TaskResult.CANCELED:
                print("[INFO] Goal was canceled.")
            elif result == TaskResult.FAILED:
                print("[ERROR] Goal failed. Trying next random goal.")
            else:
                print("[WARN] Unknown result status. Moving on.")

    except KeyboardInterrupt:
        print("\n[INFO] Navigation interrupted by user.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

