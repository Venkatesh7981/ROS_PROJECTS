#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import time

class MPatternDrawer(Node):
    def __init__(self):
        super().__init__('turtlesim_cleaning_robot')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        time.sleep(2)  # Wait for the services to be available

    def move_turtle(self, linear_x, angular_z, duration):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_.publish(twist)
        time.sleep(duration)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def teleport(self, x, y, theta):
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.teleport_client.call_async(req)
        time.sleep(1)

    def set_pen(self, r, g, b, width, off):
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for pen service...')
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        self.pen_client.call_async(req)
        time.sleep(1)

    def draw_m(self):
        self.teleport(1.0, 1.0, 0.0)
        self.set_pen(0, 0, 255, 3, 0)

        forward_time = 1.5
        for i in range(10):
            self.move_turtle(2.0, 0.0, forward_time)
            self.move_turtle(0.0, 3.0 if i % 2 == 0 else -3.0, 0.7)

        self.get_logger().info('Zigzag cleaning pattern complete!')

def main(args=None):
    rclpy.init(args=args)
    m_drawer = MPatternDrawer()
    m_drawer.draw_m()
    m_drawer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
