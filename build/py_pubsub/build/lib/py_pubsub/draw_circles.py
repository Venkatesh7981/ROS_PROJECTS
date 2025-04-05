#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import random

class DrawCircles(Node):
    def __init__(self):
        super().__init__('draw_circles')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cli_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.cli_setpen = self.create_client(SetPen, '/turtle1/set_pen')
        self.timer = self.create_timer(2.0, self.draw_circle)

    def set_pen(self, r, g, b, width, off):
        while not self.cli_setpen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = SetPen.Request()
        req.r = int(r)
        req.g = int(g)
        req.b = int(b)
        req.width = int(width)
        req.off = int(off)
        self.cli_setpen.call_async(req)

    def teleport(self, x, y, theta):
        while not self.cli_teleport.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self.cli_teleport.call_async(req)

    def draw_circle(self):
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        radius = random.uniform(0.5, 2.0)  # Random radius between 0.5 and 2.0
        self.set_pen(0, 0, 0, 2, 1)  # Turn off the pen
        self.teleport(x, y, 0.0)
        pen_width = 5  # Random pen width between 2 and 5
        self.set_pen(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255), pen_width, 0)  # Turn on the pen with random color and width
        move_cmd = Twist()
        move_cmd.linear.x = radius
        move_cmd.angular.z = 1.0
        for _ in range(72):  # 72 steps to complete a circle
            self.publisher_.publish(move_cmd)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

def main(args=None):
    rclpy.init(args=args)
    draw_circles = DrawCircles()
    rclpy.spin(draw_circles)
    draw_circles.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
