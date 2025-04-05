import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute
import time
import math

class DrawLetters(Node):
    def __init__(self):
        super().__init__('draw_letters')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.pen_client.wait_for_service(timeout_sec=1.0) or not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for services...')
        time.sleep(1)
        self.draw_aiml()

    def move(self, linear, angular, duration):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)
        time.sleep(duration)
        self.stop()

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)
        time.sleep(0.1)

    def set_pen(self, r, g, b, width, off):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        self.pen_client.call_async(request)
        time.sleep(1)

    def teleport(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.teleport_client.call_async(request)
        time.sleep(1)

    def draw_aiml(self):
        start_x, start_y = 2.0, 5.0
        letter_spacing = 2.0
        rotation = math.pi / 2

        # Draw 'A'
        self.teleport(start_x, start_y, rotation)
        self.set_pen(255, 0, 0, 3, 0)
        self.move(2.0, 0.0, 1.0)
        self.move(0.0, -3.0, 1.0)
        self.move(2.0, 0.0, 1.0)
        self.move(0.0, 3.0, 1.0)
        self.move(1.0, 0.0, 0.5)
        self.move(0.0, 1.5, 1.0)
        self.move(1.0, 0.0, 0.4)

        # Skipping 'I'

        # Draw 'M'
        self.set_pen(0, 0, 0, 3, 1)
        self.teleport(start_x + 2 * letter_spacing, start_y, rotation)
        self.set_pen(0, 0, 255, 3, 0)
        self.move(2.0, 0.0, 1.0)
        self.move(0.0, -3.0, 1.0)
        self.move(2.0, 0.0, 1.0)
        self.move(0.0, 3.0, 1.0)
        self.move(2.0, 0.0, 1.0)
        self.move(0.0, -3.0, 1.0)
        self.move(2.0, 0.0, 1.0)

        # Draw 'L'
        self.set_pen(0, 0, 0, 3, 1)
        self.teleport(start_x + 3.5 * letter_spacing, start_y, rotation)
        self.set_pen(255, 165, 0, 3, 0)
        self.move(2.0, 0.0, 1.0)
        self.move(0.0, 3.0, 1.0)

        self.get_logger().info("Finished drawing 'AIML'")

def main(args=None):
    rclpy.init(args=args)
    node = DrawLetters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
