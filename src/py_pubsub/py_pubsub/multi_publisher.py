import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random


class MultiPublisher(Node):

    def __init__(self):
        super().__init__('multi_publisher')

        self.publisher1 = self.create_publisher(Int32, '/chat1', 10)
        self.publisher2 = self.create_publisher(Int32, '/chat2', 10)
        self.publisher3 = self.create_publisher(Int32, '/chat3', 10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg1 = Int32()
        msg2 = Int32()
        msg3 = Int32()

        msg1.data = random.randint(0, 100)
        msg2.data = random.randint(0, 100)
        msg3.data = random.randint(0, 100)

        self.publisher1.publish(msg1)
        self.publisher2.publish(msg2)
        self.publisher3.publish(msg3)

        self.get_logger().info(f'Published: /chat1={msg1.data}, /chat2={msg2.data}, /chat3={msg3.data}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
