import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class MultiSubscriber(Node):

    def __init__(self):
        super().__init__('multi_subscriber')

        self.chat1_val = 0
        self.chat2_val = 0
        self.chat3_val = 0

        self.create_subscription(Int32, '/chat1', self.chat1_callback, 10)
        self.create_subscription(Int32, '/chat2', self.chat2_callback, 10)
        self.create_subscription(Int32, '/chat3', self.chat3_callback, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def chat1_callback(self, msg):
        self.chat1_val = msg.data

    def chat2_callback(self, msg):
        self.chat2_val = msg.data

    def chat3_callback(self, msg):
        self.chat3_val = msg.data

    def timer_callback(self):
        total = self.chat1_val + self.chat2_val + self.chat3_val
        self.get_logger().info(f'Sum of received values: {self.chat1_val} + {self.chat2_val} + {self.chat3_val} = {total}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
