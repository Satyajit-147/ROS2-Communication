import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class SquareSubscriber(Node):
    def __init__(self):
        super().__init__('square_subscriber')
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Int32,
            'number',
            self.callback,
            qos_profile
        )

    def callback(self, msg):
        squared = msg.data ** 2
        self.get_logger().info(f'Received: {msg.data} | Square: {squared}')

def main(args=None):
    rclpy.init(args=args)
    node = SquareSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

