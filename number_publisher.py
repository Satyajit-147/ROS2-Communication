import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.publisher_ = self.create_publisher(Int32, 'number', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_number)
        self.i = 1

    def publish_number(self):
        msg = Int32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

