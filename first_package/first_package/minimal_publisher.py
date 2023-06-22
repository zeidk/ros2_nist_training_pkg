import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class MinimalPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(Int64, 'leia', 10)
        self._msg = Int64()
        self._msg.data = 43
        self._publisher.publish(self._msg)
        self.get_logger().info(f'Publishing: {self._msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher('minimal_publisher')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
