import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AdvancedPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'leia', 10)
        
        # publishing every 2s
        self._timer = self.create_timer(.02, self.timer_callback)
        self._msg = String()
        self._counter = 0

    def timer_callback(self):
        self._counter += 1
        self._msg.data = f'{self._counter}: Help me Obi-Wan Kenobi, you are my only hope.'
        self._publisher.publish(self._msg)
        self.get_logger().info(f'Publishing: {self._msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedPublisher('advanced_publisher')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
