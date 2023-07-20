import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class AVCamera(Node):
    '''
    Class for a camera node

    Args:
        Node (Node): Class for creating a ROS node
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        # Publisher
        self._publisher = self.create_publisher(Image, 'camera', 100)
        # Parameters
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('bandwidth', 2.5)
        self.declare_parameter('name', 'camera')

        self._camera_frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self._camera_bandwidth = self.get_parameter('bandwidth').get_parameter_value().double_value
        self._camera_name = self.get_parameter('name').get_parameter_value().string_value

        self.get_logger().info(f'{node_name}: [name: {self._camera_name}]')
        self.get_logger().info(f'{node_name}: [frequency: {self._camera_frequency}]')
        self.get_logger().info(f'{node_name}: [bandwidth: {self._camera_bandwidth}]')

        self.get_logger().info(f'{node_name} initialized')
        self.get_logger().info(f'{node_name} is publishing...')
        self._timer = self.create_timer(int(1/self._camera_frequency), self.timer_cb)

    def timer_cb(self):
        '''
        Build and publish an Image message.
        '''
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._camera_name
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 640 * 3
        msg.data = [0] * msg.height * msg.step
        self._publisher.publish(msg)


def main(args=None):
    '''
    Main function to create and spin the node.
    '''
    rclpy.init(args=args)
    av_camera = AVCamera('camera')
    rclpy.spin(av_camera)
    rclpy.shutdown()
