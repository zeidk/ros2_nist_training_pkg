import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class AVServer(Node):
    '''
    Class for a server node
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        self._left_sub = self.create_subscription(
            Image, 'left', self._left_sub_cb, 10)
        self._right_sub = self.create_subscription(
            Image, 'right', self._right_sub_cb, 10)
        self._front_sub = self.create_subscription(
            Image, 'front', self._front_sub_cb, 10)
        self._rear_sub = self.create_subscription(
            Image, 'rear', self._rear_sub_cb, 10)

    def _left_sub_cb(self, msg):
        '''
        Callback function to the /left topic.

        Args:
            msg (Image): Image message from the left camera.
        '''

        self.get_logger().info(f'Receiving data from left camera: {msg.header.frame_id}')

    def _right_sub_cb(self, msg):
        '''
        Callback function to the /right topic.

        Args:
            msg (Image): Image message from the right camera.
        '''
        self.get_logger().info(f'Receiving data from right camera: {msg.header.frame_id}')

    def _front_sub_cb(self, msg):
        '''
        Callback function to the /front topic.

        Args:
            msg (Image): Image message from the left camera.
        '''
        self.get_logger().info(f'Receiving data from front camera: {msg.header.frame_id}')

    def _rear_sub_cb(self, msg):
        '''
        Callback function to the /rear topic.

        Args:
            msg (Image): Image message from the rear camera.
        '''
        self.get_logger().info(f'Receiving data from rear camera: {msg.header.frame_id}')



def main(args=None):
    '''
    Main function to create and spin the node.
    '''
    rclpy.init(args=args)
    av_server = AVServer('av_server')
    rclpy.spin(av_server)
    rclpy.shutdown()
