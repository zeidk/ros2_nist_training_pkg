
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleNode(Node):
    '''
    Minimal Python node using OOP style.

    Args:
        Node (rclpy.Node): Node class from rclpy package.
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        # self.get_logger().debug('Help me Obi-Wan Kenobi, you are my only hope')
        self.get_logger().info('Help me Obi-Wan Kenobi, you are my only hope')
        # self.get_logger().warn('Help me Obi-Wan Kenobi, you are my only hope')
        # self.get_logger().error('Help me Obi-Wan Kenobi, you are my only hope')
        # self.get_logger().fatal('Help me Obi-Wan Kenobi, you are my only hope')


class AdvancedPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'leia', 10)
        # publishing every 2s
        self._timer = self.create_timer(2, self.timer_callback)
        self._msg = String()
        self._counter = 0

    def timer_callback(self):
        self._counter += 1
        self._msg.data = f'{self._counter}: Help me Obi-Wan Kenobi, you are my only hope.'
        self._publisher.publish(self._msg)
        self.get_logger().info(f'Publishing: {self._msg.data}')


class MinimalPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'leia', 10)
        self._msg = String()
        self._msg.data = 'Help me Obi-Wan Kenobi, you are my only hope.'
        self._publisher.publish(self._msg)
        self.get_logger().info(f'Publishing: {self._msg.data}')


class SubscriberNode(Node):

    '''SubscriberNode This class is a node that subscribes to a string message and publishes the first word of the message.

    Arguments:
        Node -- Node class from rclpy
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        self._subscriber = self.create_subscription(String, 'leia', self.subscriber_callback, 10)

    def subscriber_callback(self, msg):
        '''subscriber_callback Callback function for the subscriber to the chatter663 topic.

        Arguments:
            msg -- String message from the chatter663 topic.
        '''

        self.get_logger().info(f'Receiving: {msg.data}')
