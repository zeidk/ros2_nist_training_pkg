import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String


class CallbackGroupDemo(Node):
    """
    Class to demonstrate the use of callback groups
    """

    def __init__(self, node_name):
        super().__init__(node_name)  # type: ignore
        # self._cb_group1 = MutuallyExclusiveCallbackGroup()
        # self._cb_group2 = MutuallyExclusiveCallbackGroup()

        self._number = 0

        self._timer = self.create_timer(
            1, self._timer_cb)

        self._pub = self.create_publisher(
            String, 'callbackgroup_demo', 100)

        self._sub1 = self.create_subscription(
            String, 'callbackgroup_demo', self._sub1_cb, 100)

        self._logger.info('Callback group demo node created')

        self._counter = 0

    def _timer_cb(self):
        msg = String()
        msg.data = 'Timer callback: ' + str(self._counter)
        self._pub.publish(msg)
        self._logger.info("Publishing")
        self._counter += 1

    def _sub1_cb(self, msg):
        self._logger.info(f'Sub1: {msg.data}')
        self._logger.info('Sleeping for 5 seconds')
        time.sleep(5)


def main(args=None):
    """
    Main function to start the callback group demo
    """
    rclpy.init(args=args)

    callback_group_demo_node = CallbackGroupDemo('callback_group_demo')
    try:
        rclpy.spin(callback_group_demo_node)
    except KeyboardInterrupt:
        callback_group_demo_node.get_logger().info(
            'KeyboardInterrupt, shutting down.\n')

    callback_group_demo_node.destroy_node()
    rclpy.shutdown()

# def main(args=None):
#     """
#     Main function to start the callback group demo
#     """
#     rclpy.init(args=args)

#     callback_group_demo_node = CallbackGroupDemo('callback_group_demo')
#     executor = MultiThreadedExecutor()
#     executor.add_node(callback_group_demo_node)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         callback_group_demo_node.get_logger().info(
#             'KeyboardInterrupt, shutting down.\n')

#     callback_group_demo_node.destroy_node()
#     rclpy.shutdown()
