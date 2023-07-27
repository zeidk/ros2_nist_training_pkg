from datetime import datetime
from rclpy.node import Node
from builtin_interfaces.msg import Time
import rclpy
from nist_msgs.msg import WeatherStation

class WeatherDemo(Node):
    '''
    Class to publish weather information
    '''

    def __init__(self, node_name):
        super().__init__(node_name) # type: ignore
        self._forecast_pub = self.create_publisher(
            WeatherStation, 'weather', 100)
        self._pub_timer = self.create_timer(1, self._pub_timer_cb)
        self._current_day = datetime.now().weekday()

    def _pub_timer_cb(self):
        msg = WeatherStation()
        msg.weather = WeatherStation.SNOWY
        msg.day = self._current_day
        time_msg = Time()
        time_msg.sec = self.get_clock().now().seconds_nanoseconds()[0]
        time_msg.nanosec = self.get_clock().now().seconds_nanoseconds()[1]
        msg.time = time_msg

        self.get_logger().info(f'Publishing: {msg}')
        self._forecast_pub.publish(msg)



def main(args=None):
    """
    Main function to start the callback group demo
    """
    rclpy.init(args=args)

    weather_node = WeatherDemo('weather_demo')
    try:
        rclpy.spin(weather_node)
    except KeyboardInterrupt:
        weather_node.get_logger().info(
            'KeyboardInterrupt, shutting down.\n')

    weather_node.destroy_node()
    rclpy.shutdown()





