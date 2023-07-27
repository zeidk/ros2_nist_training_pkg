from datetime import datetime
import random
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nist_msgs.msg import WeatherStation
from nist_msgs.srv import AddTwoInts


class Weather(Node):
    '''
    Class to publish weather information
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
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


class ServiceClientCallFromTimer(Node):
    '''
    Class to call the add_two_ints service in the callback function of a timer
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        # This attribute is set in the subscriber callback function
        # and is used in the timer callback function to determine whether to call the service or not
        self._second_is_even = False

        # Create various callback groups to ensure that various events are called in a mutually exclusive manner
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        # Create a subscriber to the weather topic
        # The callback group ensures that the callback function is called in a mutually exclusive manner
        self._subscriber = self.create_subscription(
            WeatherStation, 'weather', self._weather_cb, 100, callback_group=subscriber_cb_group)

        # Create a timer to call the service every second
        # In the callback function, we will call the service based on the value of the attribute self._second
        self._timer = self.create_timer(
            1, self._timer_cb, callback_group=timer_cb_group)

        # Create a client to call the add_two_ints service
        # This client will be called synchronously
        # The callback group ensures that the callback function is called in a mutually exclusive manner
        self._sync_client = self.create_client(
            AddTwoInts, 'add_two_ints', callback_group=client_cb_group)

        # Create a client to call the add_two_ints service
        # This client will be called asynchronously
        # A callback group is not needed
        self._async_client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self._logger.info("client created")
        self._request = AddTwoInts.Request()

    def _weather_cb(self, msg: WeatherStation):
        '''
        Callback function for the weather subscriber

        Args:
            msg (WeatherStation): Message received from the weather topic
        '''
        self.get_logger().info(f'Second: {int(msg.time.sec)}')

        # The following code will call the service only if the seconds in the time stamp is even
        if int(msg.time.sec) % 2 == 0:
            self._second_is_even = True
        else:
            self._second_is_even = False

    def _timer_cb(self):
        '''
        Callback function for the timer
        '''

        self._request.a = random.randint(0, 9)
        self._request.b = random.randint(0, 9)

        # Async call if the seconds in the time stamp is even
        if self._second_is_even:
            self.send_async_request(self._request.a, self._request.b)
        else:
            self.send_sync_request(self._request.a, self._request.b)

    def send_async_request(self, a, b):
        '''
        Send a request asynchronously to the add_two_ints service

        Args:
            a (int): First integer
            b (int): Second integer
        '''

        while not self._async_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self._request.a = a
        self._request.b = b

        self.get_logger().info(f'TimerAsyncRequest: {a} + {b}')
        future = self._async_client.call_async(self._request)

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        '''
        Callback function for the future object

        Args:
            future (Future): A future object
        '''
        self.get_logger().info(f'TimerAsyncResult: {future.result().sum}')

    def send_sync_request(self, a, b):
        '''
        Send a request synchronously to the add_two_ints service

        Args:
            a (int): First integer
            b (int): Second integer
        '''
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info(f'TimerSyncRequest: {a} + {b}')
        self._request.a = a
        self._request.b = b
        response = self._sync_client.call(self._request)
        self.get_logger().info(f'TimerSyncResult: {response.sum}')


class ServiceClientCallFromSubscriber(Node):
    '''
    Class to call the add_two_ints service in the callback function of a subscriber
    '''

    def __init__(self, node_name):
        super().__init__(node_name)

        # Create various callback groups to ensure that various events are called in a mutually exclusive manner
        client_cb_group = MutuallyExclusiveCallbackGroup()
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()

        self._subscriber = self.create_subscription(
            WeatherStation, 'weather', self._weather_cb, 100, callback_group=subscriber_cb_group)

        # Create a client to call the add_two_ints service
        # This client will be synchronously called
        # The callback group ensures that the callback function is called in a mutually exclusive manner
        self._sync_client = self.create_client(
            AddTwoInts, 'add_two_ints', callback_group=client_cb_group)

        self._async_client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self._logger.info("client created")
        self._request = AddTwoInts.Request()

    def _weather_cb(self, msg: WeatherStation):
        '''
        Callback function for the weather subscriber

        Args:
            msg (WeatherStation): Message received from the weather topic
        '''
        self.get_logger().info(f'Second: {int(msg.time.sec)}')

        self._request.a = random.randint(0, 9)
        self._request.b = random.randint(0, 9)
        # The following code will call the service only if the seconds in the time stamp is even
        if int(msg.time.sec) % 2 == 0:
            # self.send_sync_request(self._request.a, self._request.b)
            self.send_async_request(self._request.a, self._request.b)
        else:
            self.send_sync_request(self._request.a, self._request.b)

        # To test ReentrantCallbackGroup, uncomment the following code
        # while True:
        #     ...

    def send_async_request(self, a, b):
        '''
        Send a request asynchronously to the add_two_ints service

        Args:
            a (int): First integer
            b (int): Second integer
        '''

        while not self._async_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self._request.a = a
        self._request.b = b

        self.get_logger().info(f'SubAsyncRequest: {a} + {b}')
        future = self._async_client.call_async(self._request)

        # Add a callback function to be called when the future is complete
        future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        '''
        Callback function for the future object

        Args:
            future (Future): A future object
        '''
        self.get_logger().info(f'SubAsyncResult: {future.result().sum}')

    def send_sync_request(self, a, b):
        '''
        Send a request synchronously to the add_two_ints service

        Args:
            a (int): First integer
            b (int): Second integer
        '''
        while not self._sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info(f'SubSyncRequest: {a} + {b}')
        self._request.a = a
        self._request.b = b
        response = self._sync_client.call(self._request)
        self.get_logger().info(f'SubSyncResult: {response.sum}')


class AddTwoIntsService(Node):
    """
    Class to create a service to add two integers
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_two_ints_cb)
        
        self.get_logger().info('Service created, waiting for requests...')

    def add_two_ints_cb(self, request, response):
        """
        Callback function for the add_two_ints service
        """
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request\na: {request.a} b: {request.b}')

        return response


def main(args=None):
    """
    Main function to create the nodes and spin the nodes
    """
    rclpy.init(args=args)

    weather_node = Weather('weather_forecast')
    timer_node = ServiceClientCallFromTimer('client_timer_py')
    sub_node = ServiceClientCallFromSubscriber('client_sub_py')

    executor = MultiThreadedExecutor()
    executor.add_node(weather_node)
    executor.add_node(timer_node)
    executor.add_node(sub_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        timer_node.get_logger().info('KeyboardInterrupt, shutting down.\n')

    weather_node.destroy_node()
    timer_node.destroy_node()
    sub_node.destroy_node()
    rclpy.shutdown()


def server_main(args=None):
    """
    Main function to start the server
    """
    rclpy.init(args=args)

    server_node = AddTwoIntsService('minimal_service')
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        server_node.get_logger().info('KeyboardInterrupt, shutting down.\n')

    server_node.destroy_node()
    rclpy.shutdown()
