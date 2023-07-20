'''
Demo on the use of parameters in ROS2.
'''

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class ParameterDemoInterface(Node):
    '''
    Class to demonstrate the use of parameters in ROS2.

    Args:
        Node (Node): Superclass Node.
    '''

    def __init__(self, node_name):
        super().__init__(node_name)
        # self.add_on_set_parameters_callback(self.parameters_cb)
        # # 1. Declare a parameter
        # self.declare_parameter('bandwidth', '4')
        # # 2. Store the parameter value
        # self._bandwidth = self.get_parameter('bandwidth').get_parameter_value().string_value
        

        # # Create a timer to showcase the use of parameters
        # self._timer = self.create_timer(1, self.timer_cb)
        # self._counter = 0
        self.get_logger().info(f'{node_name} initialized')

    def timer_cb(self):
        '''
        Timer callback function.
        '''
        # 3. Use the parameter value
        self.get_logger().info(f'Bandwidth: {self._bandwidth}')
        # if self._counter == 5:
        #     self.set_parameters([Parameter('bandwidth', Parameter.Type.STRING, '25')])  # set the parameter value
        # self._counter += 1

    def parameters_cb(self, params):
        '''
        Callback function for when parameters are set.

        Returns:
            bool: True if the parameters were set successfully.
        '''
        success = False
        for param in params:
            if param.name == "bandwidth":
                # validate the parameter value before setting it
                if param.type_ == Parameter.Type.STRING:
                    success = True
                    self._bandwidth = param.value  # modify the attribute
            else:
                raise ValueError(f'unknown parameter {param.name}')
        return SetParametersResult(successful=success)


def main(args=None):
    '''
    Main function.
    '''
    rclpy.init(args=args)
    demo_node = ParameterDemoInterface('parameter_demo')
    rclpy.spin(demo_node)
    demo_node.destroy_node()
    rclpy.shutdown()
