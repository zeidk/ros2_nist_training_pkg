'''
Minimal Python node.
'''

import rclpy
from rclpy.node import Node
from first_package.simple_interface import SimpleNode

def main(args=None):
    '''
    Main function for the node.
    '''
    rclpy.init(args=args)
    node = SimpleNode('minimal_python_node')
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()
