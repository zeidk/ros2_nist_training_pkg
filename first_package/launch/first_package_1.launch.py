# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node

# this function is needed
def generate_launch_description():
    # instantiate a Launchdescription object
    publisher_node = Node(                  # declare your Node
        package="first_package",            # package name
        executable="advanced_publisher_exe"  # executable as set in setup.py
    )
    subscriber_node = Node(
        package="first_package",
        executable="subscriber_exe"
    )
    
    ld = LaunchDescription()
    ld.add_action(publisher_node)   # add each Node to the LaunchDescription object
    ld.add_action(subscriber_node)  # add each Node to the LaunchDescription object
    return ld                       # return the LaunchDescription object
