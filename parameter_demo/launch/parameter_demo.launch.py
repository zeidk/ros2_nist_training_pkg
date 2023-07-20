# pull in some Python launch modules.
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from ament_index_python.packages import get_package_share_directory

# ---- VERSION 1 ----#
def generate_launch_description():
    '''
    Function to generate a LaunchDescription object.

    Returns:
        LaunchDescription: A LaunchDescription object.
    '''
    parameter_demo_node = Node(
        package="parameter_demo",
        executable="parameter_demo_exe",
        parameters=[
            {'bandwidth': '3'},
        ]
    )

    # instantiate a Launchdescription object
    launch_description = LaunchDescription()
    launch_description.add_action(parameter_demo_node)
    return launch_description


# ---- VERSION 2 ----#
# def generate_launch_description():
#     '''
#     Function to generate a LaunchDescription object.

#     Returns:
#         LaunchDescription: A LaunchDescription object.
#     '''

#     bandwidth_arg = DeclareLaunchArgument("bandwidth", default_value='4')
#     print('='*30 + 'bandwidth argument:', bandwidth_arg)

#     parameter_demo_node = Node(
#         package="parameter_demo",
#         executable="parameter_demo_exe",
#         parameters=[
#             {'bandwidth': bandwidth_arg},
#         ]
#     )

#     # instantiate a Launchdescription object
#     launch_description = LaunchDescription()
#     launch_description.add_action(parameter_demo_node)
#     return launch_description


# ---- VERSION 3 ----#
# def launch_setup(context, *args, **kwargs):
#     '''
#     Returns a list of nodes to launch.

#     Args:
#         context (): The context object for the launch.

#     Returns:
#         List[Node]: A list of nodes to launch.
#     '''

#     # initialize arguments passed to the launch file
#     bandwidth_arg = LaunchConfiguration('bandwidth').perform(context)
#     print('='*30 + 'bandwidth argument:', bandwidth_arg)
#     # bandwidth_param = {"bandwidth": bandwidth_arg}

#     parameter_demo_node = Node(
#         package="parameter_demo",
#         executable="parameter_demo_exe",
#         parameters=[
#             {"bandwidth": bandwidth_arg}
#         ]
#     )

#     # all the nodes to launch
#     nodes_to_start = [parameter_demo_node]

#     return nodes_to_start


# def generate_launch_description():
#     '''
#     Function to generate a LaunchDescription object.

#     Returns:
#         LaunchDescription: A LaunchDescription object.
#     '''

#     bandwidth_arg = DeclareLaunchArgument(
#         "bandwidth",
#         default_value='4')

#     declared_arguments = []
#     declared_arguments.append(bandwidth_arg)

#     launch_description = LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

#     return launch_description


# ---- VERSION 4 ----#
# def generate_launch_description():
#     '''
#     Function to generate a LaunchDescription object.

#     Returns:
#         LaunchDescription: A LaunchDescription object.
#     '''
    
#     # get path to params.yaml
#     param_file = os.path.join(
#         get_package_share_directory('parameter_demo'),
#         'config',
#         'params.yaml'
#     )
#     print('='*30 + 'param file path:', param_file)
        
#     parameter_demo_node = Node(
#         package="parameter_demo",
#         executable="parameter_demo_exe",
#         parameters=[param_file]
#     )

#     # instantiate a Launchdescription object
#     launch_description = LaunchDescription()
#     launch_description.add_action(parameter_demo_node)
#     return launch_description
