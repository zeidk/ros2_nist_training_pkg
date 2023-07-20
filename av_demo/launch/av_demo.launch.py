import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Loading parameters from a file
    param_file = os.path.join(
        get_package_share_directory('av_demo'),
        'config',
        'params.yaml'
    )

    camera1_node = Node(
        package="av_demo",
        executable="camera_exe",
        parameters=[param_file],   # parameter file
        name='camera1',             # node remapping
        remappings=[                # topic remapping
            ('/camera', '/left')
        ]
    )

    camera2_node = Node(
        package="av_demo",
        executable="camera_exe",
        parameters=[param_file],
        name='camera2',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/front')
        ]
    )

    camera3_node = Node(
        package="av_demo",
        executable="camera_exe",
        parameters=[param_file],
        name='camera3',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/right')
        ]
    )

    camera4_node = Node(
        package="av_demo",
        executable="camera_exe",
        parameters=[param_file],
        name='camera4',  # node remapping
        remappings=[    # topic remapping
            ('/camera', '/rear')
        ]
    )
    
    server_node = Node(
        package="av_demo",
        executable="server_exe",
    )

    launch_description = LaunchDescription()
    
    launch_description.add_action(server_node)
    launch_description.add_action(camera1_node)
    launch_description.add_action(camera2_node)
    launch_description.add_action(camera3_node)
    launch_description.add_action(camera4_node)
    return launch_description
