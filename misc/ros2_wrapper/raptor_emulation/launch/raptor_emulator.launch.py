import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{time}] [{name}] [{severity}] {message}'
    
    dbc_file_path = os.path.join(
        get_package_share_directory('tum_raptor_emulation'),
        'params',
        'CAN1-INDY-V6.dbc'
    )
    
    # Executor
    raptor_node_executor = Node(
        package='tum_raptor_emulation',
        namespace='raptor_emulation',
        executable='tum_raptor_emulation',
        output='screen',
        parameters=[
            {'dbc_file_path' : dbc_file_path},
            {'vehicle_id' : 1},
            {'steering_ratio' : 19.5},
            {'rear_wheel_radius' : 0.3118},
            {'front_wheel_radius' : 0.301},
        ],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # Add noded to LaunchDescription
    ld.add_action(raptor_node_executor)

    return ld
