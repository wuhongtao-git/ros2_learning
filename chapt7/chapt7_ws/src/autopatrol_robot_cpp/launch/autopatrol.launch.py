import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('autopatrol_robot_cpp'),
        'config',
        'patrol_config.yaml'
    )
    
    patrol_node = Node(
        package='autopatrol_robot_cpp',
        executable='patrol_node',
        name='patrol_node',
        parameters=[config],
        output='screen'
    )
    
    speaker_node = Node(
        package='autopatrol_robot_cpp',
        executable='speaker_node',
        name='speaker_node',
        output='screen'
    )
    
    return LaunchDescription([
        patrol_node,
        speaker_node
    ])