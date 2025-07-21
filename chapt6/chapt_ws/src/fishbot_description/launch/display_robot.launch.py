import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_name = 'fishbot_description'
    try:
        urdf_tutorial_path = get_package_share_directory(pkg_name)
    except Exception as e:
        raise RuntimeError(f"无法找到包 '{pkg_name}': {str(e)}")
    
    # 构建URDF文件路径
    default_model_path = os.path.join(urdf_tutorial_path, 'urdf', 'first_robot.urdf')
    
    default_rviz_config_path = os.path.join(urdf_tutorial_path, 'config', 'rviz', 'display_model.rviz')

    # 检查文件是否存在
    if not os.path.exists(default_model_path):
        raise FileNotFoundError(f"URDF文件不存在: {default_model_path}")
    
    # 声明启动参数
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='URDF的绝对路径'
    )

    # 创建机器人描述
    # 使用Command读取URDF文件
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # 创建节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config_path]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])