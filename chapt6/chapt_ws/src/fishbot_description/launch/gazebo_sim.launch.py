import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    robot_name_in_model = "fishbot"
    
    # 获取包路径
    fishbot_desc_pkg = get_package_share_directory('fishbot_description')
    
    # 修正路径拼接
    default_model_path = os.path.join(
        fishbot_desc_pkg, 
        'urdf', 'fishbot', 'fishbot.urdf.xacro'
    )
    default_world_path = os.path.join(fishbot_desc_pkg, 'world', 'custom_room.world')

    # 打印路径
    print(f"URDF 路径: {default_model_path}")
    print(f"Gazebo 世界路径: {default_world_path}")

    # 声明启动参数
    declare_model_arg = launch.actions.DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='URDF/XACRO 文件的绝对路径'
    )

    # 关键修复：按照工作正常代码的模式添加空格
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command([
            'xacro ',  # 注意这里有一个空格！
            launch.substitutions.LaunchConfiguration('model')
        ]),
        value_type=str
    )

    # 启动 robot_state_publisher
    robot_state_pub_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    # 启动 Gazebo
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 
        'launch', 'gazebo.launch.py'
    )
    
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments={
            'world': default_world_path,
            'verbose': 'true'
        }.items()
    )

    # 启动 spawn_entity
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name_in_model,
            '-z', '0.1'
        ]
    )

    return launch.LaunchDescription([
        declare_model_arg,
        robot_state_pub_node,
        launch_gazebo,
        spawn_entity_node
    ])