import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the launch directory
    bringup_dir = get_package_share_directory('bdrobot_gazebo')

    # choose your car, the default car is mini_4wd_moveit_four
    choose_car = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'robot_model_description.launch.py')),
            launch_arguments=[('use_sim_time', use_sim_time), 
                              ('urdf_file', os.path.join(get_package_share_directory('bdrobot_description'), 'urdf', 'robot_description.urdf'))],
    )

    # 启动Gazebo仿真环境
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments=[
            ('world', os.path.join(bringup_dir, 'world', 'livox_demo0.world')),
        ]
    )

    # 在Gazebo中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'myrobot',
                   '-x', '-9', '-y', '0', '-z', '0.4', '-R', '0.0', '-P', '0.0', '-Y', '-1.57'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    )
    ld.add_action(gazebo)
    ld.add_action(choose_car)
    ld.add_action(spawn_entity)

    return ld

