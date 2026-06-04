import os
from pathlib import Path
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')),
        launch_arguments=[('use_sim_time', use_sim_time)],
    )

    # 添加静态坐标变换发布器：从body到base_link（body与IMU坐标系对齐）
    static_tf_pub = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_body_to_baselink',
        arguments=['0.028', '0.360', '-0.719', '-1.571', '0', '0', 'body', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    )

    ld.add_action(fastlio_launch)
    ld.add_action(static_tf_pub)

    return ld

