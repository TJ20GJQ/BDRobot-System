import os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
    
def generate_robot_node(robot_urdf_file, use_sim_time='False'):
    robot_description = Command([FindExecutable(name="xacro"), " ", robot_urdf_file])
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': ParameterValue(robot_description, value_type=str)}],
    )

def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{child}',
        arguments=[translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], parent, child],
    )
    
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file = LaunchConfiguration('urdf_file')
    robot_state_publisher = GroupAction([
        generate_robot_node(urdf_file, use_sim_time),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true')
        )
    ld.add_action(DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(get_package_share_directory('bdrobot_description'), 'urdf', 'mini_4wd_moveit_four.urdf'),
            description='Path to the URDF file')
        )
    ld.add_action(robot_state_publisher)
    return ld

