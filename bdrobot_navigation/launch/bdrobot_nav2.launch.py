# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    # Get the launch directory
    my_nav_dir = get_package_share_directory('bdrobot_navigation')
    my_launch_dir = os.path.join(my_nav_dir, 'launch')
    my_param_dir = os.path.join(my_nav_dir, 'param')
    my_param_file = 'bdrobot_params.yaml'
    my_bt_file = 'navigate_w_replanning_and_round_robin_recovery.xml'

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    open_rviz = LaunchConfiguration('open_rviz')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')  # 启用行缓冲模式后，日志信息会在遇到换行符时立即输出，这样就能实时看到程序的运行日志，有助于调试和监控程序的运行状态

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(my_map_dir, my_map_file),
    #     description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(my_param_dir, my_param_file),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(my_param_dir, my_bt_file),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_open_rviz_cmd = DeclareLaunchArgument(
        'open_rviz',
        default_value='True',
        description='Launch Rviz?')
    
    # pcd2pgm launch
    start_pcd2pgm_cmd = Node(
        package="pcd2pgm",
        executable="pcd2pgm_node",
        name="pcd2pgm",
        output="screen",
        parameters=[params_file],
    )

    # # 在线地图服务器：订阅 FAST-LIO2 /Laser_map → /map OccupancyGrid
    # start_online_map_server_cmd = Node(
    #     package="bdrobot_zone_planner",
    #     executable="online_map_server",
    #     name="online_map_server",
    #     output="screen",
    #     parameters=[{
    #         'cloud_topic': '/Laser_map',
    #         'map_topic': 'map',
    #         'frame_id': 'map',
    #         'resolution': 0.05,
    #         'z_min': -0.2,
    #         'z_max': 2.0,
    #         'publish_rate': 1.0,
    #     }],
    # )

    # # 静态 TF: map → camera_init (identity)，无后端时 map 原点 = FAST-LIO2 里程计原点
    # start_map_tf_cmd = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="map_to_camera_init_tf",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"],
    # )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
 
        # IncludeLaunchDescription(
        #     # Run Localization only when we don't use
        #     PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'localization_launch.py')),
        #     launch_arguments={'namespace': namespace,
        #                       'map': map_yaml_file,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'params_file': params_file,
        #                       'use_lifecycle_mgr': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),        
    ])

    rviz_view = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(my_nav_dir, 'rviz', 'wheeltec_demo.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(open_rviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_open_rviz_cmd)
    ld.add_action(start_pcd2pgm_cmd)
    # ld.add_action(start_online_map_server_cmd)
    # ld.add_action(start_map_tf_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)
    
    ld.add_action(rviz_view)

    return ld
