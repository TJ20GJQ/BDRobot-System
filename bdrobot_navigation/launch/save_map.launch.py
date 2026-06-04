# ros2 run nav2_map_server map_saver_cli -f ~/map
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', os.path.join(get_package_share_directory('bdrobot_navigation'), 'map', 'WHEELTEC')],
        
        parameters=[{'save_map_timeout': 10000},
                    {'free_thresh_default':0.196}]

        )
    ld = LaunchDescription()

    ld.add_action(map_saver)

    return ld
 
