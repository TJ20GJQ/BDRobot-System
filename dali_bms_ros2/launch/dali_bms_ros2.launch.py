from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dali_bms_ros2',
            executable='dali_bms_ros2_node',
            name='dali_bms_ros2_node',
            output='screen',
            parameters=[{
                'dali_bms_port': '/dev/ttyUSB0',
                'dali_bms_baudrate': 9600,
                'dali_bms_log_display': True,
                'dali_bms_original_display': False,
            }],
            remappings=[
                # pub: old, new
                ('/dali_bms/frame_info',    '/dali_bms/frame_info'),
                ('/dali_bms/battery_info',  '/dali_bms/battery_info'),
                ('/dali_bms/battery_state', '/dali_bms/battery_state'),

                # sub: old, new
            ]
        )
    ])