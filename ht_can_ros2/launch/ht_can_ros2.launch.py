from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ht_can_ros2',
            executable='ht_can_ros2_node',
            name='ht_can_ros2_node',
            output='screen',
            parameters=[{
                'ht_odom_enable': True,
                'ht_log_display': True,
                'ht_original_display': False,
                'ht_skip_find': True,
            }],
            remappings=[
                # pub: old, new
                ('/ht/frame_info',          '/ht/frame_info'),
                ('/ht/auto_charge_info',    '/ht/auto_charge_info'),
                ('/ht/battery_info',        '/ht/battery_info'),
                ('/ht/state_info',          '/ht/state_info'),
                ('/ht/remote_control_info', '/ht/remote_control_info'),
                ('/ht/encoder_count_info',  '/ht/encoder_count_info'),
                ('/ht/motion_info',         '/ht/motion_info'),
                ('/ht/speed_info',          '/ht/speed_info'),
                ('/ht/angle_info',          '/ht/angle_info'),

                # sub: old, new
                ('/ht/velocity_ctrl',    '/ht/velocity_ctrl'),
                ('/ht/motion_ctrl',      '/ht/motion_ctrl'),
                ('/ht/lamp_ctrl',        '/ht/lamp_ctrl'),
                ('/ht/auto_charge_ctrl', '/ht/auto_charge_ctrl'),
            ]
        )
    ])
