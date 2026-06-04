import os
from pathlib import Path
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import launch_ros.actions
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

    # 启动Gazebo仿真环境 livox_demo0/livox_demo_obs
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments=[
            ('world', os.path.join(bringup_dir, 'world', 'livox_demo_obs.world')),
        ]
    )

    # 在Gazebo中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bdrobot',
                   '-x', '-10', '-y', '-5', '-z', '0.114', '-R', '0.0', '-P', '0.0', '-Y', '0'],
        output='screen'
    )

    package_path = get_package_share_directory("fast_lio_localization")
    default_config_path = os.path.join(package_path, "config")
    default_rviz_config_path = os.path.join(package_path, "rviz", "fastlio_localization.rviz")

    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    pcd_map_path = LaunchConfiguration("map")

    # 启动Fastlio里程计重定位，发布odom->base_link
    fast_lio_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        parameters=[PathJoinSubstitution([config_path, config_file]), {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # 添加静态坐标变换发布器：从body到baselink
    static_tf_pub = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_body_to_baselink',
        arguments=['0.028', '0.360', '-0.719', '-1.571', '0', '0', 'body', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 启动全局定位节点，发布map->odom
    global_localization_node = Node(
        package="fast_lio_localization",
        executable="global_localization.py",
        name="global_localization",
        output="screen",
        parameters=[{"map_voxel_size": 0.4,
                     "scan_voxel_size": 0.1,
                     "freq_localization": 0.5,
                     "freq_global_map": 0.25,
                     "localization_threshold": 0.8,
                     "fov": 6.28319,
                     "fov_far": 100,
                     "pcd_map_path": pcd_map_path,
                     "pcd_map_topic": pcd_map_topic}],
    )

    # 融合odom->base_link和map->odom，发布map->base_link
    transform_fusion_node = Node(
        package="fast_lio_localization",
        executable="transform_fusion.py",
        name="transform_fusion",
        output="screen",
    )

    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="map_publisher",
        output="screen",
        parameters=[{"file_name": pcd_map_path,
                     "tf_frame": "map",
                    "cloud_topic": pcd_map_topic}],
                    # "period_ms_": 500}],
        remappings=[
            ("cloud_pcd", pcd_map_topic),
        ]
    )

    rviz_node = Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], condition=IfCondition(rviz_use))

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    )
    ld.add_action(DeclareLaunchArgument(
        "config_path", default_value=default_config_path, description="Yaml config file path"
    ))
    ld.add_action(DeclareLaunchArgument(
        "config_file", default_value="mid360.yaml", description="Yaml config file name"
    ))
    ld.add_action(DeclareLaunchArgument(
        "rviz", default_value="True", description="Use rviz to visualize the map"
    ))
    ld.add_action(DeclareLaunchArgument(
        "rviz_cfg", default_value=default_rviz_config_path, description="Rviz config file path"
    ))
    ld.add_action(DeclareLaunchArgument(
        "pcd_map_topic", default_value="/cloud_pcd", description="Topic name of PCD map"
    ))
    ld.add_action(DeclareLaunchArgument(
        "map", default_value="/home/yuyouling/bdrobot_ws/test_1.pcd", description="Path to the map file"
    ))

    ld.add_action(gazebo)
    ld.add_action(choose_car)
    ld.add_action(spawn_entity)
    ld.add_action(fast_lio_node)
    ld.add_action(static_tf_pub)
    ld.add_action(global_localization_node)
    ld.add_action(transform_fusion_node)
    ld.add_action(pcd_publisher_node)
    ld.add_action(rviz_node)

    return ld

