import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ========================
    # 共享部分（Gazebo仿真）
    # ========================
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    use_namespace = LaunchConfiguration('use_namespace', default='True')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true'
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value=use_namespace,
        description='Whether to apply a namespace to the navigation stack'
    )

    # 获取Gazebo包路径
    gazebo_dir = get_package_share_directory('bdrobot_gazebo')
    
    # 构建MoveIt配置对象
    moveit_config = (MoveItConfigsBuilder("arm_group", package_name="bdrobot_moveit_config")
                     .robot_description('config/bdrobot_description.gazebo.urdf.xacro')  # 加载机器人仿真模型的URDF文件（Xacro格式）
                     .robot_description_semantic('config/robot_description.srdf')     # 加载机器人语义描述（定义规划组、自碰撞等）
                     .robot_description_kinematics('config/kinematics.yaml')  # 加载机器人运动学配置（IK求解器）
                     .to_moveit_configs()  # 生成MoveIt配置对象
                     )  # MoveItConfigsBuilder参考https://blog.csdn.net/ZPC8210/article/details/144989012
    
    gazebo_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace='bdrobot_common'),

        # Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
            ),
            launch_arguments=[
                ('world', os.path.join(gazebo_dir, 'world', 'livox_demo_obs.world')),
            ]
        ),

        # 发布机器人状态（关节变换、TF等）：让RViz和MoveIt能看到机器人的模型和运动
        Node(
            package="robot_state_publisher",  # 功能包：机器人状态发布
            executable="robot_state_publisher",  # 可执行文件：状态发布节点
            name="robot_state_publisher",  # 节点名称
            output="screen",
            parameters=[
                moveit_config.robot_description,  # 机器人URDF描述
                {'use_sim_time': use_sim_time},  # 启用仿真时间（和Gazebo时钟同步）
                {"publish_frequency": 30.0},  # 状态发布频率（30Hz）
            ],
        ),

        # Spawn entity in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'bdrobot',
                    '-x', '-10', '-y', '-5', '-z', '0.114', '-R', '0.0', '-P', '0.0', '-Y', '0'],
            output='screen'
        )
    ])


    # ========================
    # FAST-LIO 配置部分
    # ========================
    # FAST-LIO 配置参数
    fastlio_localization_package_path = get_package_share_directory("fast_lio_localization")
    fastlio_localization_default_config_path = os.path.join(fastlio_localization_package_path, "config")
    fastlio_localization_default_rviz_config_path = os.path.join(fastlio_localization_package_path, "rviz", "fastlio_localization.rviz")
    fastlio_localization_config_path = LaunchConfiguration("fastlio_localization_config_path")
    fastlio_localization_config_file = LaunchConfiguration("fastlio_localization_config_file")
    fastlio_localization_rviz_use = LaunchConfiguration("fastlio_localization_rviz")
    pcd_map_topic = LaunchConfiguration("pcd_map_topic")
    pcd_map_path = LaunchConfiguration("map")
    
    # FAST-LIO 参数声明
    declare_fastlio_localization_config_path = DeclareLaunchArgument(
        "fastlio_localization_config_path", 
        default_value=fastlio_localization_default_config_path, 
        description="Yaml config file path"
    )
    declare_fastlio_localization_config_file = DeclareLaunchArgument(
        "fastlio_localization_config_file", 
        default_value="mid360.yaml", 
        description="Yaml config file name"
    )
    declare_fastlio_localization_rviz_use = DeclareLaunchArgument(
        "fastlio_localization_rviz", 
        default_value="True", 
        description="Use rviz to visualize the map"
    )
    declare_pcd_map_topic = DeclareLaunchArgument(
        "pcd_map_topic", 
        default_value="/cloud_pcd", 
        description="Topic name of PCD map"
    )
    declare_pcd_map_path = DeclareLaunchArgument(
        "map", 
        default_value="/home/yuyouling/bdrobot_ws/test_1.pcd", 
        description="Path to the map file"
    )
     
    # FAST-LIO 节点
    fastlio_localization_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace='bdrobot_fastlio_localization'),

        # 启动Fastlio里程计重定位，发布odom->base_link
        Node(
            package="fast_lio_localization",
            executable="fastlio_mapping",
            parameters=[PathJoinSubstitution([fastlio_localization_config_path, fastlio_localization_config_file]), 
                        {"use_sim_time": use_sim_time}],
            output="screen",
        ),

        # 添加静态坐标变换发布器：从body到baselink
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_body_to_baselink',
            arguments=['0.028', '0.360', '-0.719', '-1.571', '0', '0', 'body', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # 启动全局定位节点，发布map->odom
        Node(
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
        ),

        # 融合odom->base_link和map->odom，发布map->base_link
        Node(
            package="fast_lio_localization",
            executable="transform_fusion.py",
            name="transform_fusion",
            output="screen",
        ),

        # 发布PCD地图点云
        Node(
            package="pcl_ros",
            executable="pcd_to_pointcloud",
            name="map_publisher",
            output="screen",
            parameters=[{"file_name": pcd_map_path,
                        "tf_frame": "map",
                        "cloud_topic": pcd_map_topic}],
            remappings=[
                ("cloud_pcd", pcd_map_topic),
            ]
        ),

        # FAST-LIO RViz节点
        Node(
            package="rviz2", 
            executable="rviz2", 
            name='rviz',
            arguments=["-d", fastlio_localization_default_rviz_config_path], 
            condition=IfCondition(fastlio_localization_rviz_use)
        )
    ])
    

    # ========================
    # MoveIt 配置部分
    # ========================
    # MoveIt 参数声明
    moveit_rviz_use = LaunchConfiguration('moveit_rviz_use')
    
    declare_moveit_rviz_use = DeclareLaunchArgument(
        'moveit_rviz_use',
        default_value='False',
        description='Launch MoveIt RViz if true'
    )

    # 获取包路径
    moveit_config_dir = get_package_share_directory('bdrobot_moveit_config')
    moveit_default_rviz_config_path = os.path.join(moveit_config_dir, "config", "moveit.rviz")

    # MoveIt 节点
    moveit_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace='bdrobot_moveit'),
            
        # 启动ros2_control控制器管理器：管理所有关节控制器的启动、停止
        Node(
            package='controller_manager',  # 功能包：控制器管理器
            executable='ros2_control_node',  # 可执行文件：ros2_control核心节点
            parameters=[
                os.path.join(moveit_config_dir, 'config', 'ros2_controllers.yaml'),  # 加载控制器配置文件（定义关节控制器参数）
                {'use_sim_time': use_sim_time},  # 启用仿真时间
            ],
            output="screen",  # 输出日志到终端和文件
        ),
    
        # 启动具体的控制器：关节状态发布器、机械臂控制器、夹爪控制器
        Node(
            package="controller_manager",  # 功能包：控制器管理器
            executable="spawner",  # 可执行文件：控制器启动工具
            # 参数：需要启动的控制器名称（必须和ros2_controllers.yaml中定义的一致）
            arguments=[
                # "joint_state_broadcaster",   # 仿真不需要，由gazebo插件发布/joint_state话题
                "arm_group_controller"
            ],
        ),

        # 启动MoveIt核心规划节点（move_group）：负责运动规划、逆解、轨迹生成等核心功能
        Node(
            package="moveit_ros_move_group",  # 功能包：MoveIt运动规划核心
            executable="move_group",  # 可执行文件：move_group节点
            output="screen",  # 输出日志到终端
            parameters=[
                moveit_config.to_dict(),  # 加载所有MoveIt配置
                {'use_sim_time': use_sim_time},  # 启用仿真时间
            ],
            # arguments=["--ros-args", "--log-level", "DEBUG"]
        ),

        # 启动RViz并加载MoveIt预设可视化配置：可视化规划过程、机器人模型、环境等
        Node(
            package="rviz2",  # 功能包：RViz可视化工具
            executable="rviz2",  # 可执行文件：RViz启动程序
            name='rviz',
            output="screen",  # 输出日志到文件（减少终端冗余）
            # 参数：加载预设的MoveIt可视化配置文件
            arguments=["-d", moveit_default_rviz_config_path],
            parameters=[
                # moveit_config.robot_description,  # 机器人URDF描述
                # moveit_config.robot_description_semantic,  # 机器人语义描述（SRDF）
                # moveit_config.robot_description_kinematics,  # 运动学配置（逆解、正解）
                # moveit_config.planning_pipelines,  # 规划管线配置（如OMPL算法）
                # moveit_config.joint_limits,  # 关节限制配置（最大角度、速度等）
                moveit_config.to_dict(),  # 加载所有MoveIt配置
                {'use_sim_time': use_sim_time},  # 启用仿真时间
            ],
            condition=IfCondition(moveit_rviz_use),
        )
    ])


    # ========================
    # Nav2 配置部分
    # ========================
    # Nav2 参数声明
    nav2_rviz_use = LaunchConfiguration('nav2_rviz_use', default='True')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_default_bt_xml_filename = LaunchConfiguration('nav2_default_bt_xml_filename')
    
    # 获取Nav2包路径
    nav2_dir = get_package_share_directory('bdrobot_navigation')
        
    # Nav2 参数声明
    declare_nav2_rviz_use = DeclareLaunchArgument(
        'nav2_rviz_use',
        default_value=nav2_rviz_use,
        description='Launch Nav2 RViz if true'
    )

    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(nav2_dir, 'param', 'bdrobot_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_nav2_default_bt_xml_cmd = DeclareLaunchArgument(
        'nav2_default_bt_xml_filename',
        default_value=os.path.join(nav2_dir, 'param', 'navigate_w_replanning_and_round_robin_recovery.xml'),
        description='Full path to the behavior tree xml file to use')
    
    # Navigation launch
    nav2_bringup_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace='nav2',
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                            'params_file': nav2_params_file,
                            'default_bt_xml_filename': nav2_default_bt_xml_filename,
                            'map_subscribe_transient_local': 'true'}.items()
        ),

        # Launch RViz for navigation
        Node(
            package='rviz2',
            executable='rviz2',
            name='nav2_rviz',
            output='screen',
            arguments=['-d', os.path.join(nav2_dir, 'rviz', 'wheeltec_demo.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(nav2_rviz_use)
        )
    ])


    # ========================
    # 创建Launch Description
    # ========================
    ld = LaunchDescription()
    
    # FAST-LIO 参数声明
    ld.add_action(declare_fastlio_localization_config_path)
    ld.add_action(declare_fastlio_localization_config_file)
    ld.add_action(declare_fastlio_localization_rviz_use)
    ld.add_action(declare_pcd_map_topic)
    ld.add_action(declare_pcd_map_path)
    
    # MoveIt 参数声明
    ld.add_action(declare_moveit_rviz_use)
    
    # Nav2 参数声明
    ld.add_action(declare_nav2_rviz_use)
    ld.add_action(declare_nav2_params_file_cmd)
    ld.add_action(declare_nav2_default_bt_xml_cmd)

    # Add all launch actions in order: Gazebo -> FAST-LIO -> MoveIt -> Nav2
    # Gazebo simulation
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(gazebo_group)
    
    # FAST-LIO
    ld.add_action(fastlio_localization_group)
    
    # MoveIt
    ld.add_action(moveit_group)
    
    # Nav2
    ld.add_action(nav2_bringup_group)

    return ld