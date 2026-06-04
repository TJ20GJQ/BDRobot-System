import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # Get the launch directory
    bringup_dir = get_package_share_directory('bdrobot_gazebo')

    # 获取当前功能包（robot_arm_config）的共享目录路径，后续用于定位配置文件
    moveit_config_dir = get_package_share_directory('bdrobot_moveit_config')

    # 构建MoveIt配置：加载机器人仿真模型（URDF）和语义描述（SRDF）
    # "arm"是规划组名称，package_name指定功能包，robot_description加载URDF，robot_description_semantic加载SRDF
    moveit_config = (MoveItConfigsBuilder("arm_group", package_name="bdrobot_moveit_config")
                     .robot_description('config/bdrobot_description.gazebo.urdf.xacro')  # 加载机器人仿真模型的URDF文件（Xacro格式）
                     .robot_description_semantic('config/robot_description.srdf')     # 加载机器人语义描述（定义规划组、自碰撞等）
                     .robot_description_kinematics('config/kinematics.yaml')  # 加载机器人运动学配置（IK求解器）
                     .to_moveit_configs()  # 生成MoveIt配置对象
                     )  # MoveItConfigsBuilder参考https://blog.csdn.net/ZPC8210/article/details/144989012
    
    # 发布机器人状态（关节变换、TF等）：让RViz和MoveIt能看到机器人的模型和运动
    robot_desc_node = Node(
        package="robot_state_publisher",  # 功能包：机器人状态发布
        executable="robot_state_publisher",  # 可执行文件：状态发布节点
        name="robot_state_publisher",  # 节点名称
        output="screen",
        parameters=[
            moveit_config.robot_description,  # 机器人URDF描述
            {'use_sim_time': use_sim_time},  # 启用仿真时间（和Gazebo时钟同步）
            {"publish_frequency": 30.0},  # 状态发布频率（30Hz）
        ],
    )

    # 启动Gazebo仿真环境
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments=[
            ('world', os.path.join(bringup_dir, 'world', 'bdrobot_ceil_demo.world')),
        ]
    )

    # 在Gazebo中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'myrobot',
                   '-x', '-9', '-y', '0', '-z', '0.114', '-R', '0.0', '-P', '0.0', '-Y', '-1.57'],
        output='screen'
    )

    # 启动RViz并加载MoveIt预设可视化配置：可视化规划过程、机器人模型、环境等
    rviz_node = Node(
        package="rviz2",  # 功能包：RViz可视化工具
        executable="rviz2",  # 可执行文件：RViz启动程序
        output="screen",  # 输出日志到文件（减少终端冗余）
        # 参数：加载预设的MoveIt可视化配置文件
        arguments=["-d", moveit_config_dir + '/config/moveit.rviz'],
        parameters=[
            moveit_config.robot_description,  # 机器人URDF描述
            moveit_config.robot_description_semantic,  # 机器人语义描述（SRDF）
            moveit_config.robot_description_kinematics,  # 运动学配置（逆解、正解）
            moveit_config.planning_pipelines,  # 规划管线配置（如OMPL算法）
            moveit_config.joint_limits,  # 关节限制配置（最大角度、速度等）
            {'use_sim_time': use_sim_time},  # 启用仿真时间
        ],
        condition=IfCondition(rviz),
    )

    # 启动ros2_control控制器管理器：管理所有关节控制器的启动、停止
    ros2_control_node = Node(
        package='controller_manager',  # 功能包：控制器管理器
        executable='ros2_control_node',  # 可执行文件：ros2_control核心节点
        parameters=[
            os.path.join(moveit_config_dir, 'config', 'ros2_controllers.yaml'),  # 加载控制器配置文件（定义关节控制器参数）
            {'use_sim_time': use_sim_time},  # 启用仿真时间
        ],
        output="screen",  # 输出日志到终端和文件
    )
    # 启动具体的控制器：关节状态发布器、机械臂控制器、夹爪控制器
    controller_spawner_node = Node(
        package="controller_manager",  # 功能包：控制器管理器
        executable="spawner",  # 可执行文件：控制器启动工具
        # 参数：需要启动的控制器名称（必须和ros2_controllers.yaml中定义的一致）
        arguments=[
            # "joint_state_broadcaster",   # 仿真不需要，由gazebo插件发布/joint_state话题
            "arm_group_controller"
        ],
    )

    # 启动MoveIt核心规划节点（move_group）：负责运动规划、逆解、轨迹生成等核心功能
    move_group_node = Node(
        package="moveit_ros_move_group",  # 功能包：MoveIt运动规划核心
        executable="move_group",  # 可执行文件：move_group节点
        output="screen",  # 输出日志到终端
        parameters=[
            moveit_config.to_dict(),  # 加载所有MoveIt配置
            {'use_sim_time': use_sim_time},  # 启用仿真时间
        ],
        # arguments=["--ros-args", "--log-level", "DEBUG"]
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    )
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch RViz if true')
    )
    ld.add_action(gazebo)
    ld.add_action(robot_desc_node)
    ld.add_action(spawn_entity)
    ld.add_action(ros2_control_node)
    ld.add_action(controller_spawner_node)
    ld.add_action(move_group_node)
    ld.add_action(rviz_node)

    return ld

