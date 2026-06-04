import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


# def load_file(package_name, file_path):
#     package_path = get_package_share_directory(package_name)
#     absolute_file_path = os.path.join(package_path, file_path)

#     try:
#         with open(absolute_file_path, "r") as file:
#             return file.read()
#     except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
#         return None
    

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    
    # 获取功能包目录
    bringup_dir = get_package_share_directory('bdrobot_gazebo')
    
    # 包含现有的mygazebo_moveit.launch.py文件
    mygazebo_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'mygazebo_moveit.launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('rviz', rviz),  # 传递rviz参数
        ]
    )

    # 构建MoveIt配置：加载机器人仿真模型（URDF）和语义描述（SRDF）
    moveit_config = (MoveItConfigsBuilder("arm_group", package_name="bdrobot_moveit_config")
                     .robot_description('config/bdrobot_description.gazebo.urdf.xacro')  # 加载机器人仿真模型的URDF文件（Xacro格式）
                     .robot_description_semantic('config/robot_description.srdf')     # 加载机器人语义描述（定义规划组、自碰撞等）
                     .robot_description_kinematics('config/kinematics.yaml')  # 加载机器人运动学配置（IK求解器）
                     .to_moveit_configs()  # 生成MoveIt配置对象
                     )
    
    # 添加笛卡尔路径规划demo节点
    cartesian_demo_node = TimerAction(
        period=5.0,  # 5秒延时
        actions=[
            Node(
                package="bdrobot_arm_controller",
                executable="move_cartesian_demo",
                name="arm_cartesian",
                output="screen",
                parameters=[  # 节点参数
                    {'use_sim_time': use_sim_time},  # 启用仿真时间
                    moveit_config.to_dict(),  # 加载所有MoveIt配置
                    # {'robot_description_semantic': load_file('bdrobot_moveit_config', 'config/robot_description.srdf')},
                    # {'robot_description_kinematics': load_file('bdrobot_moveit_config', 'config/kinematics.yaml')}
                ],
                # arguments=["--ros-args", "--log-level", "DEBUG"]
            )
        ]
    )
    
    # 构建LaunchDescription
    ld = LaunchDescription()
    
    # 声明参数
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='False',  # 默认不启动RViz
        description='Start RViz if true'
    ))
    
    # 添加包含的launch文件和新节点
    ld.add_action(mygazebo_moveit_launch)
    ld.add_action(cartesian_demo_node)
    return ld