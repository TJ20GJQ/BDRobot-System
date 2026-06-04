import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None
    

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

    robot_description_semantic = {
        'robot_description_semantic': load_file('bdrobot_moveit_config', 'config/robot_description.srdf')
    }
    
    # 添加FK demo节点
    fk_demo_node = TimerAction(
        period=5.0,  # 5秒延时
        actions=[
            Node(
                package="bdrobot_arm_controller",
                executable="fk_demo",
                name="arm_fk",
                parameters=[  # 节点参数
                    {'use_sim_time': use_sim_time},  # 启用仿真时间
                    robot_description_semantic  # 机器人模型参数
                ]
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
    ld.add_action(fk_demo_node)
    
    return ld