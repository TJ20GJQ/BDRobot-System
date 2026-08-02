# HT系列底盘 相关 ROS 包

ROS包 话题参数说明

## ROS包信息

包名: ht_can_ros2 <br>
程序名: ht_can_ros2_node <br>
节点名: ht_can_ros2_node

## 话题

发布话题名称: /ht/frame_info <br>
发布话题数据: ht_can_ros2::msg::Frame <br>
发布话题说明: 原始数据

发布话题名称: /ht/drive_steer_front_left_error_info <br>
发布话题数据: hstd_msgs::msg::UInt16 <br>
发布话题说明: 前左转向电机驱动器错误  [0: 无错误] [其他: 有错误]

发布话题名称: /ht/drive_steer_front_right_error_info <br>
发布话题数据: hstd_msgs::msg::UInt16 <br>
发布话题说明: 前右转向电机驱动器错误  [0: 无错误] [其他: 有错误]

发布话题名称: /ht/drive_steer_rear_left_error_info <br>
发布话题数据: hstd_msgs::msg::UInt16 <br>
发布话题说明: 后左转向电机驱动器错误  [0: 无错误] [其他: 有错误]

发布话题名称: /ht/drive_steer_rear_right_error_info <br>
发布话题数据: hstd_msgs::msg::UInt16 <br>
发布话题说明: 后右转向电机驱动器错误  [0: 无错误] [其他: 有错误]

发布话题名称: /ht/drive_front_left_error_info <br>
发布话题数据: ht_can_ros2::msg::DriveSdfzError <br>
发布话题说明: 前左动力电机驱动器错误

发布话题名称: /ht/drive_front_right_error_info <br>
发布话题数据: ht_can_ros2::msg::DriveSdfzError <br>
发布话题说明: 前右动力电机驱动器错误

发布话题名称: /ht/drive_rear_left_error_info <br>
发布话题数据: ht_can_ros2::msg::DriveSdfzError <br>
发布话题说明: 后左动力电机驱动器错误

发布话题名称: /ht/drive_rear_right_error_info <br>
发布话题数据: ht_can_ros2::msg::DriveSdfzError <br>
发布话题说明: 后右动力电机驱动器错误

发布话题名称: /ht/auto_charge_info <br>
发布话题数据: robot_ros2_msgs::msg::AutoCharge <br>
发布话题说明: 回充状态信息

发布话题名称: /ht/battery_info <br>
发布话题数据: robot_ros2_msgs::msg::BatteryState <br>
发布话题说明: 主板采集的电池信息

发布话题名称: /ht/state_info <br>
发布话题数据: robot_ros2_msgs::msg::ChassisState <br>
发布话题说明: 底盘状态

发布话题名称: /ht/remote_control_info <br>
发布话题数据: robot_ros2_msgs::msg::RemoteControl <br>
发布话题说明: 遥控数据

发布话题名称: /ht/angle_info <br>
发布话题数据: robot_ros2_msgs::msg::FourWheelSteerAngle <br>
发布话题说明: 底盘转向电机角度

发布话题名称: /ht/current_info <br>
发布话题数据: robot_ros2_msgs::msg::FourWheelSteerCurrent <br>
发布话题说明: 底盘电机电流

发布话题名称: /ht/encoder_count_info <br>
发布话题数据: robot_ros2_msgs::msg::FourWheelSteerEncoder <br>
发布话题说明: 底盘驱动电机编码器计数

发布话题名称: /ht/motion_info <br>
发布话题数据: robot_ros2_msgs::msg::FourWheelSteerMotion <br>
发布话题说明: 底盘运动状态

发布话题名称: /ht/speed_info <br>
发布话题数据: robot_ros2_msgs::msg::FourWheelSteerSpeed <br>
发布话题说明: 底盘驱动电机速度

发布话题名称: /ht/odom_info <br>
发布话题数据: nav_msgs::msg::Odometry <br>
发布话题说明: 底盘里程信息  (里程计算仅FTFD模式和阿克曼的原地旋转有效)

---

订阅话题名称: /ht/velocity_ctrl <br>
订阅话题数据: geometry_msgs::msg::Twist <br>
订阅话题说明: 线速度 角速度 角度 控制

订阅话题名称: /ht/ftfd_ctrl <br>
订阅话题数据: geometry_msgs::msg::Twist <br>
订阅话题说明: 线速度 角速度 控制

订阅话题名称: /ht/ackermann_ctrl <br>
订阅话题数据: geometry_msgs::msg::Twist <br>
订阅话题说明: 速度 角度 控制

订阅话题名称: /ht/rotate_ctrl <br>
订阅话题数据: geometry_msgs::msg::Twist <br>
订阅话题说明: 原地旋转 控制

订阅话题名称: /ht/motion_ctrl <br>
订阅话题数据: robot_ros2_msgs::msg::FourWheelSteerMotion <br>
订阅话题说明: 底盘运动控制

订阅话题名称: /ht/auto_charge_ctrl <br>
订阅话题数据: std_msgs::UInt8 <br>
订阅话题说明: 回充开关, [0: 关闭回充] [1: 开启红外回充] [2: 开启激光回充]

订阅话题名称: /ht/collision_clean <br>
订阅话题数据: std_msgs::msg::Empty <br>
订阅话题说明: 碰撞状态清除

订阅话题名称: /ht/lamp_ctrl <br>
订阅话题数据: std_msgs::msg::Bool <br>
订阅话题说明: 照明灯控制, [true: 打开] [false: 关闭]

订阅话题名称: /ht/odom_clean <br>
订阅话题数据: std_msgs::msg::Empty <br>
订阅话题说明: 里程计清零

## 参数

参数名: ht_odom_enable <br>
数据类型: bool <br>
默认值: true <br>
参数说明: 里程计开关

参数名: ht_log_display <br>
数据类型: bool <br>
默认值: true <br>
参数说明: 日志信息显示开关

参数名: ht_original_display <br>
数据类型: bool <br>
默认值: false <br>
参数说明: 原始数据输出开关

## 版本信息

当前: V 1.0.2 <br>

V 1.0.2 <br>
++launch文件增加跳过寻找can分析仪选项，默认开启。 <br>
++log信息改为使用ros官方log输出。

V 1.0.1 <br>
++增加双转阿克曼的控制和数据反馈 <br>
++转向电机错误类型从bool转为uint16

V 1.0.0 <br>
++首次创建

## 其他
