# BMS 相关 ROS 包

ROS包 话题参数说明

## ROS包信息

包名: dali_bms_ros2 <br>
程序名: dali_bms_ros2_node <br>
节点名: dali_bms_ros2_node

## 话题

发布话题名称: /dali_bms/frame_info <br>
发布话题数据: dali_bms_ros2::msg::Frame <br>
发布话题说明: 原始数据

发布话题名称: /dali_bms/battery_info <br>
发布话题数据: robot_ros2_msgs::msg::DaliBms <br>
发布话题说明: 完整电池数据

发布话题名称: /dali_bms/battery_state <br>
发布话题数据: sensor_msgs::msg::BatteryState <br>
发布话题说明: 通用电池数据

## 参数

参数名: dali_bms_port <br>
数据类型: string <br>
默认值: /dev/ttyUSB0 <br>
参数说明: 通讯接口

参数名: dali_bms_baudrate <br>
数据类型: int <br>
默认值: 9600 <br>
参数说明: 通讯波特率

参数名: dali_bms_log_display <br>
数据类型: bool <br>
默认值: true <br>
参数说明: 日志信息显示开关

参数名: dali_bms_original_display <br>
数据类型: bool <br>
默认值: false <br>
参数说明: 原始数据输出开关

## 版本信息

当前: V 1.0.0 <br>

V 1.0.0 <br>
++首次创建

## 其他
