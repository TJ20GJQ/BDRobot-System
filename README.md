系统加速库示例
cd ~/samples/notebooks/
./start_notebook.sh

启动mid360驱动
sudo ifconfig eth0 192.168.1.50
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

fastlio建图
ros2 launch fast_lio mapping.launch.py

保存并查看地图
ros2 service call /map_save std_srvs/srv/Trigger
pcl_viewer test.pcd

fastlio重定位
ros2 launch fast_lio_localization localization.launch.py map:=/home/HwHiAiUser/ros_ws/test.pcd

启动可视化面板
ros2 run rosboard rosboard_node

启动底盘数据交互
ros2 launch ht_can_ros2 ht_can_ros2.launch.py
测试
ros2 topic pub /ht/ftfd_ctrl geometry_msgs/msg/Twist "{linear: {x: 0.0,y: 0.0,z: 0.0},angular: {x: 0.0,y: 0.0,z: 0.0}}" -1
ros2 topic pub /ht/auto_charge_ctrl std_msgs/msg/UInt8 "data: 1" -1

检查orbbec深度相机连接并启动
ros2 run orbbec_camera list_devices_node
ros2 launch orbbec_camera gemini_330_series.launch.py
=================TOOL==================
查看TF树
ros2 run rqt_tf_tree rqt_tf_tree

ATC
atc --model=best.onnx --framework=5 --output=model --soc_version=Ascend310B1 --input_shape="images:1,3,640,640"

