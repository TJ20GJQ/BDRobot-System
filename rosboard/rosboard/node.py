#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import sys
import os
# 添加项目路径以便导入cv_bridge模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rosboard.cv_bridge import imgmsg_to_cv2
from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, LivelinessPolicy


# 尝试导入matplotlib进行图像显示
import matplotlib.pyplot as plt
can_display = True

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        # 创建订阅者，指定话题名、消息类型、回调函数和QoS配置
        qos = QoSProfile(depth=10, 
                        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, 
                        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
                        history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                        liveliness=LivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
                        avoid_ros_namespace_conventions=False)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 订阅camera话题
            self.image_callback,
            qos_profile=qos)  # QoS深度设为10
        self.subscription  # 防止未使用变量警告
        
        # 初始化图像计数器
        self.image_count = 0
        
        # 如果可以显示图像，初始化matplotlib
        if can_display:
            plt.ion()  # 开启交互模式
            self.fig, self.ax = plt.subplots()
            self.imshow = None

    def image_callback(self, msg):
        try:
            # 使用自定义的cv_bridge将ROS图像消息转换为numpy数组
            cv_image = imgmsg_to_cv2(msg, desired_encoding="bgr8", flip_channels=True)
            
            # 记录图像信息
            self.image_count += 1
            self.get_logger().info(f'收到图像 #{self.image_count}: {msg.width}x{msg.height}, 编码: {msg.encoding}')
            
            # 如果可以显示图像，更新显示
            if can_display:
                if self.imshow is None:
                    self.imshow = self.ax.imshow(cv_image)
                else:
                    self.imshow.set_data(cv_image)
                
                self.ax.set_title(f'Camera Image #{self.image_count}')
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}')


def main(args=None):
    # 初始化ROS2环境
    rclpy.init(args=args)
    # 创建节点实例
    camera_subscriber = CameraSubscriber()
    
    try:
        # 自旋以保持节点运行并处理回调
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        # 捕获Ctrl+C以优雅退出
        camera_subscriber.get_logger().info('收到退出信号，关闭节点')
    finally:
        # 清理资源
        camera_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()