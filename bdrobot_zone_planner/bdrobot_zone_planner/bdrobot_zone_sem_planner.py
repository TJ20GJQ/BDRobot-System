#!/home/yuyouling/anaconda3/envs/humble/bin/python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import random
import threading
import time
import numpy as np
from cv_bridge import CvBridge
import cv2
from bdrobot_zone_planner.utils import *
from bdrobot_zone_planner.semantic_map import SemanticMap, visualize_components

GLOBAL_FRAME = "map"
SLAM_MODE = "OFFLINE"

class ZonePlannerNode(Node):

    def __init__(self):
        super().__init__('zone_planner_node')

        # 点云地图持久订阅
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            '/cloud_pcd',
            self.cloud_callback,
            1
        )

        # 里程计订阅，用于确定墙面法向量朝向
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )

        # 发布转换后的里程计: map → base_link
        self._bdrobot_odom_pub = self.create_publisher(
            Odometry, 
            '/bdrobot_odom', 
            10
        )

        # 导航目标位姿发布者
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            QoSProfile(durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=5)
        )

        # 创建发布者，发布set_rebound_goals话题
        self.rebound_goals_publisher = self.create_publisher(
            PoseArray,
            'set_rebound_goals',
            10
        )

        # 调试用 marker 发布者
        self.marker_publisher = self.create_publisher(
            Marker,
            'point_marker',
            10
        )

        # 语义地图可视化发布者
        self.semantic_map_viz_pub = self.create_publisher(
            MarkerArray,
            'semantic_map_markers',
            10
        )

        # TF 用于查询 map → base_link 变换
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 图像/点云数据
        self.image_subscription = None
        self.cv_bridge = CvBridge()
        self.current_image = None
        self._latest_cloud_msg = None  # 仅缓存原始消息，按需转换

        # 语义地图
        self._semantic_map = SemanticMap(
            voxel_size=0.1,
            ransac_dist_thresh=0.05,
            ransac_min_inliers=100,
            confirm_total_points=500,
            confirm_density=20.0,
            run_ransac_every_n_frames=5,
        )

        # ======全局状态======
        self.planner_state = "Ready"  # 状态机：ready->locating->executing
        self._robot_pose: Pose = None  # 当前机器人base_link位置
        self._state_thread = threading.Thread(target=self._state_machine, daemon=True)  # 状态机子线程循环，主线程留给 rclpy.spin() 处理所有订阅回调

        # ======第一部分：图像定位待测区域，得到可行区域======
        self.camera_estimate_point = None

        # ======第二部分：根据可行区域点云地图，修正测点位置，规划AGV导航点======
        self.global_wall_point: Pose = None
        self.local_wall_point: Pose = None

        self._state_thread.start()
        self.get_logger().info('Zone Planner Node started...')
        self.get_logger().info('Press Ctrl+C to exit')

    def cloud_callback(self, msg):
        """点云地图回调：缓存原始消息，并喂入语义地图"""
        self._latest_cloud_msg = msg

        # 将点云变换到全局坐标系后喂入语义地图
        try:
            tf = self._tf_buffer.lookup_transform(GLOBAL_FRAME, msg.header.frame_id, rclpy.time.Time())
            cloud = pointcloud2_to_xyz(msg)
            if len(cloud) > 0:
                # 将点云变换到全局坐标系
                cloud_global = transform_points(cloud, transform_stamped_to_pose(tf))
                self._semantic_map.add_frame(cloud_global)

                # 发布语义地图可视化
                self._publish_semantic_map_viz()

        except Exception as e:
            self.get_logger().debug(f'Semantic map update: {e}')
        
        if SLAM_MODE == "OFFLINE":
            self.cloud_subscription.destroy()

    def odom_callback(self, msg):
        """里程计回调：缓存 body 位置，并发布 map→base_link 里程计"""
        try:
            tf = self._tf_buffer.lookup_transform(GLOBAL_FRAME, 'base_link', rclpy.time.Time())
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = GLOBAL_FRAME
            odom.child_frame_id = 'base_link'
            odom.pose.pose = transform_stamped_to_pose(tf)
            self._robot_pose = odom.pose.pose
            self._bdrobot_odom_pub.publish(odom)
        except Exception as e:
            self.get_logger().debug(f'TF lookup({GLOBAL_FRAME}->base_link): {e}')

    def _state_machine(self):
        """子线程循环驱动状态机，主线程 rclpy.spin() 处理订阅回调"""
        while rclpy.ok():
            if self.planner_state == "Ready":
                self.find_zone()
            elif self.planner_state == "Locating":
                self.locate_zone()
            elif self.planner_state == "Executing":
                self.execute_zone()
            time.sleep(0.1)

    def find_zone(self):
        """查找区域，设置粗略三维坐标"""
        # TODO: 从相机或其他传感器获取粗略三维坐标
        # 此处使用固定值作为示例
        self.camera_estimate_point = np.array([1, 1.4, 0.7])
        self._publish_marker(self.camera_estimate_point, marker_name='find_rough_point', color='red')

        input("Now find a rough point by camera. Press Enter to continue...")
        self.planner_state = "Locating"
        self.get_logger().info(f'Locating zone with rough point: {self.camera_estimate_point}')

    def _publish_marker(self, point, marker_name='', color='red'):
        """发布 point 的 Marker 到 RViz 调试"""
        marker = Marker()
        marker.header.frame_id = GLOBAL_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = marker_name
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        if color == 'red':
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        elif color == 'green':
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
        elif color == 'blue':
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
        elif color == 'yellow':
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
        elif color == 'orange':
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
        else:
            marker.color = color
        self.marker_publisher.publish(marker)

    def _publish_semantic_map_viz(self):
        """发布语义地图的可视化 MarkerArray"""
        components = self._semantic_map.components
        if not components:
            return
        marker_array = visualize_components(components, frame_id=GLOBAL_FRAME)
        now = self.get_clock().now().to_msg()
        for marker in marker_array.markers:
            marker.header.stamp = now
        self.semantic_map_viz_pub.publish(marker_array)

    def _locate_wall_point(self, cloud, point, search_radius=0.2, wall_threshold=20) -> tuple[np.ndarray, np.ndarray]:
        # TODO：转换为增量式语义地图，就可以直接计算距离最近的墙面了
        """在点云中定位粗略点，返回对应墙面上投影点"""
        dists = np.linalg.norm(cloud - point, axis=1)  # 计算点到所有点的距离
        nearby_idx = np.where(dists < search_radius)[0]
        if len(nearby_idx) < wall_threshold:
            self.get_logger().warn(
                f'Not enough nearby points ({len(nearby_idx)}), '
                f'cannot correct to wall surface'
            )
            return None, None
        
        nearby_points = cloud[nearby_idx]

        # PCA 平面拟合：最小特征值对应的特征向量即为平面法向量
        centroid = np.mean(nearby_points, axis=0)
        centered = nearby_points - centroid
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        normal = eigenvectors[:, 0]  # 最小特征值对应法向量（方向任意）

        # 将点正交投影到墙面上
        vec = point - centroid
        dist_to_plane = np.dot(vec, normal)
        projected_point = point - dist_to_plane * normal
        
        self._publish_marker(projected_point, marker_name='locate_wall_point', color='orange')
        return projected_point, normal

    def locate_zone(self):
        """定位区域：输入粗略三维坐标，通过语义地图将坐标矫正到墙面上"""
        if self._latest_cloud_msg is None:
            self.get_logger().info('No point cloud update yet, waiting...')
            return

        # 1. 查询语义地图中最近已确认墙面，将粗略点投影到墙面上
        result = self._semantic_map.query_nearest_wall(self.camera_estimate_point)
        if result is None:
            self.get_logger().warn(
                'No confirmed wall in semantic map yet. '
                f'({len(self._semantic_map.components)} components, '
                f'{len(self._semantic_map.get_confirmed_walls())} walls confirmed)'
            )
            return

        wall_point, wall_normal = result

        self.get_logger().info(
            f'Corrected: {self.camera_estimate_point} -> {wall_point}, wall normal: {wall_normal}'
        )

        # 2. 计算底盘目标位置
        if self._robot_pose is not None:  # 用机器人位置确定法向量朝向：法向量应指向机器人（房间内侧）
            robot_to_wall = wall_point - np.array([self._robot_pose.position.x, self._robot_pose.position.y, self._robot_pose.position.z])
            if np.dot(wall_normal, robot_to_wall) > 0:
                wall_normal = -wall_normal

        # 计算底盘目标方向
        wall_dir = np.array([0, 0, 1])  # 墙面方向（向上）
        chassis_goal_dir = np.cross(wall_normal, wall_dir)  # 底盘目标方向 = 法向量 × 墙方向
        chassis_goal_dir /= np.linalg.norm(chassis_goal_dir)

        # 计算底盘目标位置
        chassis_goal_pos = wall_point + wall_normal * 0.65 + chassis_goal_dir * 0.35  # 目标位置：墙面点 + 沿法向向里60cm + 向右10cm
        chassis_goal_pos[2] = 0.0 
        chassis_goal_R = np.column_stack([chassis_goal_dir, wall_normal, wall_dir])  # 旋转矩阵 [wall_dir, goal_dir, normal] → 四元数，小车平行于墙面

        self._publish_goal_pose(ndarray_to_pose(chassis_goal_pos, chassis_goal_R))
        self._publish_marker(chassis_goal_pos, marker_name='chassis_goal_point', color='green')

        # 计算墙面点的全局位姿
        wall_point_R = np.column_stack([-wall_normal, chassis_goal_dir, wall_dir])
        self.global_wall_point = ndarray_to_pose(wall_point, wall_point_R)

        input('Locate zone finished. Press Enter to continue Executing...')
        self.planner_state = "Executing"
        self.get_logger().info('Zone located, executing...')

    def _publish_goal_pose(self, pose: Pose):
        """发布导航目标位姿到 set_goal"""
        msg = PoseStamped()
        msg.header.frame_id = GLOBAL_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose
        self.goal_publisher.publish(msg)
    
    def execute_zone(self):
        """执行区域"""
        self.publish_test_points()
        self.planner_state = "Ready"
        self.get_logger().info('Finding zone...')

    def publish_test_points(self):
        """发布模拟的测试点"""
        pose_array_msg = PoseArray()
        pose_array_msg.header = Header()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = 'base_link'  # 或其他合适的坐标系
        
        # 生成一些模拟的测试点
        num_points = 1  # 每次发布5个测试点
        
        # for i in range(num_points):
        #     # x = random.uniform(-0.05, -0)  # 随机X坐标
        #     x = 0.2
        #     y = random.uniform(-0.4, -0.3)  # 随机Y坐标
        #     z = random.uniform(0, 0.5)  # 随机高度
            
        #     # 创建姿态
        #     pose = Pose()
        #     pose.position.x = x
        #     pose.position.y = y
        #     pose.position.z = z

        #     # 设置一个简单的方向（指向-Z方向，即向下）30度
        #     pose.orientation.x = 0.0000
        #     pose.orientation.y = 0.0000
        #     pose.orientation.z = 0.7071
        #     pose.orientation.w = 0.7071
            
        #     pose_array_msg.poses.append(pose)

        self.local_wall_point = transform_pose(self.global_wall_point, inverse_pose(self._robot_pose))
        pose_array_msg.poses.append(self.local_wall_point)
        print(self.local_wall_point)
        # 发布消息
        self.rebound_goals_publisher.publish(pose_array_msg)

        input("Execute finished. Press Enter to continue...")
        self.get_logger().info(f'Published {len(pose_array_msg.poses)} test points to set_rebound_goals')


def main(args=None):
    rclpy.init(args=args)
    node = ZonePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()