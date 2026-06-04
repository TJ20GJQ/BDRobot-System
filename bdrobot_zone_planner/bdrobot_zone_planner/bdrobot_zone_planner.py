#!/home/yuyouling/anaconda3/envs/humble/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from nav2_msgs.action import NavigateToPose
from bdrobot_arm_controller.action import ExecuteGoals
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from typing import Optional
import threading
import time
import math
import numpy as np
from cv_bridge import CvBridge
import cv2
from bdrobot_zone_planner.utils import *

GLOBAL_FRAME = "map"
MAX_RETRIES_PER_GOAL = 10  # 一个目标点最大重试次数
MOVE_PRECISION = 0.02  # 底盘移动精度2cm
REBOUND_POINT_SEPARATION = 0.05  # 5cm 间距
SIDE_REBOUND = False  # 是否侧向/正向回弹

class ZonePlannerNode(Node):

    def __init__(self):
        super().__init__('zone_planner_node')

        # 点云地图持久订阅
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            '/cloud_pcd',  # /Laser_map
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

        # 调试用 marker 发布者
        self.marker_publisher = self.create_publisher(
            Marker,
            'point_marker',
            10
        )

        # Nav2 导航 action 客户端，可获取到达目标的反馈
        self._nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_done_event = threading.Event()
        self._nav_success = False

        # 机械臂执行 action 客户端
        self._execute_goals_client = ActionClient(self, ExecuteGoals, 'execute_goals')
        self._execute_done_event = threading.Event()
        self._execute_result = None

        # TF 用于查询 map → base_link 变换
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 图像/点云数据
        self.image_subscription = None
        self.cv_bridge = CvBridge()
        self.current_image = None
        self._latest_cloud_msg = None  # 仅缓存原始消息，按需转换

        # ======全局状态======
        self.planner_state = "Ready"  # 状态机：ready->locating->executing
        self._robot_pose: Pose = None  # 当前机器人base_link位置
        self._robot_pose_update: bool = False  # 机器人位置是否更新
        self._state_thread = threading.Thread(target=self._state_machine, daemon=True)  # 状态机子线程循环，主线程留给 rclpy.spin() 处理所有订阅回调

        # ======第一部分：图像定位待测区域，得到可行区域======
        self.camera_estimate_point = None

        # ======第二部分：根据可行区域点云地图，修正测点位置，规划AGV导航点======
        self.global_wall_point: Pose = None

        # ======第三部分：根据导航点，微调底盘并执行机械臂运动======
        self._last_feedback_status: str = None
        self._pending_deltas: list = None

        self._state_thread.start()
        self.get_logger().info('Zone Planner Node started...')
        self.get_logger().info('Press Ctrl+C to exit')

    def cloud_callback(self, msg):
        """点云地图回调，仅缓存原始消息，不做转换"""
        self._latest_cloud_msg = msg

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
            self._robot_pose_update = True
            self._bdrobot_odom_pub.publish(odom)
        except Exception as e:
            self.get_logger().debug(f'TF lookup({GLOBAL_FRAME}->base_link): {e}')

    def _wait_for_robot_pose_update(self, frequency=10):
        """等待机器人最新位置更新"""
        self._robot_pose_update = False
        while not self._robot_pose_update:
            time.sleep(1/frequency)

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
        # TODO：YOLO分割区域
        # TODO: 从相机或其他传感器获取粗略三维坐标
        # 此处使用固定值作为示例
        self.camera_estimate_point = np.array([1, 1.4, 0.7])
        self._publish_marker(self.camera_estimate_point, marker_name='find_rough_point', color='red')

        input("Now find a rough point by camera. Press Enter to continue...")
        self.planner_state = "Locating"
        self.get_logger().info(f'Locating zone with rough point: {self.camera_estimate_point}')

    def _publish_marker(self, point: np.ndarray | Point, marker_name='', id=0, color='red'):
        """发布 point 的 Marker 到 RViz 调试"""
        marker = Marker()
        marker.header.frame_id = GLOBAL_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = marker_name
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        if type(point) == np.ndarray:
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
        elif type(point) == Point:
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.z
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
        """定位区域：输入粗略三维坐标，通过点云地图将坐标矫正到墙面上"""
        if self._latest_cloud_msg is None:
            self.get_logger().info('No point cloud update yet, waiting...')
            return
        
        # TODO：转换为增量式语义地图，RANSAC平面拟合墙面，累积到再次添加新点依然是内点就不再更新，这样就不需要FASTLIO发布map了
        cloud = pointcloud2_to_xyz(self._latest_cloud_msg)
        self.get_logger().info(f'Point cloud shape: {cloud.shape}')

        # 1. 定位墙面点
        wall_point, wall_normal = self._locate_wall_point(cloud, self.camera_estimate_point)
        if wall_point is None:
            return

        self.get_logger().info(
            f'Corrected: {self.camera_estimate_point} -> {wall_point}, wall normal: {wall_normal}'
        )

        # 2. 计算底盘目标位置
        if self._robot_pose is not None:  # 用机器人位置确定法向量朝向：法向量应指向机器人（房间内侧）
            robot_to_wall = wall_point - np.array([self._robot_pose.position.x, self._robot_pose.position.y, self._robot_pose.position.z])
            if np.dot(wall_normal, robot_to_wall) > 0:
                wall_normal = -wall_normal

        if SIDE_REBOUND:  # 侧向回弹
            # 计算底盘目标方向
            wall_dir = np.array([0, 0, 1])  # 墙面方向（向上）
            chassis_goal_dir = np.cross(wall_normal, wall_dir)  # 底盘目标方向 = 法向量 × 墙方向
            chassis_goal_dir /= np.linalg.norm(chassis_goal_dir)

            # 计算底盘目标位置
            chassis_goal_pos = wall_point + wall_normal * 0.65 + chassis_goal_dir * 0.35  # 目标位置：墙面点 + 沿法向向里60cm + 向右10cm
            chassis_goal_pos[2] = 0.0 
            chassis_goal_R = np.column_stack([chassis_goal_dir, wall_normal, wall_dir])  # 旋转矩阵 [wall_dir, goal_dir, normal] → 四元数，小车平行于墙面

            # 计算墙面点的全局位姿
            wall_point_R = np.column_stack([-wall_normal, chassis_goal_dir, wall_dir])
            self.global_wall_point = ndarray_to_pose(wall_point, wall_point_R)
        else:  # 正向回弹
            # 计算底盘目标方向
            wall_dir = np.array([0, 0, 1])  # 墙面方向（向上）
            chassis_goal_dir = wall_normal
            chassis_goal_dir /= np.linalg.norm(chassis_goal_dir)
            wall_align_dir = np.cross(wall_dir, chassis_goal_dir)

            # 计算底盘目标位置
            chassis_goal_pos = wall_point + chassis_goal_dir * 0.77 + wall_align_dir * 0.4  # 目标位置：墙面点 + 沿法向向里60cm + 向右10cm
            chassis_goal_pos[2] = 0.0 
            chassis_goal_R = np.column_stack([chassis_goal_dir, wall_align_dir, wall_dir])  # 旋转矩阵 [wall_dir, goal_dir, normal] → 四元数，小车平行于墙面

            # 计算墙面点的全局位姿，姿态朝向墙面外侧
            wall_point_R = np.column_stack([-wall_normal, -wall_align_dir, wall_dir])
            self.global_wall_point = ndarray_to_pose(wall_point, wall_point_R)

        self._publish_marker(chassis_goal_pos, marker_name='chassis_goal_point', color='green')
        if not self._navigate_goal_pose(ndarray_to_pose(chassis_goal_pos, chassis_goal_R), wait=True):
            self.get_logger().error('Failed to navigate to chassis goal')
            self.planner_state = "Ready"
            return
        
        input('Locate zone finished and chassis goal reached. Press Enter to continue Executing...')
        self.planner_state = "Executing"
        self.get_logger().info('Zone located, executing...')

    def _navigate_goal_pose(self, pose: Pose, wait=True) -> Optional[bool]:
        """通过 Nav2 action 发送导航目标，wait=True 时阻塞等待完成并返回是否成功"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = GLOBAL_FRAME
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = pose

        self._nav_done_event.clear()
        self._nav_success = False

        if not self._nav_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available')
            self._nav_done_event.set()
            return False if wait else None

        self.get_logger().info('Sending navigation goal via action...')
        send_goal_future = self._nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(self._nav_goal_response_callback)

        if wait:
            self.get_logger().info('Waiting for navigation to complete...')
            self._nav_done_event.wait()
            return self._nav_success

    def _nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected by Nav2')
            self._nav_done_event.set()
            return
        self.get_logger().info('Navigation goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        # feedback = feedback_msg.feedback
        # self.get_logger().info(
        #     f'Nav feedback: distance remaining={feedback.distance_remaining:.2f}m'
        # )
        pass

    def _nav_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded: goal reached!')
            self._nav_success = True
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted: goal not reachable')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled')
        else:
            self.get_logger().error(f'Navigation ended with unknown status: {status}')
        self._nav_done_event.set()
    
    def execute_zone(self):
        """执行区域：导航到达目标后逐个发送弹击目标点，IK 无解时微调底盘后重试"""
        # 以 global_wall_point 为右下角点，向上和向左拓展 4x4 测区
        goals = []  # 全局目标
        wall_mat = pose_to_matrix(self.global_wall_point)
        wall_pos = wall_mat[:3, 3]
        wall_R = wall_mat[:3, :3]  # 列：x=入墙方向, y=左方向, z=上方向
        for row in range(4):       # 向上 (z)
            for col in range(4):   # 向左 (y)
                offset = wall_R[:, 1] * col * REBOUND_POINT_SEPARATION + wall_R[:, 2] * row * REBOUND_POINT_SEPARATION
                goal_pos = wall_pos + offset
                goals.append(ndarray_to_pose(goal_pos, wall_R))

        if not self._execute_goals_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('ExecuteGoals action server not available')
            input("Execute failed. Press Enter to continue...")
            return

        failed_num = 0
        for idx, goal_pose in enumerate(goals):
            goal_finished = False
            for attempt in range(MAX_RETRIES_PER_GOAL):
                self.get_logger().info(f'Sending goal {idx + 1}/{len(goals)}, attempt {attempt + 1}')
                self._wait_for_robot_pose_update()  # 静止状态下等待机器人最新位置更新，避免上一帧位置处于移动状态
                local_wall_point = transform_pose(goal_pose, inverse_pose(self._robot_pose))
                success, need_adjust, deltas = self._send_single_goal(local_wall_point)

                if success:
                    self.get_logger().info(f'Goal {idx + 1} succeeded')
                    goal_finished = True
                    self._publish_marker(goal_pose.position, marker_name='rebound_points', id=idx, color='blue')
                    break

                if need_adjust and deltas is not None:
                    self.get_logger().info(f'Adjusting chassis: dx={deltas[0]:.3f} dy={deltas[1]:.3f} dz={deltas[2]:.3f} yaw={deltas[3]:.3f}')
                    if self._adjust_chassis(*deltas):
                        continue  # 底盘微调成功，重试当前目标
                    else:
                        self.get_logger().error(f'Chassis adjustment failed, skipping goal {idx + 1}')
                        break
                else:
                    self.get_logger().error(f'Goal {idx + 1} failed unrecoverable, skipping')
                    break
            if not goal_finished:
                self.get_logger().error(f'Goal {idx + 1} failed after {MAX_RETRIES_PER_GOAL} attempts')
                failed_num += 1

        input(f'Execute finished. Success {len(goals) - failed_num}/{len(goals)} goals. Press Enter to continue...')
        self.planner_state = "Ready"
        self.get_logger().info('Finding zone...')

    def _send_single_goal(self, goal_pose: Pose) -> tuple[bool, bool, Optional[list[float]]]:
        """发送单个目标，返回 (success, need_adjust, deltas)"""
        self._execute_done_event.clear()
        self._execute_result = None
        self._last_feedback_status = ""
        self._pending_deltas = None

        goal_msg = ExecuteGoals.Goal()
        goal_msg.target = goal_pose

        send_goal_future = self._execute_goals_client.send_goal_async(
            goal_msg,
            feedback_callback=self._execute_feedback_callback
        )
        send_goal_future.add_done_callback(self._execute_goal_response_callback)
        self._execute_done_event.wait()
        
        if self._execute_result is None:
            self.get_logger().error('ExecuteGoals result not available')
            return False, False, None

        need_adjust = (self._last_feedback_status == "no_ik_solution")
        print(f'need_adjust: {need_adjust}, self._last_feedback_status: {self._last_feedback_status}')
        return self._execute_result.success, need_adjust, self._pending_deltas

    def _execute_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('ExecuteGoals rejected')
            self._execute_done_event.set()
            return
        self.get_logger().info('ExecuteGoals accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._execute_result_callback)

    def _execute_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._last_feedback_status = fb.status
        if fb.status == "executing":
            self.get_logger().info('Arm executing...')
        elif fb.status == "success":
            self.get_logger().info('Goal succeeded')
        elif fb.status == "no_ik_solution":
            self._pending_deltas = [fb.chassis_delta_x, fb.chassis_delta_y,
                                     fb.chassis_delta_z, fb.chassis_delta_yaw]
            for i in range(len(self._pending_deltas)):
                if 1e-6 < abs(self._pending_deltas[i]) < MOVE_PRECISION:
                    self._pending_deltas[i] = (self._pending_deltas[i] + MOVE_PRECISION) if self._pending_deltas[i] > 0 else (self._pending_deltas[i] - MOVE_PRECISION)
                    self.get_logger().warn(f'偏差小于导航精度，进行过量补偿')
            self.get_logger().warn(
                f'No IK solution, chassis deltas: '
                f'x={self._pending_deltas[0]:.3f} y={self._pending_deltas[1]:.3f} '
                f'z={self._pending_deltas[2]:.3f} yaw={self._pending_deltas[3]:.3f}'
            )
        elif fb.status == "goal_state_invalid":
            self.get_logger().warn('Goal state invalid (pitch out of range)')
        else:
            self.get_logger().error(f'Goal failed: {fb.status}')

    def _execute_result_callback(self, future):
        result = future.result().result
        self._execute_result = result
        self._execute_done_event.set()

    def _adjust_chassis(self, dx, dy, dz, dyaw):
        """根据 base_link 系下的微调量，通过 Nav2 调整底盘位置，阻塞等待完成"""
        if self._robot_pose is None:
            self.get_logger().error('No robot pose for chassis adjustment')
            return False

        if abs(dz) > 0.001:
            self.get_logger().warn(f'Chassis cannot adjust z axis, dz={dz:.3f} ignored')

        try:
            tf = self._tf_buffer.lookup_transform(GLOBAL_FRAME, 'base_link', rclpy.time.Time())
            q = tf.transform.rotation
            _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return False

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        map_dx = dx * cos_yaw - dy * sin_yaw
        map_dy = dx * sin_yaw + dy * cos_yaw
        new_yaw = current_yaw + dyaw

        goal_pose = Pose()
        goal_pose.position.x = self._robot_pose.position.x + map_dx
        goal_pose.position.y = self._robot_pose.position.y + map_dy
        goal_pose.position.z = 0.0
        q_arr = quaternion_from_euler(0.0, 0.0, new_yaw)
        goal_pose.orientation.x = q_arr[0]
        goal_pose.orientation.y = q_arr[1]
        goal_pose.orientation.z = q_arr[2]
        goal_pose.orientation.w = q_arr[3]

        self.get_logger().info(
            f'Chassis adjusting: map_dx={map_dx:.3f} map_dy={map_dy:.3f} dyaw={dyaw:.3f}'
            f' -> x={goal_pose.position.x:.3f} y={goal_pose.position.y:.3f} yaw={new_yaw:.3f}'
        )

        self._publish_marker(
            np.array([goal_pose.position.x, goal_pose.position.y, goal_pose.position.z]),
            marker_name='chassis_adjust', color='yellow'
        )

        return self._navigate_goal_pose(goal_pose, wait=True)


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