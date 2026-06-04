import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Quaternion
from tf_transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    translation_from_matrix
)
from sensor_msgs.msg import PointCloud2


# ============================== 数据转换函数 ===============================

def pose_to_pose_stamped(pose: Pose, frame_id: str = "map", stamp = None) -> PoseStamped:
    """
    Pose 转为 PoseStamped
    :param pose: 原始 Pose
    :param frame_id: 坐标系名称
    :param stamp: 时间戳，不传则默认 0
    :return: PoseStamped
    """
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    if stamp is not None:
        ps.header.stamp = stamp
    ps.pose = pose
    return ps


def pose_stamped_to_pose(ps: PoseStamped) -> Pose:
    """PoseStamped 转为 Pose"""
    return ps.pose


def transform_stamped_to_pose(tf_stamped: TransformStamped) -> Pose:
    """
    TransformStamped 转换为 Pose
    :param tf_stamped: 坐标变换消息
    :return: 对应位姿 Pose
    """
    pose = Pose()
    # 平移部分
    pose.position.x = tf_stamped.transform.translation.x
    pose.position.y = tf_stamped.transform.translation.y
    pose.position.z = tf_stamped.transform.translation.z
    # 姿态四元数
    pose.orientation.x = tf_stamped.transform.rotation.x
    pose.orientation.y = tf_stamped.transform.rotation.y
    pose.orientation.z = tf_stamped.transform.rotation.z
    pose.orientation.w = tf_stamped.transform.rotation.w
    return pose


def pointcloud2_to_xyz(cloud_msg: PointCloud2) -> np.ndarray:  
    """将 PointCloud2 消息转换为 (N, 3) numpy 数组"""
    data = np.frombuffer(cloud_msg.data, dtype=np.float32)
    return data.reshape(-1, cloud_msg.point_step // 4)[:, :3]


def rotation_matrix_to_quaternion(R: np.ndarray) -> Quaternion:
    """3x3 旋转矩阵转 geometry_msgs/Quaternion"""
    q = Quaternion()
    
    # 将 3x3 旋转矩阵扩展为 4x4 齐次变换矩阵
    mat = np.eye(4)
    mat[:3, :3] = R
    
    # 使用 quaternion_from_matrix 提取四元数
    q_array = quaternion_from_matrix(mat)
    
    q.x = q_array[0]
    q.y = q_array[1]
    q.z = q_array[2]
    q.w = q_array[3]
    return q


def ndarray_to_pose(position: np.ndarray, rotation: np.ndarray) -> Pose:
    """1*3 位置数组 + 3x3 旋转矩阵 → Pose"""
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation = rotation_matrix_to_quaternion(rotation)
    return pose


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """Pose → 4x4 齐次变换矩阵"""
    t = [pose.position.x, pose.position.y, pose.position.z]
    q = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    mat = quaternion_matrix(q)
    mat[0:3, 3] = t
    return mat


def matrix_to_pose(mat: np.ndarray) -> Pose:
    """4x4 齐次变换矩阵 → Pose"""
    pose = Pose()
    t = translation_from_matrix(mat)
    pose.position.x = t[0]
    pose.position.y = t[1]
    pose.position.z = t[2]
    q = quaternion_from_matrix(mat)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

# ============================== 坐标变换函数 ===============================

def transform_pose(pose_in: Pose, transform: Pose) -> Pose:
    """
    将 pose_in 按 transform 进行坐标变换
    :param pose_in: 输入位姿（例如在 B 系下）
    :param transform: 变换位姿（T_AB：B→A 的变换）
    :return: 输出位姿（在 A 系下）
    """
    mat_pose = pose_to_matrix(pose_in)
    mat_transform = pose_to_matrix(transform)
    mat_out = mat_transform @ mat_pose
    return matrix_to_pose(mat_out)


def inverse_pose(pose: Pose) -> Pose:
    """求 Pose 的逆变换"""
    mat = pose_to_matrix(pose)
    mat_inv = np.linalg.inv(mat)
    return matrix_to_pose(mat_inv)


def transform_points(points: np.ndarray, transform: Pose) -> np.ndarray:
    """将 (N, 3) 点云按 Pose 进行坐标变换

    Args:
        points: (N, 3) numpy 数组
        transform: T_AB，表示 B→A 的变换

    Returns:
        (N, 3) 变换后的点
    """
    mat = pose_to_matrix(transform)
    R = mat[:3, :3]
    t = mat[:3, 3]
    return (R @ points.T).T + t