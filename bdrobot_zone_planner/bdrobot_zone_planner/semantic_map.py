"""
增量式室内建筑构件语义地图

模块结构:
  PointAccumulator    -- 体素哈希网格累积点云，支持降采样
  RANSACExtractor     -- 顺序平面RANSAC，提取多个平面
  SemanticClassifier  -- 基于法向量方向分类建筑构件类型
  ComponentManager    -- 构件生命周期管理：数据关联、合并、置信度
  SemanticMap         -- 顶层接口：喂入点云帧 → 查询投影
  visualize_components -- 生成 MarkerArray 用于 RViz 可视化
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from collections import defaultdict
import uuid

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Pose, Point

# =========================== 数据结构 ===========================


@dataclass
class PlaneResult:
    """单帧RANSAC提取的临时结果"""
    normal: np.ndarray        # (3,) 单位法向量
    centroid: np.ndarray      # (3,) 内点质心
    d: float                  # 平面方程: x·normal + d = 0
    inlier_count: int         # 内点数
    inlier_points: np.ndarray # (M, 3) 内点


@dataclass
class Component:
    """持久化建筑构件"""
    id: str
    comp_type: str            # "wall" | "floor" | "ceiling" | "column" | "unknown"
    normal: np.ndarray        # (3,) 单位法向量
    centroid: np.ndarray      # (3,) 质心
    d: float                  # 平面方程: x·normal + d = 0
    total_points: int         # 累计内点总数
    density: float            # pts/m²
    confirmed: bool           # 是否已确认
    frames_observed: int      # 被观测到的帧数
    inlier_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    """用于可视化的最近帧内点"""


# =========================== 点云累积器 ===========================


class PointAccumulator:
    """体素哈希网格累积点云，每个体素只保留质心（运行均值）"""

    def __init__(self, voxel_size: float = 0.05):
        self.voxel_size = voxel_size
        self._centroids: Dict[Tuple[int, int, int], np.ndarray] = {}
        self._counts: Dict[Tuple[int, int, int], int] = {}
        self._total_added = 0

    def add_points(self, points: np.ndarray):
        """将一帧点云加入累积器"""
        if len(points) == 0:
            return
        indices = np.floor(points / self.voxel_size + 1e-8).astype(np.int32)
        for i in range(len(points)):
            key = (indices[i, 0].item(), indices[i, 1].item(), indices[i, 2].item())
            if key in self._centroids:
                n = self._counts[key]
                self._centroids[key] = (n * self._centroids[key] + points[i]) / (n + 1)
                self._counts[key] = n + 1
            else:
                self._centroids[key] = points[i].copy()
                self._counts[key] = 1
        self._total_added += len(points)

    def get_downsampled(self) -> np.ndarray:
        """返回降采样后的点云 (N, 3)"""
        if not self._centroids:
            return np.empty((0, 3))
        return np.array(list(self._centroids.values()))

    def get_dense_voxels(self, min_points: int = 3) -> np.ndarray:
        """只返回点数超过阈值的体素质心"""
        pts = []
        for key, count in self._counts.items():
            if count >= min_points:
                pts.append(self._centroids[key])
        if not pts:
            return np.empty((0, 3))
        return np.array(pts)

    @property
    def voxel_count(self) -> int:
        return len(self._centroids)

    def clear(self):
        self._centroids.clear()
        self._counts.clear()
        self._total_added = 0


# =========================== RANSAC 提取器 ===========================


class RANSACExtractor:
    """基于 numpy 的平面 RANSAC 提取器"""

    @staticmethod
    def sequential_plane_ransac(
        points: np.ndarray,
        num_planes: int = 10,
        dist_thresh: float = 0.05,
        min_inliers: int = 100,
        max_iterations: int = 200,
    ) -> List[PlaneResult]:
        """
        顺序RANSAC：重复提取最佳平面 → 移除内点 → 继续

        Args:
            points: (N, 3) 输入点云
            num_planes: 最多提取的平面数（安全上限）
            dist_thresh: 点到平面的距离阈值 (m)
            min_inliers: 平面最少内点数，低于此值停止提取
            max_iterations: 每次RANSAC的最大迭代次数

        Returns:
            提取到的平面列表，按内点数降序排列
        """
        remaining = points.copy()
        remaining_mask = np.ones(len(remaining), dtype=bool)
        planes: List[PlaneResult] = []

        for _ in range(num_planes):
            if remaining_mask.sum() < min_inliers:
                break

            result = RANSACExtractor._ransac_single_plane(
                remaining[remaining_mask],
                dist_thresh, min_inliers, max_iterations,
            )
            if result is None:
                break

            # 在原始点云中找出属于该平面的内点
            all_dists = np.abs(points @ result.normal + result.d)
            plane_mask = all_dists < dist_thresh
            inlier_points = points[plane_mask]

            # 使用全部内点重新拟合，得到更精准的平面方程
            normal, centroid, d = RANSACExtractor._fit_plane_svd(inlier_points)

            planes.append(PlaneResult(
                normal=normal,
                centroid=centroid,
                d=d,
                inlier_count=len(inlier_points),
                inlier_points=inlier_points,
            ))

            # 从剩余点中移除该平面的内点
            remaining_mask[plane_mask] = False

        return planes

    @staticmethod
    def _ransac_single_plane(
        points: np.ndarray,
        dist_thresh: float,
        min_inliers: int,
        max_iterations: int,
    ) -> Optional[PlaneResult]:
        """单次 RANSAC，提取一个最优平面"""
        n = len(points)
        if n < 3:
            return None

        best_normal = None
        best_centroid = None
        best_d = 0.0
        best_inlier_count = 0

        for _ in range(max_iterations):
            idx = np.random.choice(n, 3, replace=False)
            p1, p2, p3 = points[idx]

            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-10:
                continue
            normal /= norm

            d = -np.dot(normal, p1)
            dists = np.abs(points @ normal + d)
            count = (dists < dist_thresh).sum()

            if count > best_inlier_count:
                best_inlier_count = count
                best_normal = normal
                best_d = d
                best_centroid = p1  # 临时，后面会用SVD重新拟合

        if best_inlier_count < min_inliers:
            return None

        return PlaneResult(
            normal=best_normal,
            centroid=best_centroid if best_centroid is not None else np.zeros(3),
            d=best_d,
            inlier_count=best_inlier_count,
            inlier_points=np.empty((0, 3)),
        )

    @staticmethod
    def _fit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """SVD拟合平面，返回 (normal, centroid, d)"""
        centroid = np.mean(points, axis=0)
        centered = points - centroid
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[2]  # 最小奇异值对应的右奇异向量
        if normal[2] < 0:
            normal = -normal  # 统一法向量朝上（Z正方向）
        d = -np.dot(normal, centroid)
        return normal, centroid, d


# =========================== 语义分类器 ===========================


class SemanticClassifier:
    """基于法向量方向分类建筑构件"""

    @staticmethod
    def classify(
        normal: np.ndarray,
        gravity_dir: np.ndarray = np.array([0.0, 0.0, 1.0]),
        wall_angle_range: Tuple[float, float] = (70.0, 110.0),
        floor_angle_max: float = 20.0,
        ceiling_angle_min: float = 160.0,
    ) -> str:
        """
        根据法向量与重力方向的夹角分类

        Args:
            normal: 平面单位法向量
            gravity_dir: 重力方向（默认 Z轴朝上）
            wall_angle_range: 墙面法向量与重力的夹角范围（度）
            floor_angle_max: 地面法向量与重力的最大夹角
            ceiling_angle_min: 天花板法向量与重力的最小夹角

        Returns:
            "wall" | "floor" | "ceiling" | "unknown"
        """
        cos_angle = np.dot(normal, gravity_dir)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_angle))

        if angle_deg <= floor_angle_max:
            return "floor"
        elif angle_deg >= ceiling_angle_min:
            return "ceiling"
        elif wall_angle_range[0] <= angle_deg <= wall_angle_range[1]:
            return "wall"
        else:
            return "unknown"


# =========================== 辅助函数 ===========================


def _estimate_density(points: np.ndarray, normal: np.ndarray) -> float:
    """估算点云在平面上的密度 (pts/m²)

    将点投影到平面上，用有向包围盒面积作为面积估计。
    """
    if len(points) < 3:
        return float(len(points))

    centroid = np.mean(points, axis=0)
    centered = points - centroid
    proj = centered - np.outer(centered @ normal, normal)

    # PCA 找平面内主轴
    _, _, vh = np.linalg.svd(proj, full_matrices=False)
    u_axis = vh[0]
    v_axis = vh[1]

    proj_u = proj @ u_axis
    proj_v = proj @ v_axis
    width = max(proj_u.max() - proj_u.min(), 0.05)
    height = max(proj_v.max() - proj_v.min(), 0.05)
    area = width * height
    return len(points) / area


# =========================== 构件管理器 ===========================


@dataclass
class _Association:
    """数据关联结果"""
    matched: bool
    comp_id: Optional[str] = None
    score: float = 0.0


class ComponentManager:
    """
    构件生命周期管理

    每帧运行:
      1. 接收新提取的平面列表
      2. 与已有构件做数据关联
      3. 匹配 → 更新构件参数；不匹配 → 新建候选构件
      4. 对达到阈值的构件标记 confirmed
    """

    def __init__(
        self,
        confirm_total_points: int = 500,
        confirm_density: float = 20.0,       # pts/m²
        merge_normal_angle_thresh: float = 15.0,  # 度
        merge_dist_thresh: float = 0.3,      # m
        gravity_dir: np.ndarray = np.array([0.0, 0.0, 1.0]),
    ):
        self._components: Dict[str, Component] = {}
        self._frame_count = 0

        self.confirm_total_points = confirm_total_points
        self.confirm_density = confirm_density
        self.merge_normal_angle_thresh = merge_normal_angle_thresh
        self.merge_dist_thresh = merge_dist_thresh
        self.gravity_dir = gravity_dir / np.linalg.norm(gravity_dir)

    def update(self, planes: List[PlaneResult]):
        """用新一帧提取的平面更新构件列表"""
        self._frame_count += 1

        for plane in planes:
            comp_type = SemanticClassifier.classify(plane.normal, self.gravity_dir)
            assoc = self._associate(plane)

            if assoc.matched and assoc.comp_id is not None:
                self._merge(assoc.comp_id, plane)
            else:
                self._create_component(plane, comp_type)

        self._update_confirmation()

    @property
    def components(self) -> Dict[str, Component]:
        return self._components

    def get_confirmed_walls(self) -> List[Component]:
        """获取所有已确认的墙面构件"""
        return [c for c in self._components.values()
                if c.confirmed and c.comp_type == "wall"]

    def get_confirmed(self, comp_type: Optional[str] = None) -> List[Component]:
        """获取已确认构件，可按类型过滤"""
        comps = [c for c in self._components.values() if c.confirmed]
        if comp_type is not None:
            comps = [c for c in comps if c.comp_type == comp_type]
        return comps

    def _associate(self, plane: PlaneResult) -> _Association:
        """将新平面与已有构件匹配"""
        best_score = -1.0
        best_id = None

        for comp_id, comp in self._components.items():
            angle = np.degrees(np.arccos(np.clip(
                np.abs(np.dot(plane.normal, comp.normal)), 0.0, 1.0
            )))
            if angle > self.merge_normal_angle_thresh:
                continue

            # 新平面质心到已有构件平面的距离
            dist = np.abs(np.dot(plane.centroid, comp.normal) + comp.d)
            if dist > self.merge_dist_thresh:
                continue

            # 得分：角度和距离的综合（越小越好）
            score = 1.0 / (1.0 + angle + dist * 10)
            if score > best_score:
                best_score = score
                best_id = comp_id

        if best_id is not None:
            return _Association(matched=True, comp_id=best_id, score=best_score)
        return _Association(matched=False)

    def _merge(self, comp_id: str, plane: PlaneResult):
        """将新平面合并到已有构件"""
        comp = self._components[comp_id]
        w_old = comp.frames_observed
        w_new = 1.0
        total_w = w_old + w_new

        # 加权更新法向量
        comp.normal = (comp.normal * w_old + plane.normal * w_new) / total_w
        comp.normal /= np.linalg.norm(comp.normal)

        comp.centroid = (comp.centroid * w_old + plane.centroid * w_new) / total_w
        comp.d = -np.dot(comp.normal, comp.centroid)
        comp.total_points += plane.inlier_count
        comp.frames_observed += 1
        comp.inlier_points = plane.inlier_points

        comp.density = _estimate_density(plane.inlier_points, comp.normal)

    def _create_component(self, plane: PlaneResult, comp_type: str):
        """创建新构件"""
        comp_id = f"{comp_type}_{uuid.uuid4().hex[:8]}"
        density = _estimate_density(plane.inlier_points, plane.normal)

        self._components[comp_id] = Component(
            id=comp_id,
            comp_type=comp_type,
            normal=plane.normal.copy(),
            centroid=plane.centroid.copy(),
            d=plane.d,
            total_points=plane.inlier_count,
            density=density,
            confirmed=False,
            frames_observed=1,
            inlier_points=plane.inlier_points.copy(),
        )

    def _update_confirmation(self):
        """更新构件的确认状态"""
        for comp in self._components.values():
            if not comp.confirmed:
                if (comp.total_points >= self.confirm_total_points
                        and comp.density >= self.confirm_density):
                    comp.confirmed = True


# =========================== 语义地图（顶层接口）===========================


class SemanticMap:
    """
    增量式室内建筑构件语义地图

    用法:
        smap = SemanticMap(voxel_size=0.05)

        # 每帧点云到达时喂入
        smap.add_frame(points)

        # 查询目标点在墙面上的投影
        projected, normal = smap.query_nearest_wall(rough_point)
    """

    def __init__(
        self,
        voxel_size: float = 0.05,
        ransac_dist_thresh: float = 0.05,
        ransac_min_inliers: int = 100,
        ransac_num_planes: int = 10,
        confirm_total_points: int = 500,
        confirm_density: float = 20.0,
        merge_normal_angle_thresh: float = 15.0,
        merge_dist_thresh: float = 0.3,
        run_ransac_every_n_frames: int = 5,
        gravity_dir: np.ndarray = np.array([0.0, 0.0, 1.0]),
    ):
        self.accumulator = PointAccumulator(voxel_size=voxel_size)
        self.extractor = RANSACExtractor()
        self.manager = ComponentManager(
            confirm_total_points=confirm_total_points,
            confirm_density=confirm_density,
            merge_normal_angle_thresh=merge_normal_angle_thresh,
            merge_dist_thresh=merge_dist_thresh,
            gravity_dir=gravity_dir,
        )

        self.ransac_dist_thresh = ransac_dist_thresh
        self.ransac_min_inliers = ransac_min_inliers
        self.ransac_num_planes = ransac_num_planes
        self.run_ransac_every_n_frames = run_ransac_every_n_frames

        self._frame_count = 0

    def add_frame(self, points: np.ndarray):
        """
        喂入一帧点云，累积并周期性运行RANSAC提取

        Args:
            points: (N, 3) 点云（应已变换到全局坐标系）
        """
        self.accumulator.add_points(points)
        self._frame_count += 1

        if self._frame_count % self.run_ransac_every_n_frames != 0:
            return

        downsampled = self.accumulator.get_downsampled()
        self.accumulator.clear()  # 清空，下一批点云重新累积

        if len(downsampled) < self.ransac_min_inliers:
            return

        planes = self.extractor.sequential_plane_ransac(
            downsampled,
            num_planes=self.ransac_num_planes,
            dist_thresh=self.ransac_dist_thresh,
            min_inliers=self.ransac_min_inliers,
        )
        self.manager.update(planes)

    def query_nearest_wall(self, point: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        查询目标点在最近已确认墙面上的投影

        Args:
            point: (3,) 粗略目标点（全局坐标系）

        Returns:
            (projected_point, wall_normal) 或 None（无已确认墙面时）
        """
        walls = self.manager.get_confirmed_walls()
        if not walls:
            return None

        # 找最近的墙面
        best_dist = float('inf')
        best_wall = None
        for wall in walls:
            dist = np.abs(np.dot(point, wall.normal) + wall.d)
            if dist < best_dist:
                best_dist = dist
                best_wall = wall

        if best_wall is None:
            return None

        # 正交投影
        vec = point - best_wall.centroid
        dist_to_plane = np.dot(vec, best_wall.normal)
        projected = point - dist_to_plane * best_wall.normal
        return projected, best_wall.normal.copy()

    @property
    def components(self) -> Dict[str, Component]:
        return self.manager.components

    def get_confirmed_walls(self) -> List[Component]:
        return self.manager.get_confirmed_walls()


# =========================== 可视化 ===========================


def visualize_components(
    components: Dict[str, Component],
    frame_id: str = "map",
    ns: str = "semantic_map",
) -> MarkerArray:
    """
    将构件列表转换为 RViz MarkerArray 用于可视化

    每类构件用不同颜色:
      墙面  → 绿/浅绿
      地面  → 蓝
      天花板 → 黄
      柱体  → 紫
      未知  → 灰
    未确认构件透明度降低
    """
    marker_array = MarkerArray()
    now = None  # 由调用方填入时间戳

    # 类型 → 颜色映射
    color_map = {
        "wall":    ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.6),
        "floor":   ColorRGBA(r=0.2, g=0.4, b=1.0, a=0.6),
        "ceiling": ColorRGBA(r=1.0, g=0.9, b=0.2, a=0.6),
        "column":  ColorRGBA(r=0.8, g=0.2, b=1.0, a=0.6),
        "unknown": ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6),
    }

    marker_id = 0

    # 1. 每个构件的平面矩形面片
    for comp_id, comp in components.items():
        color = color_map.get(comp.comp_type, color_map["unknown"])
        if not comp.confirmed:
            color.a = 0.25  # 未确认构件更透明

        corners = _compute_oriented_bbox(comp)
        if corners is None:
            continue

        # 面片 (TRIANGLE_LIST)
        tri_marker = Marker()
        tri_marker.header.frame_id = frame_id
        tri_marker.ns = f"{ns}_surface"
        tri_marker.id = marker_id
        marker_id += 1
        tri_marker.type = Marker.TRIANGLE_LIST
        tri_marker.action = Marker.ADD
        tri_marker.scale.x = 1.0
        tri_marker.scale.y = 1.0
        tri_marker.scale.z = 1.0
        tri_marker.color = color

        # 两个三角形组成矩形: 0-1-2, 0-2-3
        pts = [
            corners[0], corners[1], corners[2],
            corners[0], corners[2], corners[3],
        ]
        tri_marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        marker_array.markers.append(tri_marker)

        # 边框 (LINE_STRIP)
        edge_marker = Marker()
        edge_marker.header.frame_id = frame_id
        edge_marker.ns = f"{ns}_edge"
        edge_marker.id = marker_id
        marker_id += 1
        edge_marker.type = Marker.LINE_STRIP
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.03
        edge_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)

        edge_pts = list(corners) + [corners[0]]
        edge_marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in edge_pts]
        marker_array.markers.append(edge_marker)

        # 法向量箭头 (显示在质心)
        arrow_marker = _make_normal_arrow(
            comp.centroid, comp.normal, frame_id, f"{ns}_normal",
            marker_id, length=0.3,
        )
        marker_id += 1
        marker_array.markers.append(arrow_marker)

        # 文字标签
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.ns = f"{ns}_label"
        text_marker.id = marker_id
        marker_id += 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.scale.z = 0.15
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        text_marker.pose.position = Point(
            x=float(comp.centroid[0]),
            y=float(comp.centroid[1]),
            z=float(comp.centroid[2] + 0.2),
        )
        text_marker.text = (
            f"{comp.comp_type}:{comp.id[-4:]}\n"
            f"pts={comp.total_points} rho={comp.density:.0f}"
        )
        marker_array.markers.append(text_marker)

    # 2. 所有构件内点 (SPHERE_LIST)
    point_marker = Marker()
    point_marker.header.frame_id = frame_id
    point_marker.ns = f"{ns}_points"
    point_marker.id = marker_id
    marker_id += 1
    point_marker.type = Marker.SPHERE_LIST
    point_marker.action = Marker.ADD
    point_marker.scale.x = 0.03
    point_marker.scale.y = 0.03
    point_marker.scale.z = 0.03
    point_marker.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.5)

    for comp in components.values():
        pts = comp.inlier_points
        if len(pts) > 1000:
            idx = np.random.choice(len(pts), 1000, replace=False)
            pts = pts[idx]
        point_marker.points.extend(
            [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]
        )
    if point_marker.points:
        marker_array.markers.append(point_marker)

    return marker_array


def _compute_oriented_bbox(
    comp: Component, margin: float = 0.05,
) -> Optional[List[np.ndarray]]:
    """计算构件在平面上的有向包围盒4个角点"""
    points = comp.inlier_points
    if len(points) < 3:
        return None

    normal = comp.normal
    centroid = comp.centroid

    # 将内点投影到平面上
    centered = points - centroid
    proj = centered - np.outer(centered @ normal, normal)

    # 在平面上做PCA找主轴
    _, _, vh = np.linalg.svd(proj, full_matrices=False)
    u = vh[0]   # 第一主轴
    v = vh[1]   # 第二主轴

    # 沿主轴的范围
    proj_u = proj @ u
    proj_v = proj @ v
    u_min, u_max = proj_u.min() - margin, proj_u.max() + margin
    v_min, v_max = proj_v.min() - margin, proj_v.max() + margin

    corners_local = np.array([
        [u_min, v_min],
        [u_max, v_min],
        [u_max, v_max],
        [u_min, v_max],
    ])

    corners = []
    for cu, cv in corners_local:
        corner = centroid + cu * u + cv * v
        corners.append(corner)

    return corners


def _make_normal_arrow(
    origin: np.ndarray,
    normal: np.ndarray,
    frame_id: str,
    ns: str,
    marker_id: int,
    length: float = 0.3,
) -> Marker:
    """在指定位置创建法向量箭头"""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.02   # 杆直径
    marker.scale.y = 0.05   # 箭头直径
    marker.scale.z = 0.0    # (未使用)

    # 箭头从 origin → origin + normal*length
    start = origin
    end = origin + normal * length
    marker.points = [
        Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
        Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
    ]

    marker.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.8)
    return marker
