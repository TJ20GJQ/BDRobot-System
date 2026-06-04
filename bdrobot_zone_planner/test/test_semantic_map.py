"""
独立测试脚本：不依赖 ROS，测试 semantic_map 模块核心逻辑

用法:
    python3 test/test_semantic_map.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'bdrobot_zone_planner'))

from typing import Tuple
import numpy as np
from semantic_map import (
    SemanticMap, PointAccumulator, RANSACExtractor,
    SemanticClassifier, ComponentManager, visualize_components,
)
from visualization_msgs.msg import Marker


def generate_wall_points(
    origin: np.ndarray,
    normal: np.ndarray,
    width: float,
    height: float,
    density: int = 200,
    noise: float = 0.01,
) -> np.ndarray:
    """生成一面墙的合成点云"""
    # 在墙面上采样点
    u = np.random.uniform(0, width, density)
    v = np.random.uniform(0, height, density)

    # 构建墙面坐标系: normal方向垂直于墙, u沿宽度, v沿高度
    # 找到两个与 normal 正交的方向
    if abs(normal[2]) < 0.9:
        up = np.array([0.0, 0.0, 1.0])
    else:
        up = np.array([0.0, 1.0, 0.0])
    dir_u = np.cross(normal, up)
    dir_u /= np.linalg.norm(dir_u)
    dir_v = np.cross(normal, dir_u)
    dir_v /= np.linalg.norm(dir_v)

    points = origin + np.outer(u, dir_u) + np.outer(v, dir_v)
    points += np.random.normal(0, noise, points.shape)
    return points


def generate_floor_ceiling(
    z: float,
    x_range: Tuple[float, float],
    y_range: Tuple[float, float],
    density: int = 500,
    noise: float = 0.01,
) -> np.ndarray:
    """生成地面或天花板点云"""
    x = np.random.uniform(x_range[0], x_range[1], density)
    y = np.random.uniform(y_range[0], y_range[1], density)
    z_arr = np.full(density, z)
    points = np.column_stack([x, y, z_arr])
    points += np.random.normal(0, noise, points.shape)
    return points


def test_point_accumulator():
    """测试体素累积器"""
    print("=== Test PointAccumulator ===")
    acc = PointAccumulator(voxel_size=0.1)

    pts1 = np.array([[0.0, 0.0, 0.0], [0.2, 0.0, 0.0], [0.5, 0.5, 0.5]])
    acc.add_points(pts1)
    ds1 = acc.get_downsampled()
    print(f"  After 1 frame: {acc.voxel_count} voxels, {len(ds1)} downsampled points")
    assert acc.voxel_count == 3

    pts2 = np.array([[0.02, 0.03, 0.04], [0.6, 0.6, 0.6]])
    acc.add_points(pts2)
    ds2 = acc.get_downsampled()
    print(f"  After 2 frames: {acc.voxel_count} voxels, {len(ds2)} downsampled points")
    assert acc.voxel_count == 4  # 第一个体素合并了

    print("  PASSED\n")


def test_ransac():
    """测试RANSAC平面提取"""
    print("=== Test RANSAC Extractor ===")

    # 生成两个垂直墙面
    wall1 = generate_wall_points(
        origin=np.array([0, 0, 1.5]),
        normal=np.array([0, 1, 0]),   # 法向量指向 Y+
        width=4, height=3, density=300,
    )
    wall2 = generate_wall_points(
        origin=np.array([2, 0, 1.5]),
        normal=np.array([-1, 0, 0]),  # 法向量指向 X-
        width=4, height=3, density=300,
    )
    # 地面
    floor = generate_floor_ceiling(z=0, x_range=(-2, 4), y_range=(-2, 4), density=400)

    points = np.vstack([wall1, wall2, floor])
    np.random.shuffle(points)

    extractor = RANSACExtractor()
    planes = extractor.sequential_plane_ransac(
        points, num_planes=5, dist_thresh=0.05, min_inliers=80,
    )

    print(f"  Found {len(planes)} planes:")
    for i, plane in enumerate(planes):
        angle_z = np.degrees(np.arccos(np.clip(np.abs(np.dot(plane.normal, [0,0,1])), 0, 1)))
        comp_type = SemanticClassifier.classify(plane.normal)
        print(f"    Plane {i}: normal={plane.normal}, inliers={plane.inlier_count}, "
              f"angle_z={angle_z:.1f}°, type={comp_type}")

    assert len(planes) >= 3, f"Expected >=3 planes, got {len(planes)}"
    print("  PASSED\n")


def test_semantic_map():
    """测试完整语义地图流程"""
    print("=== Test SemanticMap (incremental) ===")

    smap = SemanticMap(
        voxel_size=0.1,
        ransac_dist_thresh=0.05,
        ransac_min_inliers=80,
        confirm_total_points=300,
        confirm_density=5.0,
        run_ransac_every_n_frames=2,
    )

    # 模拟多帧输入
    for frame_idx in range(6):
        wall1 = generate_wall_points(np.array([0, 0, 1.5]), np.array([0, 1, 0]),
                                      width=4, height=3, density=200)
        wall2 = generate_wall_points(np.array([2, 0, 1.5]), np.array([-1, 0, 0]),
                                      width=4, height=3, density=200)
        floor = generate_floor_ceiling(z=0, x_range=(-2, 4), y_range=(-2, 4), density=300)
        points = np.vstack([wall1, wall2, floor])

        smap.add_frame(points)

        walls = smap.get_confirmed_walls()
        all_comps = list(smap.components.values())
        confirmed_count = sum(1 for c in all_comps if c.confirmed)
        print(f"  Frame {frame_idx}: {len(all_comps)} components, "
              f"{confirmed_count} confirmed, {len(walls)} walls confirmed")

    # 测试投影
    rough_point = np.array([0.55, 1.3, 2.0])  # 粗略目标点（模拟相机估计位置）
    result = smap.query_nearest_wall(rough_point)

    if result is not None:
        projected, normal = result
        print(f"\n  Rough point:  {rough_point}")
        print(f"  Projected to: {projected}")
        print(f"  Wall normal:  {normal}")
        print(f"  Offset:       {np.linalg.norm(projected - rough_point):.4f}m")
    else:
        print("  No confirmed wall found to project onto")

    assert result is not None, "Expected a projection result"
    print("  PASSED\n")


def test_classifier():
    """测试语义分类器"""
    print("=== Test SemanticClassifier ===")

    test_cases = [
        (np.array([0, 0, 1]), "floor"),             # 0°
        (np.array([0, 0, -1]), "ceiling"),          # 180°
        (np.array([1, 0, 0]), "wall"),              # 90°
        (np.array([0, 1, 0]), "wall"),              # 90°
        (np.array([0.174, 0, 0.985]), "floor"),     # ~10°, normal nearly up
        (np.array([0.174, 0, -0.985]), "ceiling"),  # ~170°, normal nearly down
        (np.array([0.940, 0, 0.342]), "wall"),      # ~70°, wall boundary
    ]

    for normal, expected in test_cases:
        result = SemanticClassifier.classify(normal)
        status = "OK" if result == expected else f"FAIL (expected {expected})"
        print(f"  normal={normal} → {result} {status}")
        assert result == expected, f"Classification error: {normal} → {result} != {expected}"

    print("  PASSED\n")


def test_visualization():
    """测试可视化接口（不实际发布，仅验证生成 MarkerArray）"""
    print("=== Test Visualization ===")

    from semantic_map import Component

    comps = {
        "wall_01": Component(
            id="wall_01", comp_type="wall",
            normal=np.array([0, 1, 0]), centroid=np.array([0, 0, 1.5]),
            d=-1.5, total_points=800, density=60, confirmed=True,
            frames_observed=5,
            inlier_points=generate_wall_points(np.array([0, 0, 1.5]),
                                               np.array([0, 1, 0]), 4, 3, 200),
        ),
        "floor_01": Component(
            id="floor_01", comp_type="floor",
            normal=np.array([0, 0, 1]), centroid=np.array([1, 1, 0]),
            d=0, total_points=1200, density=80, confirmed=True,
            frames_observed=5,
            inlier_points=generate_floor_ceiling(z=0, x_range=(-2, 4), y_range=(-2, 4), density=200),
        ),
    }

    marker_array = visualize_components(comps, frame_id="map")
    print(f"  Generated {len(marker_array.markers)} markers")
    # 统计各类型 marker
    types = {}
    for m in marker_array.markers:
        tname = {Marker.SPHERE_LIST: "SPHERE_LIST", Marker.TRIANGLE_LIST: "TRIANGLE_LIST",
                 Marker.LINE_STRIP: "LINE_STRIP", Marker.ARROW: "ARROW",
                 Marker.TEXT_VIEW_FACING: "TEXT_VIEW_FACING"}.get(m.type, f"TYPE_{m.type}")
        types[tname] = types.get(tname, 0) + 1
    for t, c in types.items():
        print(f"    {t}: {c}")
    assert len(marker_array.markers) > 0
    print("  PASSED\n")


if __name__ == "__main__":
    np.random.seed(42)
    test_point_accumulator()
    test_classifier()
    test_ransac()
    test_semantic_map()
    test_visualization()
    print("=== ALL TESTS PASSED ===")
