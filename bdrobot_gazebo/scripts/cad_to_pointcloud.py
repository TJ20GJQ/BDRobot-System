#!/usr/bin/env python3
"""
将CAD建筑图纸转换为点云的Python实现
支持DXF格式的CAD文件
"""

import numpy as np
import open3d as o3d
import ezdxf
import math
import argparse
from typing import List, Tuple

class CADToPointCloudConverter:
    """CAD图纸到点云的转换器"""
    
    def __init__(self, scale: float = 1.0, point_density: float = 0.01):
        """
        初始化转换器
        
        Args:
            scale: 缩放因子，将CAD坐标转换为实际尺寸
            point_density: 点云密度，值越小点越密集
        """
        self.scale = scale
        self.point_density = point_density
    
    def read_dxf_file(self, file_path: str) -> ezdxf.document.Drawing:
        """读取DXF文件"""
        try:
            doc = ezdxf.readfile(file_path)
            return doc
        except ezdxf.DXFError as e:
            print(f"读取DXF文件失败: {e}")
            raise
    
    def extract_geometry(self, doc: ezdxf.document.Drawing) -> List[Tuple[str, List[Tuple[float, float]]]]:
        """提取CAD文件中的几何图形"""
        modelspace = doc.modelspace()
        geometry = []
        
        # 提取线段
        for line in modelspace.query('LINE'):
            start = line.dxf.start
            end = line.dxf.end
            geometry.append(('line', [(start.x, start.y), (end.x, end.y)]))
        
        # 提取多段线
        for polyline in modelspace.query('LWPOLYLINE'):
            points = [(p[0], p[1]) for p in polyline]
            geometry.append(('polyline', points))
        
        # 提取圆弧
        for arc in modelspace.query('ARC'):
            center = arc.dxf.center
            radius = arc.dxf.radius
            start_angle = arc.dxf.start_angle
            end_angle = arc.dxf.end_angle
            geometry.append(('arc', [(center.x, center.y), radius, start_angle, end_angle]))
        
        # 提取圆
        for circle in modelspace.query('CIRCLE'):
            center = circle.dxf.center
            radius = circle.dxf.radius
            geometry.append(('circle', [(center.x, center.y), radius]))
        
        return geometry
    
    def line_to_points(self, start: Tuple[float, float], end: Tuple[float, float]) -> List[Tuple[float, float, float]]:
        """将线段转换为点云"""
        start_x, start_y = start
        end_x, end_y = end
        
        # 计算线段长度
        length = math.hypot(end_x - start_x, end_y - start_y)
        
        # 根据密度计算点数
        num_points = max(2, int(length / self.point_density))
        
        # 生成点
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_x + t * (end_x - start_x)
            y = start_y + t * (end_y - start_y)
            points.append((x * self.scale, y * self.scale, 0.0))
        
        return points
    
    def polyline_to_points(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float, float]]:
        """将多段线转换为点云"""
        result = []
        
        for i in range(len(points) - 1):
            segment_points = self.line_to_points(points[i], points[i+1])
            result.extend(segment_points)
        
        # 如果是闭合多段线，连接最后一个点和第一个点
        if len(points) >= 3 and points[0] == points[-1]:
            segment_points = self.line_to_points(points[-1], points[0])
            result.extend(segment_points)
        
        return result
    
    def arc_to_points(self, center: Tuple[float, float], radius: float, start_angle: float, end_angle: float) -> List[Tuple[float, float, float]]:
        """将圆弧转换为点云"""
        # 将角度转换为弧度
        start_rad = math.radians(start_angle)
        end_rad = math.radians(end_angle)
        
        # 处理角度跨越360度的情况
        if end_rad < start_rad:
            end_rad += 2 * math.pi
        
        # 计算弧长
        arc_length = radius * (end_rad - start_rad)
        
        # 根据密度计算点数
        num_points = max(5, int(arc_length / self.point_density))
        
        # 生成点
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            angle = start_rad + t * (end_rad - start_rad)
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            points.append((x * self.scale, y * self.scale, 0.0))
        
        return points
    
    def circle_to_points(self, center: Tuple[float, float], radius: float) -> List[Tuple[float, float, float]]:
        """将圆转换为点云"""
        return self.arc_to_points(center, radius, 0.0, 360.0)
    
    def geometry_to_pointcloud(self, geometry: List[Tuple[str, List]]) -> List[Tuple[float, float, float]]:
        """将所有几何图形转换为点云"""
        pointcloud = []
        
        for geom_type, data in geometry:
            if geom_type == 'line':
                points = self.line_to_points(data[0], data[1])
                pointcloud.extend(points)
            elif geom_type == 'polyline':
                points = self.polyline_to_points(data)
                pointcloud.extend(points)
            elif geom_type == 'arc':
                center, radius, start_angle, end_angle = data[0], data[1], data[2], data[3]
                points = self.arc_to_points(center, radius, start_angle, end_angle)
                pointcloud.extend(points)
            elif geom_type == 'circle':
                center, radius = data[0], data[1]
                points = self.circle_to_points(center, radius)
                pointcloud.extend(points)
        
        return pointcloud
    
    def add_height_to_pointcloud(self, pointcloud: List[Tuple[float, float, float]], height: float) -> List[Tuple[float, float, float]]:
        """为2D点云添加高度信息，转换为3D点云"""
        return [(x, y, height) for x, y, _ in pointcloud]
    
    def visualize_pointcloud(self, pointcloud: List[Tuple[float, float, float]]):
        """可视化点云"""
        # 转换为open3d点云格式
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(pointcloud))
        
        # 设置点云颜色为蓝色
        pcd.paint_uniform_color([0, 0, 1])
        
        # 创建坐标系
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=[0, 0, 0])
        
        # 可视化
        o3d.visualization.draw_geometries([pcd, coordinate_frame],
                                         window_name="CAD转换的点云",
                                         width=800,
                                         height=600)
    
    def save_pointcloud(self, pointcloud: List[Tuple[float, float, float]], output_file: str):
        """保存点云到文件"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(pointcloud))
        
        # 支持多种格式：.pcd, .ply, .xyz等
        o3d.io.write_point_cloud(output_file, pcd)
        print(f"点云已保存到: {output_file}")
    
    def convert(self, input_file: str, output_file: str = None, height: float = 0.0, visualize: bool = False) -> List[Tuple[float, float, float]]:
        """
        执行完整的转换流程
        
        Args:
            input_file: 输入CAD文件路径
            output_file: 输出点云文件路径（可选）
            height: 3D点云的高度值
            visualize: 是否可视化结果
            
        Returns:
            转换后的点云数据
        """
        print(f"正在读取CAD文件: {input_file}")
        doc = self.read_dxf_file(input_file)
        
        print("正在提取几何图形...")
        geometry = self.extract_geometry(doc)
        print(f"提取到 {len(geometry)} 个几何图形")
        
        print("正在转换为点云...")
        pointcloud_2d = self.geometry_to_pointcloud(geometry)
        pointcloud_3d = self.add_height_to_pointcloud(pointcloud_2d, height)
        print(f"生成的点云包含 {len(pointcloud_3d)} 个点")
        
        if output_file:
            self.save_pointcloud(pointcloud_3d, output_file)
        
        if visualize:
            print("正在可视化点云...")
            self.visualize_pointcloud(pointcloud_3d)
        
        return pointcloud_3d

def main():
    parser = argparse.ArgumentParser(description='将CAD建筑图纸转换为点云')
    parser.add_argument('input_file', type=str, help='输入的DXF文件路径')
    parser.add_argument('-o', '--output_file', type=str, default=None, help='输出的点云文件路径（支持.pcd, .ply, .xyz等格式）')
    parser.add_argument('-s', '--scale', type=float, default=1.0, help='缩放因子')
    parser.add_argument('-d', '--density', type=float, default=0.01, help='点云密度，值越小点越密集')
    parser.add_argument('-z', '--height', type=float, default=0.0, help='3D点云的高度值')
    parser.add_argument('--visualize', action='store_true', help='可视化转换结果')
    
    args = parser.parse_args()
    
    # 创建转换器实例
    converter = CADToPointCloudConverter(
        scale=args.scale,
        point_density=args.density
    )
    
    # 执行转换
    converter.convert(
        input_file=args.input_file,
        output_file=args.output_file,
        height=args.height,
        visualize=args.visualize
    )

if __name__ == "__main__":
    main()