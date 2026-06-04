#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
import cv2
import numpy as np

# 霍夫变换原理：https://www.cnblogs.com/bjxqmy/p/12331656.html

class VerticalityDetector(Node):
    def __init__(self):
        super().__init__('verticality_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # 创建发布者
        self.verticality_pub = self.create_publisher(Bool, '/vericality', 10)
        self.is_show = True  # 是否显示中间过程图像
        self.threshold = 0.01  # 判断阈值，与期望精度相关

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image, self.threshold, self.is_show)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
    
    def process_image(self, image, threshold=0.025, is_show=False):  # 控制精度
        def imshow(name, image, is_show=False):
            if is_show:
                cv2.imshow(name, image)
                cv2.waitKey(1)  # 改为 1 毫秒，避免阻塞

        imshow('raw', image, is_show)

        # 转换到 HSV 色彩空间，定义绿色HSV范围，过滤出绿色区域
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        green_image = cv2.bitwise_and(image, image, mask=green_mask)
        imshow('green_filtered', green_image, is_show=False)

        # 转换为灰度图，高斯模糊，Canny边缘检测
        gray = cv2.cvtColor(green_image, cv2.COLOR_BGR2GRAY)
        imshow('gray', gray, is_show=False)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        imshow('blurred', blurred, is_show=False)
        edges = cv2.Canny(gray, 80, 180)
        imshow('edges', edges, is_show=False)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        imshow('edges_smoothed', edges, is_show)
        # cv2.waitKey(0)

        # 使用 cv2.HoughLines 检测直线，并选择最合适的激光边缘直线
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=140)
        line_groups = self.lines_filter(lines)
        if len(line_groups) != 2:
            self.get_logger().error("line_groups must be length 2")
            raise ValueError

        # 计算中心线
        center_lines = []
        if is_show:
            new_image = image.copy()
            colors = [
                (0, 0, 255),  # 红色
                (255, 0, 0),  # 蓝色
                (255, 165, 0),  # 橙色
                (128, 0, 128), # 紫色
                (255, 0, 255), # 品红色
                (255, 192, 203)  # 粉色
            ]
            for i, group in enumerate(line_groups):
                for line in group:
                    self.draw_line(new_image, line[0], line[1], 2, colors[i%len(colors)])
                center_line = (np.mean([line[0] for line in group]), np.mean([line[1] for line in group]))
                center_lines.append(center_line)
                self.draw_line(new_image, center_line[0], center_line[1], 1, colors[i%len(colors)])
            imshow('lines', new_image, is_show)
        else:
            for group in line_groups:
                center_line = (np.mean([line[0] for line in group]), np.mean([line[1] for line in group]))
                center_lines.append(center_line)
        self.get_logger().info(f'中心线: {center_lines}')

        # 计算交叉点
        intersections = self.detect_intersection(center_lines)
        self.get_logger().info(f'检测到 {len(intersections)} 个交叉点')
        if intersections:
            intersection = intersections[0]  # 假设只有一个交叉点
            if is_show:
                cv2.circle(new_image, intersection, 3, (0, 255, 0), -1)  # 标记交叉点
        else:
            self.get_logger().error("no intersection")
            raise ValueError

        # 找出两条激光边缘直线的端点及理论中点
        two_centers = []
        lengths = []
        slopes = []  # k
        for center_line in center_lines:
            endpoints = self.find_endpoints(edges, intersection, center_line)
            if is_show:
                for endpoint in endpoints:
                    cv2.circle(new_image, endpoint, 1, (255, 0, 0), -1)  # 标记端点
            two_centers.append(self.calculate_center(endpoints[0], endpoints[1]))
            lengths.append(self.calculate_distance(endpoints[0], endpoints[1]))
            slopes.append(self.calculate_slope(endpoints[0], endpoints[1]))
            self.get_logger().info(f'检测到端点: {endpoints}, 长度为: {lengths[-1]}, 中点为: {two_centers[-1]}, 斜率为: {slopes[-1]}')

        # 计算交点，判断交叉点是否为两条直线的中点
        # if max(lengths) - min(lengths) < threshold*max(lengths):
        #     error = self.calculate_circle_radius(two_centers[0], two_centers[1], intersection)*2
        #     self.get_logger().info(f'{two_centers[0]}, {two_centers[1]}, {intersection}, 偏移误差为: {error}')
        #     if error < threshold*np.mean(lengths):
        #         self.get_logger().info(f'垂直, {intersection}, {error}, {threshold*np.mean(lengths)}')
        #         is_vertical = True
        #     else:
        #         self.get_logger().info(f'交点不在两条激光线的中点，不垂直, {intersection}, {error}, {threshold*np.mean(lengths)}')
        #         is_vertical = False
        # else:
        #     self.get_logger().info(f"两线段不一样长，一定不垂直, {lengths}, {threshold*max(lengths)}")
        #     is_vertical = False
        center_error = self.calculate_circle_radius(two_centers[0], two_centers[1], intersection)*2
        angle_error = np.abs(self.calculate_k_angle(slopes[0], slopes[1]) - np.pi/2)
        self.get_logger().info(f'{two_centers[0]}, {two_centers[1]}, {intersection}, 偏移误差为: {center_error}, 角度误差为: {angle_error}')
        if center_error < threshold*np.mean(lengths):
            if angle_error < threshold*np.pi/2:
                self.get_logger().info(f'垂直, {intersection}, {center_error}, {threshold*np.mean(lengths)}, {angle_error}, {threshold*np.pi/2}')
                is_vertical = True
            else:
                self.get_logger().info(f'角度偏差过大, 不垂直, {intersection}, {center_error}, {threshold*np.mean(lengths)}, {angle_error}, {threshold*np.pi/2}')
                is_vertical = False
        else:
            self.get_logger().info(f'交点不在两条激光线的中点，不垂直, {intersection}, {center_error}, {threshold*np.mean(lengths)}')
            is_vertical = False

        # 发布检测结果
        self.verticality_pub.publish(Bool(data=is_vertical))

        # 显示结果
        imshow('result', new_image, is_show)


    @staticmethod
    def lines_filter(lines):
        if lines is None or len(lines) < 4:
            print(f"lines: {lines}")
            raise ValueError("lines must be not None and length must be greater than 4")

        # 提取直线与原点的距离和角度
        distances = []
        angles = []
        for line in lines:
            rho, theta = line[0]
            # 统一角度范围到 [0, π)
            angle = theta % np.pi
            distances.append(rho)
            angles.append(angle)

        # 定义平行和垂直的角度阈值
        parallel_threshold = np.pi / 18  # 10 度
        perpendicular_threshold = np.pi / 3  # 60 度

        # 分组平行直线
        parallel_groups = []
        for i in range(len(angles)):
            found_group = False
            for group in parallel_groups:
                group_angles = [tup[1] for tup in group]
                if all(abs(angles[i] - angle) < parallel_threshold for angle in group_angles):
                    group.append((distances[i], angles[i]))
                    found_group = True
                    break
            if not found_group:
                parallel_groups.append([(distances[i], angles[i])])
        print(f"平行分组: {parallel_groups}")

        # 每组选择最平行的两条线
        for p, group in enumerate(parallel_groups):
            p1 = None
            p2 = None
            pair_diff_min = (np.inf, np.inf)
            for i in range(len(group)):
                for j in range(i + 1, len(group)):
                    distance_diff = abs(group[i][0] - group[j][0])
                    angle_diff = abs(group[i][1] - group[j][1])
                    if angle_diff < pair_diff_min[1]:
                        pair_diff_min = (distance_diff, angle_diff)
                        p1 = group[i]
                        p2 = group[j]
                    elif angle_diff == pair_diff_min[1]:
                        if distance_diff > pair_diff_min[0]:  # 角度相同，距离更远的线更合适
                            pair_diff_min = (distance_diff, angle_diff)
                            p1 = group[i]
                            p2 = group[j]
            if p1 is not None and p2 is not None:
                parallel_groups[p] = [p1, p2]
            else:
                parallel_groups.pop(p)
        print(f"新平行分组: {parallel_groups}")

        # 寻找垂直的直线对
        perpendicular_pairs = []
        for i in range(len(parallel_groups)):
            for j in range(i + 1, len(parallel_groups)):
                angle_diff = abs(np.mean([tup[1] for tup in parallel_groups[i]]) - np.mean([tup[1] for tup in parallel_groups[j]]))
                if abs(angle_diff - np.pi / 2) < perpendicular_threshold:
                    perpendicular_pairs.append((parallel_groups[i], parallel_groups[j], abs(angle_diff - np.pi / 2)))
        print(f"垂直分组: {perpendicular_pairs}")

        # 选择垂直度最高的两对垂直直线
        perpendicular_pairs.sort(key=lambda pair: pair[2])
        top_pair = perpendicular_pairs[0]
        print(f"选择垂直分组: {top_pair}")

        result = []
        for pair in top_pair[:2]:
            result.append(pair)

        return result

    @staticmethod
    def detect_intersection(lines):
        ''' 
        检测交点
        '''
        if len(lines) < 2:
            raise ValueError("lines must be no less than 2")

        intersections = []
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                rho1, theta1 = lines[i][0], lines[i][1]
                rho2, theta2 = lines[j][0], lines[j][1]
                # 转换为笛卡尔坐标
                a1 = np.cos(theta1)
                b1 = np.sin(theta1)
                a2 = np.cos(theta2)
                b2 = np.sin(theta2)
                denominator = a1 * b2 - a2 * b1
                if np.abs(denominator) < 1e-6:  # 考虑浮点数精度问题
                    continue
                # 克莱姆法则求交点
                x = (b2 * rho1 - b1 * rho2) / denominator
                y = (a1 * rho2 - a2 * rho1) / denominator
                intersections.append((int(x), int(y)))
        return intersections
    
    @staticmethod
    def find_endpoints(edge_image, center, line):
        """
        找出线与边缘图像的交点作为端点。
        """
        rho, theta = line
        height, width = edge_image.shape
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 5000 * (-b))
        y1 = int(y0 + 5000 * (a))
        x2 = int(x0 - 5000 * (-b))
        y2 = int(y0 - 5000 * (a))

        # 生成中心线的像素点，找到中心起始点
        debug = edge_image.copy()
        line_points = np.linspace((x1, y1), (x2, y2), num=max(abs(x1-x2), abs(y1-y2)), dtype=int)
        line_points_center = sorted(line_points, key=lambda p: VerticalityDetector.calculate_distance(p, center))[0]
        center_index = np.where((line_points == line_points_center).all(axis=1))[0][0]

        # 从中心点向两端延伸找端点，如果没有找到，就找图像边缘点
        endpoints = []
        # 向左端点延伸
        for point_index in reversed(range(0, center_index)):
            cv2.circle(debug, line_points[point_index], 1, (255, 0, 0), -1)  # 标记端点
            x, y = line_points[point_index]
            if 0 <= x < width and 0 <= y < height and edge_image[y, x] == 255:
                endpoints.append((x, y))
                break
        if len(endpoints) == 0:
            for point in line_points[:center_index]:
                x, y = point
                if 0 <= x < width and 0 <= y < height:
                    endpoints.append((x, y))
                    break
        # 向右端点延伸
        for point_index in range(center_index, len(line_points)):
            cv2.circle(debug, line_points[point_index], 1, (255, 0, 0), -1)  # 标记端点
            x, y = line_points[point_index]
            if 0 <= x < width and 0 <= y < height and edge_image[y, x] == 255:
                endpoints.append((x, y))
                break
        if len(endpoints) == 1:
            for point in reversed(line_points[center_index:]):
                x, y = point
                if 0 <= x < width and 0 <= y < height:
                    endpoints.append((x, y))
                    break
        return endpoints

    @staticmethod
    def calculate_center(point1, point2):
        """
        根据两个点计算中心点。
        """
        x1, y1 = point1
        x2, y2 = point2
        return (x1+x2)/2, (y1+y2)/2

    @staticmethod
    def calculate_distance(p1, p2):
        """
        计算两点之间的距离。

        :param p1: 第一个点的坐标，格式为 (x1, y1)
        :param p2: 第二个点的坐标，格式为 (x2, y2)
        :return: 两点之间的距离
        """
        x1, y1 = p1
        x2, y2 = p2
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    @staticmethod
    def calculate_slope(point1, point2):
        """
        计算两点之间的斜率。

        :param p1: 第一个点的坐标，格式为 (x1, y1)
        :param p2: 第二个点的坐标，格式为 (x2, y2)
        :return: 两点之间的斜率
        """
        x1, y1 = point1
        x2, y2 = point2
        if x2 - x1 == 0:  # 处理垂直直线的情况
            return float('inf')
        return (y2 - y1) / (x2 - x1)
    
    @staticmethod
    def calculate_k_angle(k1, k2):
        if k1 == float('inf') and k2 == 0 or k2 == float('inf') and k1 == 0:
            # 一条直线垂直，另一条直线水平，夹角为 90 度
            angle = np.pi/2
        elif k1 == float('inf'):
            # 第一条直线垂直，计算夹角
            angle = np.abs(np.arctan(-1 / k2))
        elif k2 == float('inf'):
            # 第二条直线垂直，计算夹角
            angle = np.abs(np.arctan(-1 / k1))
        elif 1 + k1 * k2 == 0:
            # 两条直线垂直，夹角为 90 度
            angle = np.pi/2
        else:
            # 正常情况，计算夹角
            angle = np.abs(np.arctan((k1 - k2) / (1 + k1 * k2)))
        return angle
    
    @staticmethod
    def calculate_circle_radius(point1, point2, point3):
        """
        根据三个点计算经过这三个点的圆的半径。

        :param point1: 第一个点的坐标，格式为 (x1, y1)
        :param point2: 第二个点的坐标，格式为 (x2, y2)
        :param point3: 第三个点的坐标，格式为 (x3, y3)
        :return: 圆的半径，如果三个点共线则返回 None
        """
        x1, y1 = point1
        x2, y2 = point2
        x3, y3 = point3

        # 构建线性方程组 Ax = B 来求解圆心坐标 (a, b)
        A = np.array([
            [2 * (x2 - x1), 2 * (y2 - y1)],
            [2 * (x3 - x1), 2 * (y3 - y1)]
        ])
        B = np.array([
            x2**2 - x1**2 + y2**2 - y1**2,
            x3**2 - x1**2 + y3**2 - y1**2
        ])

        try:
            # 求解线性方程组
            a, b = np.linalg.solve(A, B)
            # 计算半径
            r = VerticalityDetector.calculate_distance((a, b), point1)
            return r
        except np.linalg.LinAlgError:
            # 若三个点共线，计算三个点间的距离
            dist12 = VerticalityDetector.calculate_distance(point1, point2)
            dist13 = VerticalityDetector.calculate_distance(point1, point3)
            dist23 = VerticalityDetector.calculate_distance(point2, point3)
            max_dist = max(dist12, dist13, dist23)
            return max_dist / 2

    @staticmethod
    def draw_line(image, rho, theta, thickness, color):
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 5000 * (-b))
        y1 = int(y0 + 5000 * (a))
        x2 = int(x0 - 5000 * (-b))
        y2 = int(y0 - 5000 * (a))
        # 绘制直线
        cv2.line(image, (x1, y1), (x2, y2), color, thickness)

def main(args=None):
    rclpy.init(args=args)
    detector = VerticalityDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
