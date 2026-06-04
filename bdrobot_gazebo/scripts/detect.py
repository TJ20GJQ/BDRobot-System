import cv2
import numpy as np

# https://www.cnblogs.com/bjxqmy/p/12331656.html

def detect_intersection(lines):
    intersections = []
    if lines is not None:
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                line1 = lines[i][0]
                line2 = lines[j][0]
                x1, y1, x2, y2 = line1
                x3, y3, x4, y4 = line2
                # 计算两条直线的交点
                denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
                if denominator != 0:
                    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
                    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator
                    intersections.append((int(px), int(py)))
    return intersections

def is_midpoint(intersection, line):
    x1, y1, x2, y2 = line
    mid_x = (x1 + x2) / 2
    mid_y = (y1 + y2) / 2
    # 判断交点是否接近直线中点
    distance = np.sqrt((intersection[0] - mid_x) ** 2 + (intersection[1] - mid_y) ** 2)
    return distance < 5  # 可根据实际情况调整阈值

def imshow(name, image):
    cv2.imshow(name, image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    # 读取图像
    image = cv2.imread('cross_laser.jpg')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 使用中值滤波进一步降噪
    # blurred = cv2.medianBlur(gray, 5)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    imshow('blurred', blurred)
    
    edges = cv2.Canny(blurred, 80, 180)
    imshow('edges', edges)

    # 进行闭运算平滑边缘
    edges_smoothed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    imshow('edges_smoothed', edges_smoothed)

    # 霍夫变换检测直线
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=10, minLineLength=100, maxLineGap=10)

    # 复制一份图像用于绘制直线
    line_image = image.copy()
    colors = [
        (0, 0, 255),  # 红色
        (255, 165, 0),  # 橙色
        (255, 0, 0),  # 蓝色
        (128, 0, 128), # 紫色
        (255, 0, 255), # 品红色
        (255, 192, 203)  # 粉色
    ]
    if lines is not None:
        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            # 绘制直线
            cv2.line(line_image, (x1, y1), (x2, y2), colors[i%6], 2)
            print(x1, y1, x2, y2)
    imshow('line', line_image)

    # 计算交叉点
    intersections = detect_intersection(lines)

    if intersections:
        intersection = intersections[0]  # 假设只有一个交叉点
        cv2.circle(image, intersection, 5, (0, 255, 0), -1)  # 标记交叉点

        # 判断交叉点是否为两条直线的中点
        is_midpoint_flag = True
        for line in lines:
            if not is_midpoint(intersection, line[0]):
                is_midpoint_flag = False
                break

        if is_midpoint_flag:
            print("交叉点是两条直线的中点")
        else:
            print("交叉点不是两条直线的中点")

    # 显示结果
    imshow('result', image)

if __name__ == "__main__":
    main()