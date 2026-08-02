# -*- coding: utf-8 -*-
import cv2
import numpy as np
from ais_bench.infer.interface import InferSession
# 类别定义
CLASSES = {
0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck',
8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear',
22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase',
29: 'frisbee', 30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat',
35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle',
40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple',
48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut',
55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table', 61: 'toilet',
62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave',
69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase',
76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'
}
# 置信度阈值
CONFIDENCE = 0.4
# NMS 的 IoU 阈值
IOU = 0.45
# 为每个类别分配随机颜色
colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))
def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
"""
在图像上绘制边界框和类别标签
参数：
img - 原始图像
class_id - 类别ID
confidence - 置信度
x, y - 左上角坐标
x_plus_w, y_plus_h - 右下角坐标
"""
label = "{} {:.2f}".format(CLASSES[class_id], confidence)
color = colors[class_id]
# 画框
cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
# 获取文本大小
label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
label_width, label_height = label_size
label_x = x
label_y = y - 10 if y - 10 > label_height else y + 10
# 背景框
cv2.rectangle(img, (label_x, label_y - label_height),
(label_x + label_width, label_y + label_height), color, cv2.FILLED)
# 文字
cv2.putText(img, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX,
0.5, (0, 0, 0), 1, cv2.LINE_AA)
def main(session, original_image):
"""
加载模型，执行推理，绘制检测框并保存结果图像
参数：
session - 模型
original_image - 图片值
返回：
original_image - 画框的图片
detections - 包含每个目标信息的列表
"""
height, width, _ = original_image.shape
# 变为正方形图像用于推理
length = max(height, width)
image = np.zeros((length, length, 3), np.uint8)
image[0:height, 0:width] = original_image
# 缩放因子
scale = length / 640
# 预处理图像
blob = cv2.dnn.blobFromImage(image, scalefactor=1.0 / 255, size=(640, 640), swapRB=True)
# 模型推理
outputs = session.infer(feeds=blob, mode="static")
# 转换输出维度：从 (1, 84, 8400) -> (8400, 84)
outputs = np.array([cv2.transpose(outputs[0][0])])
rows = outputs.shape[1]
boxes = []
scores = []
class_ids = []
# 解析输出
for i in range(rows):
classes_scores = outputs[0][i][4:]
(minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
if maxScore >= CONFIDENCE:
box = [
(outputs[0][i][0] - outputs[0][i][2] / 2) * scale,  # x 左上角
(outputs[0][i][1] - outputs[0][i][3] / 2) * scale,  # y 左上角
outputs[0][i][2] * scale,  # 宽
outputs[0][i][3] * scale   # 高
]
boxes.append(box)
scores.append(maxScore)
class_ids.append(maxClassIndex)
# 非极大值抑制
result_boxes = cv2.dnn.NMSBoxes(boxes, scores, CONFIDENCE, IOU, 0.5)
detections = []
# 绘制边界框
for i in range(len(result_boxes)):
index = result_boxes[i]
box = boxes[index]
detection = {
"class_id": class_ids[index],
"class_name": CLASSES[class_ids[index]],
"confidence": scores[index],
"box": box,
"scale": scale,
}
detections.append(detection)
draw_bounding_box(
original_image,
class_ids[index],
scores[index],
round(box[0]),
round(box[1]),
round(box[0] + box[2]),
round(box[1] + box[3])
)
return original_image, detections
if __name__ == "__main__":
model_path = "yolov8s.om"
# 创建推理会话
session = InferSession(device_id=0, model_path=model_path)
# 图片推理
input_image_path = "street.jpg"
image = cv2.imread(input_image_path)
draw_image, _ = main(session, image)
# cv2.imshow("Image Detection", draw_image)
cv2.imwrite("output_image.jpg", draw_image)
cv2.waitKey(0)
cv2.destroyAllWindows()