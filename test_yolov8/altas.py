# test.py 

# 导入代码依赖
import cv2
import numpy as np
from skvideo.io import vreader, FFmpegWriter
from ais_bench.infer.interface import InferSession

import time
#from det_utils import postprocess, preprocess_warpAffine, random_color, draw_bbox

cfg = {
    'conf_thres': 0.25,  # 模型置信度阈值，阈值越低，得到的预测框越多
    'iou_thres': 0.45,  # IOU阈值，高于这个阈值的重叠预测框会被过滤掉
    'input_shape': [640, 640],  # 模型输入尺寸
}

# 模型路径
model_path = 'yolov8n-seg_brick.om'

# 初始化推理模型
model = InferSession(0, model_path)

# 选择模式
infer_mode = 'image'

# 模型推理
def model_infer(image, model, cfg):
    start_time = time.time()

    # 数据预处理
    #img_pre, IM = preprocess_warpAffine(image)
    pre_time = time.time()
    #print("pre:",pre_time - start_time)
    image = image.astype(np.float32)
    image = image / 255.0
    image = image.transpose(2, 0, 1)
    image = np.ascontiguousarray(image, dtype=np.float32)
    # 模型推理
    output = model.infer([image])[0].transpose(0,2,1)  #(1, 56, 8400) to (1,8400,56)
    infer_time = time.time()
    print("infer:",infer_time - pre_time)

    # 后处理
    #boxes = postprocess(output, IM, cfg['conf_thres'], cfg['iou_thres'])
    #post_time = time.time()
    #print("post:",post_time - infer_time)

    # 绘图    
    #img = draw_bbox(boxes, image)
    #draw_time = time.time()
    #print("draw:",draw_time - post_time)

    return image

# 推理图片
def infer_image(img_path, model, cfg):
    # 通过cv2.imread()载入图像
    img = model_infer(cv2.imread(img_path), model, cfg)

    # 保存
    cv2.imwrite("infer-pose.jpg", img)
    print("save done")

# 推理视频
def infer_video(video_path, model, labels_dict, cfg):
    """视频推理"""
    image_widget = widgets.Image(format='jpeg', width=800, height=600)
    display(image_widget)

    # 读入视频
    cap = cv2.VideoCapture(video_path)
    while True:
        ret, img_frame = cap.read()
        if not ret:
            break
        # 对视频帧进行推理
        image_pred = infer_frame_with_vis(img_frame, model, labels_dict, cfg, bgr2rgb=True)
        image_widget.value = img2bytes(image_pred)

# 推理摄像头的流
def infer_camera(model, cfg):
    """外设摄像头实时推理"""
    def find_camera_index():
        max_index_to_check = 2  # Maximum index to check for camera

        for index in range(max_index_to_check):
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:
                cap.release()
                return index

        # If no camera is found
        raise ValueError("No camera found.")

    # 获取摄像头
    camera_index = find_camera_index()
    cap = cv2.VideoCapture(camera_index)
    
    # 返回当前时间
    start_time = time.time()
    counter = 0
    while True:
        # 从摄像头中读取一帧图像
        _, frame = cap.read()
        image  = model_infer(frame, model, cfg)
        counter += 1  # 计算帧数
        if frame is not None:
          try:	
            # 实时显示帧数
            if (time.time() - start_time) != 0:
              cv2.putText(image, "FPS:{0}".format(float('%.1f' % (counter / (time.time() - start_time)))), (5, 30),
            			cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 1)
              # 显示图像
              cv2.imshow('keypoint', image)
          except:
            print(frame)
        else:
        	exit(0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
   	# 释放资源
    cap.release()
    cv2.destroyAllWindows()

# 推理模式选择
if infer_mode == 'image':
    img_path = 'test.jpg'
    infer_image(img_path, model, cfg)
elif infer_mode == 'camera':
    infer_camera(model, cfg)
elif infer_mode == 'video':
    video_path = 'racing.mp4'
    infer_video(video_path, model, cfg)

