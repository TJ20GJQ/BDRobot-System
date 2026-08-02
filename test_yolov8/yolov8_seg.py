import argparse
import time
import cv2
import numpy as np
import os
from ais_bench.infer.interface import InferSession

class YOLO:
    """YOLO segmentation model class for handling inference"""
    def __init__(self, om_model, imgsz=(640, 640), device_id=0, model_ndtype=np.single, mode="static", postprocess_type="v8", aipp=False):
        """
        Initialization.
        Args:
        om_model (str): Path to the om model.
        """
        # 构建ais_bench推理引擎
        self.session = InferSession(device_id=device_id, model_path=om_model)

        # Numpy dtype: support both FP32(np.single) and FP16(np.half) om model
        self.ndtype = model_ndtype
        self.mode = mode
        self.postprocess_type = postprocess_type
        self.aipp = aipp
        self.model_height, self.model_width = imgsz[0], imgsz[1]  # 图像resize大小

    def __call__(self, im0, conf_threshold=0.4, iou_threshold=0.45):
        """
        The whole pipeline: pre-process -> inference -> post-process.
        Args:
        im0 (Numpy.ndarray): original input image.
        conf_threshold (float): confidence threshold for filtering predictions.
        iou_threshold (float): iou threshold for NMS.
        Returns:
        boxes (List): list of bounding boxes.
        """
        # 前处理Pre-process
        t1 = time.time()
        im, ratio, (pad_w, pad_h) = self.preprocess(im0)
        pre_time = round(time.time() - t1, 3)

        # 推理 inference
        t2 = time.time()
        preds = self.session.infer([im], mode=self.mode)  # mode有动态"dymshape"和静态"static"等
        det_time = round(time.time() - t2, 3)

        # 后处理Post-process
        t3 = time.time()
        if self.postprocess_type == "v5":
            boxes, segments, masks = self.postprocess_v5(preds,
                im0=im0,
                ratio=ratio,
                pad_w=pad_w,
                pad_h=pad_h,
                conf_threshold=conf_threshold,
                iou_threshold=iou_threshold,
            )
        elif self.postprocess_type == "v8":
            boxes, segments, masks = self.postprocess_v8(preds,
            im0=im0,
            ratio=ratio,
            pad_w=pad_w,
            pad_h=pad_h,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            )
        else:
            boxes = [], segments = [], masks = []
        
        post_time = round(time.time() - t3, 3)

        return boxes, segments, masks, (pre_time, det_time, post_time)
    
    # 前处理，包括：resize, pad, 其中HWC to CHW，BGR to RGB，归一化，增加维度CHW -> BCHW可选择是否开启AIPP加速处理
    def preprocess(self, img):
        """
        Pre-processes the input image.
        Args:
        img (Numpy.ndarray): image about to be processed.
        Returns:
        img_process (Numpy.ndarray): image preprocessed for inference.
        ratio (tuple): width, height ratios in letterbox.
        pad_w (float): width padding in letterbox.
        pad_h (float): height padding in letterbox.
        """
        # Resize and pad input image using letterbox() (Borrowed from Ultralytics)
        shape = img.shape[:2]  # original image shape
        new_shape = (self.model_height, self.model_width)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        ratio = r, r
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        pad_w, pad_h = (new_shape[1] - new_unpad[0]) / 2, (new_shape[0] - new_unpad[1]) / 2  # wh padding
        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        
        top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
        left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))  # 填充
        
        # 是否开启aipp加速预处理，需atc中完成
        if self.aipp:
            return img, ratio, (pad_w, pad_h)
        
        # Transforms: HWC to CHW -> BGR to RGB -> div(255) -> contiguous -> add axis(optional)
        img = np.ascontiguousarray(np.einsum('HWC->CHW', img)[::-1], dtype=self.ndtype) / 255.0
        img_process = img[None] if len(img.shape) == 3 else img
        return img_process, ratio, (pad_w, pad_h)
    
    # YOLOv5/6/7通用后处理，包括：阈值过滤与NMS+masks处理
    def postprocess_v5(self, preds, im0, ratio, pad_w, pad_h, conf_threshold, iou_threshold, nm=32):
        """
        Post-process the prediction.
        Args:
        preds (Numpy.ndarray): predictions come from ort.session.run().
        im0 (Numpy.ndarray): [h, w, c] original input image.
        ratio (tuple): width, height ratios in letterbox.
        pad_w (float): width padding in letterbox.
        pad_h (float): height padding in letterbox.
        conf_threshold (float): conf threshold.
        iou_threshold (float): iou threshold.
        nm (int): the number of masks.
        Returns:
        boxes (List): list of bounding boxes.
        segments (List): list of segments.
        masks (np.ndarray): [N, H, W], output masks.
        """
        # (Batch_size, Num_anchors, xywh_score_conf_cls), v5和v6_1.0的[..., 4]是置信度分数，v8v9采用类别里面最大的概率作为置信度score
        x, protos = preds[0], preds[1]  # 与bbox区别：Two outputs: 检测头的输出(1, 8400*3, 117), 分割头的输出(1, 32, 160, 160)
        # Predictions filtering by conf-threshold
        x = x[x[..., 4] > conf_threshold]
        # Create a new matrix which merge these(box, score, cls, nm) into one
        # For more details about `numpy.c_()`: https://numpy.org/doc/1.26/reference/generated/numpy.c_.html
        x = np.c_[x[..., :4], x[..., 4], np.argmax(x[..., 5:-nm], axis=-1), x[..., -nm:]]
        # NMS filtering
        # 经过NMS后的值, np.array([[x, y, w, h, conf, cls, nm], ...]), shape=(-1, 4 + 1 + 1 + 32)
        x = x[cv2.dnn.NMSBoxes(x[:, :4], x[:, 4], conf_threshold, iou_threshold)]
        # 重新缩放边界框，为画图做准备
        if len(x) > 0:
            # Bounding boxes format change: cxcywh -> xyxy
            x[..., [0, 1]] -= x[..., [2, 3]] / 2
            x[..., [2, 3]] += x[..., [0, 1]]
            # Rescales bounding boxes from model shape(model_height, model_width) to the shape of original image
            x[..., :4] -= [pad_w, pad_h, pad_w, pad_h]
            x[..., :4] /= min(ratio)
            # Bounding boxes boundary clamp
            x[..., [0, 2]] = x[:, [0, 2]].clip(0, im0.shape[1])
            x[..., [1, 3]] = x[:, [1, 3]].clip(0, im0.shape[0])
            # 与bbox区别：增加masks处理
            # Process masks
            masks = self.process_mask(protos[0], x[:, 6:], x[:, :4], im0.shape)
            # Masks -> Segments(contours)
            segments = self.masks2segments(masks)
            return x[..., :6], segments, masks  # boxes, segments, masks
        else:
            return [], [], []
    
    def postprocess_v8(self, preds, im0, ratio, pad_w, pad_h, conf_threshold, iou_threshold):
        x, protos = preds[0], preds[1]  # x: (1, 4+N+32, 8400), protos: (1, 32, 160, 160)
        
        # 统一为 (8400, 4+N+32)
        if x.ndim == 3:
            if x.shape[1] < x.shape[2]:  # (1, 4+N+32, 8400)
                x = np.einsum('bcn->bnc', x)[0]  # -> (8400, 4+N+32)
            else:  # (1, 8400, 37)
                x = x[0]
        else:
            raise ValueError(f'unexpected pred0 shape: {x.shape}')
        
        # 动态确定 nm（mask channel, default 32）
        nm = protos.shape[1] if (protos.ndim == 4 and protos.shape[1] in (16, 32, 64)) else 32

        # 取类别分支并按需做 sigmoid
        cls_blob = x[:, 4:-nm]  # 维度应为 (8400, num_classes)；模型 num_classes=3
        if cls_blob.size and (cls_blob.max() > 1 or cls_blob.min() < 0):
            cls_scores = 1.0 / (1.0 + np.exp(-cls_blob))
        else:
            cls_scores = cls_blob
        if cls_scores.size == 0:
            return [], [], []

        scores = cls_scores.max(axis=-1)
        clses = cls_scores.argmax(axis=-1)
        keep = scores > conf_threshold  # (8400)
        if not np.any(keep):
            return [], [], []
        
        # 拼接为统一数组
        x = np.c_[x[keep, :4], scores[keep], clses[keep], x[keep, -nm:]]  # [cx,cy,w,h,score,cls,32-vec]

        # === NMS 前先把 cx,cy,w,h -> x,y,w,h（左上+宽高）===
        tlwh = np.c_[x[:, 0] - x[:, 2] / 2, x[:, 1] - x[:, 3] / 2, x[:, 2], x[:, 3]]
        idxs = cv2.dnn.NMSBoxes(tlwh.tolist(), x[:, 4].astype(float).tolist(), conf_threshold, iou_threshold)  # valid frames
        if len(idxs) == 0:
            return [], [], []
        idxs = np.array(idxs).reshape(-1)
        x = x[idxs]
        
        # 转回 xyxy 并映射回原图
        x[..., [0, 1]] -= x[..., [2, 3]] / 2
        x[..., [2, 3]] += x[..., [0, 1]]
        x[:, :4] -= [pad_w, pad_h, pad_w, pad_h]
        x[:, :4] /= min(ratio)
        x[:, [0, 2]] = x[:, [0, 2]].clip(0, im0.shape[1])
        x[:, [1, 3]] = x[:, [1, 3]].clip(0, im0.shape[0])  # (n, 38)

        # 处理 mask
        masks = self.process_mask(protos[0], x[:, 6:], x[:, :4], im0.shape)
        segments = self.masks2segments(masks)
        return x[:, :6], segments, masks
    
    @staticmethod
    def masks2segments(masks):
        """
        It takes a list of masks(n,h,w) and returns a list of segments(n,xy) (Borrowed from
        https://github.com/ultralytics/ultralytics/blob/465df3024f44fa97d4fad9986530d5a13cdabdca/ultralytics/utils/ops.py#L750)
        Args:
        masks (numpy.ndarray): the output of the model, which is a tensor of shape (batch_size, 160, 160).
        Returns:
        segments (List): list of segment masks.
        """
        segments = []
        for x in masks.astype('uint8'):  # (n,640,640)
            c = cv2.findContours(x, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]  # CHAIN_APPROX_SIMPLE  该函数用于查找二值图像中的轮廓 (k,1,2)
            if c:
                # 这段代码的目的是找到图像x中的最外层轮廓，并从中选择最长的轮廓，然后将其转换为NumPy数组的形式
                c = np.array(c[np.array([len(x) for x in c]).argmax()]).reshape(-1, 2)  # (k,2)
            else:
                c = np.zeros((0, 2))  # no segments found
            segments.append(c.astype('float32'))
        return segments
    
    def process_mask(self, protos, masks_in, bboxes, im0_shape):
        c, mh, mw = protos.shape  # (32, 160, 160)
        masks = np.matmul(masks_in, protos.reshape((c, -1))).reshape((-1, mh, mw))  # [n, mh, mw]
        # ★ 先 sigmoid，把线性叠加的 logits 转成概率
        masks = 1.0 / (1.0 + np.exp(-masks))
        masks = np.transpose(masks, (1, 2, 0))  # -> [mh, mw, n]
        masks = np.ascontiguousarray(masks)
        masks = self.scale_mask(masks, im0_shape)  # -> [H, W, n]
        masks = np.einsum('HWN -> NHW', masks)  # -> [n, H, W]
        a = time.time()        
        masks = self.crop_mask(masks, bboxes)  # 裁剪到框内
        print(time.time()-a)
        return masks > 0.5
    
    @staticmethod
    def scale_mask(masks, im0_shape, ratio_pad=None):
        """
        Takes a mask, and resizes it to the original image size. (Borrowed from
        https://github.com/ultralytics/ultralytics/blob/465df3024f44fa97d4fad9986530d5a13cdabdca/ultralytics/utils/ops.py#L305)
        Args:
        masks (np.ndarray): resized and padded masks/images, [h, w, num]/[h, w, 3].
        im0_shape (tuple): the original image shape.
        ratio_pad (tuple): the ratio of the padding to the original image.
        Returns:
        masks (np.ndarray): The masks that are being returned.
        """
        im1_shape = masks.shape[:2]
        if ratio_pad is None:  # calculate from im0_shape
            gain = min(im1_shape[0] / im0_shape[0], im1_shape[1] / im0_shape[1])  # gain  = old / new
            pad = (im1_shape[1] - im0_shape[1] * gain) / 2, (im1_shape[0] - im0_shape[0] * gain) / 2  # wh padding
        else:
            pad = ratio_pad[1]

        # Calculate tlbr of mask
        top, left = int(round(pad[1] - 0.1)), int(round(pad[0] - 0.1))  # y, x
        bottom, right = int(round(im1_shape[0] - pad[1] + 0.1)), int(round(im1_shape[1] - pad[0] + 0.1))
        if len(masks.shape) < 2:
            raise ValueError(f'"len of masks shape" should be 2 or 3, but got {len(masks.shape)}')
        masks = masks[top:bottom, left:right]
        masks = cv2.resize(masks, (im0_shape[1], im0_shape[0]),
        interpolation=cv2.INTER_LINEAR)  # INTER_CUBIC would be better
        if len(masks.shape) == 2:
            masks = masks[:, :, None]
        return masks
    
    @staticmethod
    def crop_mask(masks, boxes):
        """
        It takes a mask and a bounding box, and returns a mask that is cropped to the bounding box. (Borrowed from
        https://github.com/ultralytics/ultralytics/blob/465df3024f44fa97d4fad9986530d5a13cdabdca/ultralytics/utils/ops.py#L599)
        Args:
        masks (Numpy.ndarray): [n, h, w] tensor of masks.
        boxes (Numpy.ndarray): [n, 4] tensor of bbox coordinates in relative point form.
        Returns:
        (Numpy.ndarray): The masks are being cropped to the bounding box.
        """
        n, h, w = masks.shape  # (n,640,640)
        x1, y1, x2, y2 = np.split(boxes[:, :, None], 4, 1)  # (n,1,1)
        r = np.arange(w, dtype=x1.dtype)[None, None, :]  # (1,1,640)
        c = np.arange(h, dtype=x1.dtype)[None, :, None]  # (1,640,1)
        return masks * ((r >= x1) * (r < x2) * (c >= y1) * (c < y2))  # too time-costly 0.4s
    

if __name__ == '__main__':
    # Create an argument parser to handle command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--seg_model', type=str, default=r"yolov8n-seg_brick.om", help='Path to OM model')
    parser.add_argument('--source', type=str, default=r'images', help='image文件夹')
    parser.add_argument('--out_path', type=str, default=r'results', help='结果保存文件夹')
    parser.add_argument('--imgsz_seg', type=tuple, default=(640, 640), help='Image input size')
    parser.add_argument('--classes', type=list, default=['brick', 'broken_brick', 'crack'], help='类别')
    parser.add_argument('--conf', type=float, default=0.25, help='Confidence threshold')
    parser.add_argument('--iou', type=float, default=0.7, help='NMS IoU threshold')
    parser.add_argument('--device_id', type=int, default=0, help='device id')
    parser.add_argument('--mode', default='static', help='om是动态dymshape或静态static')
    parser.add_argument('--model_ndtype', default=np.single, help='om是fp32或fp16')
    parser.add_argument('--postprocess_type', type=str, default='v8', help='后处理方式, 对应v5/v8两种后处理')
    parser.add_argument('--aipp', default=False, action='store_true', help='是否开启aipp加速YOLO预处理, 需atc中完成om集成')
    args = parser.parse_args()
    
    # 创建结果保存文件夹
    if not os.path.exists(args.out_path):
        os.mkdir(args.out_path)
    print('开始运行：')

    # Build model
    seg_model = YOLO(args.seg_model, args.imgsz_seg, args.device_id, args.model_ndtype, args.mode, args.postprocess_type, args.aipp)
    color_palette = np.random.uniform(0, 255, size=(len(args.classes), 3))  # 为每个类别生成调色板

    for i, img_name in enumerate(os.listdir(args.source)):
        try:
            t1 = time.time()
            
            # Read image by OpenCV
            img = cv2.imread(os.path.join(args.source, img_name))

            # 检测Inference
            boxes, segments, _, (pre_time, det_time, post_time) = seg_model(img, conf_threshold=args.conf, iou_threshold=args.iou)
            print('{}/{} ==>总耗时间: {:.3f}s, 其中, 预处理: {:.3f}s, 推理: {:.3f}s, 后处理: {:.3f}s, 识别{}个目标'.format(i + 1, len(os.listdir(args.source)), time.time() - t1, pre_time, det_time, post_time, len(boxes)))
            
            # Draw rectangles and polygons
            im_canvas = img.copy()

            # 在绘制循环里加健壮性判断
            for (*box, conf, cls_), segment in zip(boxes, segments):
                # segment 可能为空或很短；也可能是 float，需要转 int32
                if segment is None or len(segment) < 3:
                    continue

                seg = np.round(segment).astype(np.int32).reshape(-1, 1, 2)  # -> kx1x2, int32
                cls_i = int(cls_)

                # 先画边，再填充
                cv2.polylines(img, [seg], True, (255, 255, 255), 1)
                cv2.fillPoly(im_canvas, [seg], color_palette[cls_i])
               
                # 画 bbox 和标签
                x1, y1, x2, y2 = map(int, box[:4])
                cv2.rectangle(img, (x1, y1), (x2, y2), color_palette[cls_i], 1, cv2.LINE_AA)
                label = args.classes[cls_i] if 0 <= cls_i < len(args.classes) else f'cls{cls_i}'
                cv2.putText(img, f'{label}: {conf:.3f}', (x1, max(0, y1 - 9)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_palette[cls_i], 2, cv2.LINE_AA)
                
            # Mix image
            img = cv2.addWeighted(img, 0.7, im_canvas, 0.3, 0)
            
            cv2.imwrite(os.path.join(args.out_path, img_name), img)
        except Exception as e:
            print(e)
