import cv2
import numpy as np
import onnxruntime as ort
try:
    import spacemit_ort  # noqa: F401
    HAS_SPACEMIT_ORT = True
except ImportError:
    spacemit_ort = None
    HAS_SPACEMIT_ORT = False
import threading
import queue

class AGVDetection:
    def __init__(self, model_path, label_path):
        self.model_path = model_path
        self.class_conf = 0.3
        self.nms_thresh = 0.45
        self.labels = [line.strip() for line in open(label_path, 'r').readlines()]
        self.infer_session= self.init_infer_session()
        self.warm_up_times = 1
        self.input_name = self.infer_session.get_inputs()[0].name
        self.output_name = self.infer_session.get_outputs()[0].name
        self.input_size = self.infer_session.get_inputs()[0].shape[2:4]


    def init_infer_session(self):
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 2

        # 加载 ONNX 模型
        providers = ["CPUExecutionProvider"]
        if HAS_SPACEMIT_ORT:
            try:
                available_providers = ort.get_available_providers()
                if "SpaceMITExecutionProvider" in available_providers:
                    providers = ["SpaceMITExecutionProvider", "CPUExecutionProvider"]
            except Exception:
                providers = ["CPUExecutionProvider"]

        try:
            session = ort.InferenceSession(
                self.model_path,
                sess_options=session_options,
                providers=providers,
            )
        except Exception:
            session = ort.InferenceSession(
                self.model_path,
                sess_options=session_options,
                providers=["CPUExecutionProvider"],
            )

        return session

    def warm_up(self):
        warm_up_img = np.random.rand(1,3, self.input_size[0], self.input_size[1]).astype(np.float32)

        for i in range(self.warm_up_times):
            self.infer_session.run([self.output_name], {self.input_name: warm_up_img})

    # 追踪的检测接口
    def infer_track(self,image):
        img = image.copy()

        # 图像预处理
        input_tensor = self.preprocess(img,self.input_size)
        # 进行推理
        outputs = self.infer_session.run([self.output_name], {self.input_name: input_tensor})
        output = outputs[0]
        offset = output.shape[1]
        anchors = output.shape[2]

        # 后处理
        dets = self.postprocess(image, output, anchors, offset, self.class_conf,self.input_size)
        dets = self.nms(dets)

        dets_np = np.array(dets)
        filtered_dets = dets_np[dets_np[:, 4] == 0]
        new_dets = np.delete(filtered_dets, 4, axis=1)


        return new_dets

    def infer(self,image):
        img = image.copy()

        # 图像预处理
        input_tensor = self.preprocess(img,self.input_size)
        # 进行推理
        outputs = self.infer_session.run([self.output_name], {self.input_name: input_tensor})
        output = outputs[0]
        offset = output.shape[1]
        anchors = output.shape[2]

        # 后处理
        dets = self.postprocess(image, output, anchors, offset, self.class_conf,self.input_size)
        dets = self.nms(dets)

        # 保留第五个元素（索引为 4）为 0 的成员
        filtered_dets = [item for item in dets if item[4] == 0]
        # 去除第5个维度和第6个维度
        new_dets = [[val for i, val in enumerate(item) if i not in [4, 5]] for item in filtered_dets]

        return new_dets


    def preprocess(self, image, input_size=(320, 320)):
        shape = image.shape[:2]
        pad_color=(0,0,0)
        #调整图像大小
        # Scale ratio
        r = min(input_size[0] / shape[0], input_size[1] / shape[1])
        # Compute padding
        ratio = r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = input_size[1] - new_unpad[0], input_size[0] - new_unpad[1]  # wh padding
        dw /= 2  # divide padding into 2 sides
        dh /= 2
        if shape[::-1] != new_unpad:  # resize
            image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=pad_color)  # add border

        # 归一化处理
        # image = image.astype(np.float32) / 255.0
        image = cv2.normalize(image, None, 0.0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        # 调整维度以匹配模型输入 [batch, channel, height, width]
        image = np.transpose(image, (2, 0, 1))
        image = np.expand_dims(image, axis=0)

        return image

    def postprocess(self,image,output, anchors, offset, conf_threshold,input_size=(320,320)):
        # 获取图像的高和宽
        shape = image.shape[:2]
        # 计算缩放比例
        r = min(input_size[0] / shape[0], input_size[1] / shape[1])
        # 计算新的未填充尺寸
        new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
        # 计算填充量
        dw, dh = input_size[1] - new_unpad[0], input_size[0] - new_unpad[1]
        # 将填充量平分到两侧
        dw /= 2
        dh /= 2

        # 去除 output 多余的维度
        output = output.squeeze()

        # 提取每个锚点对应的边界框信息（中心坐标、宽高）
        center_x = output[0, :anchors]
        center_y = output[1, :anchors]
        box_width = output[2, :anchors]
        box_height = output[3, :anchors]

        # 提取每个锚点对应的所有类别概率
        class_probs = output[4:offset, :anchors]

        # 找出每个锚点下概率最大的类别索引及其概率值
        max_prob_indices = np.argmax(class_probs, axis=0)
        max_probs = class_probs[max_prob_indices, np.arange(anchors)]

        # 过滤掉置信度低于阈值的锚点
        valid_mask = max_probs > conf_threshold
        valid_center_x = center_x[valid_mask]
        valid_center_y = center_y[valid_mask]
        valid_box_width = box_width[valid_mask]
        valid_box_height = box_height[valid_mask]
        valid_max_prob_indices = max_prob_indices[valid_mask]
        valid_max_probs = max_probs[valid_mask]

        # 计算边界框坐标
        half_width = valid_box_width / 2
        half_height = valid_box_height / 2
        x1 = np.maximum(0, ((valid_center_x - half_width) - dw) / r).astype(int)
        x2 = np.maximum(0, ((valid_center_x + half_width) - dw) / r).astype(int)
        y1 = np.maximum(0, ((valid_center_y - half_height) - dh) / r).astype(int)
        y2 = np.maximum(0, ((valid_center_y + half_height) - dh) / r).astype(int)

        # 组合结果
        objects = np.column_stack((x1, y1, x2, y2, valid_max_prob_indices, valid_max_probs)).tolist()

        return objects

    def nms(self,dets):
        if len(dets) == 0:
            return np.empty((0, 6))

        dets_array = np.array(dets)
        # 按类别分组
        unique_labels = np.unique(dets_array[:, 4])
        final_dets = []

        for label in unique_labels:
            # 获取当前类别的检测结果
            mask = dets_array[:, 4] == label
            dets_class = dets_array[mask]

            # 按置信度从高到低排序
            order = np.argsort(-dets_class[:, 5])
            dets_class = dets_class[order]

            # 逐个进行 NMS
            keep = []
            while dets_class.shape[0] > 0:
                # 保留当前置信度最高的检测结果
                keep.append(dets_class[0])
                if dets_class.shape[0] == 1:
                    break

                # 计算当前框与其他框的 IoU
                ious = self.calculate_iou(keep[-1], dets_class[1:])
                # 去除 IoU 大于阈值的框
                dets_class = dets_class[1:][ious < self.nms_thresh]

            # 将当前类别的结果添加到最终结果中
            final_dets.extend(keep)

        return final_dets


    def calculate_iou(self,box, boxes):
        """
        计算一个框与一组框的 IoU
        :param box: 单个框 [x1, y1, x2, y2]
        :param boxes: 一组框 [N, 4]
        :return: IoU 值 [N]
        """
        # 计算交集区域
        x1 = np.maximum(box[0], boxes[:, 0])
        y1 = np.maximum(box[1], boxes[:, 1])
        x2 = np.minimum(box[2], boxes[:, 2])
        y2 = np.minimum(box[3], boxes[:, 3])
        inter_area = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)

        # 计算并集区域
        box_area = (box[2] - box[0]) * (box[3] - box[1])
        boxes_area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        union_area = box_area + boxes_area - inter_area

        # 计算 IoU
        return inter_area / union_area



    # 可视化结果
    def draw_result(self,image,outputs,tid,results):
        img = image.copy()

        x1,y1,x2,y2 = outputs[0][0:4]
        # 绘制矩形框
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        # 创建文本字符串：标签和置信度

        text = f"id{tid},{results[2][0]}: {results[3][0]:.2f}"

        # 获取文本尺寸
        ((text_width, text_height), _) = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

        # 计算文本框的坐标，并确保它不会超出图像边界
        text_offset_x = int(max(x1, 0))
        text_offset_y = int(min(y1- 5, image.shape[0] - text_height - 5) )

        # 显示文本
        cv2.putText(img, text, (text_offset_x, text_offset_y + int(1.3 * text_height)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), thickness=1)

        return img


    def convert_rect_list(self,original_list):
        converted_list = []
        for x1, y1, x2, y2, label, prob in original_list:
            width = x2 - x1
            height = y2 - y1
            new_rect = ((x1, y1), width, height, label, prob)
            converted_list.append(new_rect)
        return converted_list