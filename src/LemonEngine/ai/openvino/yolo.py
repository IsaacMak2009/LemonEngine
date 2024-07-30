from LemonEngine.ai.basemodel import BaseOpenvinoModel
from ultralytics.utils import ops

import torch
import numpy as np
import cv2

from typing import *


class YoloDetect(BaseOpenvinoModel):
    def __init__(self,
                 model_name: str = "yolov8n",
                 device_name: str = "CPU",
                 model_dir: Optional[str] = None,
                 n_classes: int = 80,
                 min_conf: float = 0.25,
                 min_iou: float = 0.7,
                 ) -> None:

        super().__init__(model_name=model_name, models_dir=model_dir, device_name=device_name)
        self.n_output = len(self.model.outputs)
        self.n_classes = n_classes
        self.min_conf = min_conf
        self.min_iou = min_iou

    def predict(self, frame: np.ndarray) -> torch.Tensor:
        """
        The predict method for Yolov8, output format:
        tensor([[x1, y1, x2, y2, conf, label_id], # first bounding box
                [x1, y1, x2, y2, conf, label_id], # second bounding box,
                ...])

        :param frame:
        :return: the predicted results, tensor
        """
        # preprocess
        image = frame.copy()
        input_tensor = self.preprocess(image)

        # predict
        result = self.model([input_tensor])
        boxes = result[self.model.output(0)]
        input_hw = input_tensor.shape[2:]

        detections = self.postprocess(pred_boxes=boxes,
                                      input_hw=(input_hw[0], input_hw[1]),
                                      orig_img=image,
                                      min_conf_threshold=self.min_conf,
                                      nms_iou_threshold=self.min_iou)

        # {"det": [[x1, y1, x2, y2, score, label_id], ...]}
        return detections[0]["det"]

    @staticmethod
    def preprocess(img: np.ndarray) -> np.ndarray:
        image = img.copy()
        image = YoloDetect.letterbox(image)[0]
        image = image.transpose(2, 0, 1)  # Convert HWC to CHW
        image = np.ascontiguousarray(image)

        # convert to tensor
        input_tensor = image.astype(np.float32)  # uint8 to fp32
        input_tensor /= 255.0  # 0 - 255 to 0.0 - 1.0
        return np.expand_dims(input_tensor, 0)

    """
    Using the functions from https://docs.openvino.ai/2022.3/notebooks/230-yolov8-optimization-with-output.html
    """

    @staticmethod
    def letterbox(img: np.ndarray,
                  new_shape: Tuple[int, int] = (640, 640),
                  color: Tuple[int, int, int] = (114, 114, 114),
                  auto: bool = False,
                  scale_fill: bool = False,
                  scaleup: bool = False,
                  stride: int = 32):
        """
        Resize image and padding for detection. Takes image as input,
        resizes image to fit into new shape with saving original aspect ratio and pads it to meet stride-multiple constraints

        Parameters:
          img (np.ndarray): image for preprocessing
          new_shape (Tuple(int, int)): image size after preprocessing in format [height, width]
          color (Tuple(int, int, int)): color for filling padded area
          auto (bool): use dynamic input size, only padding for stride constrins applied
          scale_fill (bool): scale image to fill new_shape
          scaleup (bool): allow scale image if it is lower then desired input size, can affect model accuracy
          stride (int): input padding stride
        Returns:
          img (np.ndarray): image after preprocessing
          ratio (Tuple(float, float)): hight and width scaling ratio
          padding_size (Tuple(int, int)): height and width padding size


        """
        # Resize and pad image while meeting stride-multiple constraints
        shape = img.shape[:2]  # current shape [height, width]

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better test mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
        elif scale_fill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return img, ratio, (dw, dh)

    @staticmethod
    def postprocess(
            pred_boxes: np.ndarray,
            input_hw: Tuple[int, int],
            orig_img: np.ndarray,
            min_conf_threshold: float = 0.25,
            nms_iou_threshold: float = 0.7,
            agnostic_nms: bool = False,
            max_detections: int = 300):
        """
        YOLOv8 model postprocessing function. Applied non-maximum suppression algorithm to detections and rescale boxes
        to original image size
        Parameters:
            pred_boxes (np.ndarray): model output prediction boxes
            input_hw (np.ndarray): preprocessed image
            orig_img (np.ndarray): image before preprocessing
            min_conf_threshold (float, *optional*, 0.25): minimal accepted confidence for object filtering
            nms_iou_threshold (float, *optional*, 0.45): minimal overlap score for removing objects duplicates in NMS
            agnostic_nms (bool, *optional*, False): apply class agnostic NMS approach or not
            max_detections (int, *optional*, 300):  maximum detections after NMS
        Returns:
           pred (List[Dict[str, np.ndarray]]): list of dictionary with det - detected boxes in format [x1, y1, x2, y2, score, label]
        """
        predicts = ops.non_max_suppression(
            torch.from_numpy(pred_boxes),
            min_conf_threshold,
            nms_iou_threshold,
            nc=80,
            agnostic=agnostic_nms,
            max_det=max_detections
        )

        results = []
        for i, pred in enumerate(predicts):
            shape = orig_img[i].shape if isinstance(orig_img, list) else orig_img.shape
            if not len(pred):
                results.append(
                    {"det": torch.tensor([])})  # DIFF: replace [] with tensor([]) to maintain output consistency
                continue
            pred[:, :4] = ops.scale_boxes(input_hw, pred[:, :4], shape).round()
            results.append({"det": pred})

        return results


class YoloSegment(BaseOpenvinoModel):
    pass


class YoloPose(BaseOpenvinoModel):
    pass
