# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
import argparse
import os
import sys
from pathlib import Path

import numpy as np
import torch
import torch.backends.cudnn as cudnn

from models.common import DetectMultiBackend
from utils.general import (check_img_size,
                           non_max_suppression, scale_coords)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox

import time

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative


class YoloV5Detect:

    def __init__(self):
        self.weights = ROOT / 'weights/best.pt'
        self.imgsz = (640, 640)
        self.device = select_device('0')
        self.half = False
        self.conf_thres = 0.95  # confidence threshold
        self.iou_thres = 0.45  # NMS IOU threshold
        self.max_det = 1000  # maximum detections per image
        self.classes = None,  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False,  # class-agnostic NMS
        self.augment=False,  # augmented inference
        self.visualize=False,  # visualize features
        self.line_thickness=3,  # bounding box thickness (pixels)
        self.hide_labels=False,  # hide labels
        self.hide_conf=False,  # hide confidences
        self.model = DetectMultiBackend(self.weights, device=self.device, dnn=False, data=False)
        self.names = self.model.names
        self.model.model.half() if self.half else self.model.model.float()
        cudnn.benchmark = True

    @torch.no_grad()
    def run(self, image):
        im = letterbox(image, self.imgsz[0], 32, True)[0]
        im = im.transpose((2, 0, 1))[::-1]
        im = np.ascontiguousarray(im)
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()
        im /= 255 # 0 - 255 to 0.0 - 1.0   torch.Size([3, 480, 640])
        if len(im.shape) == 3:
            im = im[None]  # torch.Size([1, 3, 480, 640])
        # Run inference
        pred = self.model(im, augment=self.augment, visualize=self.visualize)
        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        boxes, scores, classes = [], [], []
        for i, det in enumerate(pred):
            annotator = Annotator(image, line_width=self.line_thickness, example=str(self.names))
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], image.shape).round()
            for *xyxy, conf, cls in reversed(det):
                c = int(cls)  # integer class
                label = None if self.hide_labels[0] else (self.names[c] if self.hide_conf[0] else f'{self.names[c]} {conf:.2f}')
                annotator.box_label(xyxy, label, color=colors(c, True))
                xyxy[0] = xyxy[0].cpu()
                xyxy[1] = xyxy[1].cpu()
                xyxy[2] = xyxy[2].cpu()
                xyxy[3] = xyxy[3].cpu()
                boxes.append(xyxy)
                scores.append(conf.tolist())
                classes.append(cls.tolist())
        return boxes, scores, classes, image



