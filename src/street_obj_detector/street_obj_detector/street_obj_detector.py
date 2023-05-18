#!/usr/bin/env python3

# Copyright 2023 tomekkapron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolov8_msgs.msg import Yolov8Inference
from yolov8_msgs.msg import InferenceResult
from typing import Union

class StreetObjDetector:
    def __init__(self) -> None:
        self._param_name = 123
        self.br = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.yolov8_inference = Yolov8Inference()


    def setParameters(self, **kwargs) -> None:
        for arg in kwargs:
            if hasattr(self, "_" + arg):
                setattr(self, "_" + arg, kwargs[arg])

    def printHello(self) -> int:
        print(f"Hello World, {self._param_name}")
        return self._param_name
    
    def detect(self, msg) -> Union[Image, Yolov8Inference]:
        current_frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(current_frame)
        
        annotated_frame = results[0].plot()
        img_msg = self.br.cv2_to_imgmsg(annotated_frame)
        self.unpackYoloToRosMsg(results)

        return img_msg, self.yolov8_inference
    
    def unpackYoloToRosMsg(self, results):
        self.yolov8_inference.yolov8_inference.clear()
        self.yolov8_inference.header.frame_id = "inference"
            
        for r in results:
            boxes = r.boxes
            for box in boxes:
                inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                inference_result.class_name = self.model.names[int(c)]
                inference_result.top = int(b[0])
                inference_result.left = int(b[1])
                inference_result.bottom = int(b[2])
                inference_result.right = int(b[3])
                self.yolov8_inference.yolov8_inference.append(inference_result)