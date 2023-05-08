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

class StreetObjDetector:
    def __init__(self) -> None:
        self._param_name = 123
        self.br = CvBridge()
        self.model = YOLO('yolov8n.pt')


    def setParameters(self, **kwargs) -> None:
        for arg in kwargs:
            if hasattr(self, "_" + arg):
                setattr(self, "_" + arg, kwargs[arg])

    def printHello(self) -> int:
        print(f"Hello World, {self._param_name}")
        return self._param_name
    
    def detect(self, msg) -> Image:
        current_frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
        # results = self.model('/home/tomek/mgr_ws/src/street_obj_detector/pic.jpg')
        results = self.model(current_frame)
        return results
    
