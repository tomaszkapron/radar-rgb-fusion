#!/usr/bin/env python3

# Copyright 2023 tomaszkapron
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

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from radar_msgs.msg import RadarScan


class ScanDetectionMatcher:
    def __init__(self) -> None:
        self.br = CvBridge()
        
    def linear_mapping(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def visualize_scan(self, scan: RadarScan, image: Image, radar_max_angle: float) -> Image:
        image_cv = self.br.imgmsg_to_cv2(image)
        image_with_scan = self.add_visualization(scan, image_cv, radar_max_angle)
        return self.br.cv2_to_imgmsg(image_with_scan)
    
    def add_visualization(self, scan: RadarScan, image: np.ndarray, radar_max_angle: float) -> np.ndarray:
        height, width, _ = image.shape
        strip = self.create_strip(scan, width, radar_max_angle)
        
        image_with_strip = np.concatenate((image, strip), axis=0)
        return image_with_strip
        
    def create_strip(self, scan: RadarScan, width: int, radar_max_angle: float) -> np.ndarray:
        strip = np.zeros((25, width, 3), dtype=np.uint8)

        ranges = scan.returns
        for measure in ranges:
            azimuth = measure.azimuth
            x = int(self.linear_mapping(azimuth, -radar_max_angle, radar_max_angle, 0, width))
            start_x = x - 5
            if start_x < 0:
                start_x = 0
            end_x = x + 5
            if end_x > width:
                end_x = width
            strip[0:25, start_x:end_x] = (0, 255, 0)
            
        return strip
