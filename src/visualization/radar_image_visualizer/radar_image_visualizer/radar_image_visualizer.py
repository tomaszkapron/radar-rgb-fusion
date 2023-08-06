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

from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from projected_radar_msgs.msg import ProjectedRadarArray
from projected_radar_msgs.msg import RadarDetectionMatchArray

class RadarImageVisualizer:
    def __init__(self, width, height) -> None:
        self.br = CvBridge()
        self.dot_size = 5
        self.width = width
        self.height = height

    def visualize_scan(self, image, radar_arr, radar_match):
        image_cv = self.br.imgmsg_to_cv2(image)
        self.add_radar_projections(image_cv, radar_arr)
        self.add_radar_matches(image_cv, radar_match)
        return self.br.cv2_to_imgmsg(cv2.cvtColor(image_cv, cv2.COLOR_BGR2RGB))
    
    def add_radar_projections(self, image_cv: Image, radar_arr: ProjectedRadarArray):
        for projection in radar_arr.projected_radar_array:
            if 0 + self.dot_size <= projection.u < self.width - self.dot_size and \
               0 + self.dot_size <= projection.v < self.height - self.dot_size:
                cv2.circle(image_cv, (int(projection.u), int(projection.v)), self.dot_size, (0, 255, 0), -1)
                
    def add_radar_matches(self, image_cv: Image, radar_match: RadarDetectionMatchArray):
        for match in radar_match.radar_match_array:
            cv2.putText(image_cv, 
                        f"r={round(match.range, 2)}, v={round(match.velocity, 2)}", 
                        (int(round(match.yolo_detection.left)), int(round(match.yolo_detection.top)) + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.85, (174, 0, 0), 4)
    
