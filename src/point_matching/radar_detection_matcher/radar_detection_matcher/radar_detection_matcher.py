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

from projected_radar_msgs.msg import RadarDetectionMatchArray
from projected_radar_msgs.msg import RadarDetectionMatch
from projected_radar_msgs.msg import ProjectedRadarArray

from yolov8_msgs.msg import Yolov8Inference

class RadarDetectionMatcher:
    def __init__(self) -> None:
        self.detections = Yolov8Inference()
        self.radar_projection = ProjectedRadarArray()
        self.matched_detections = RadarDetectionMatchArray()
        
    def match_detections(self):
        self.matched_detections.header = self.radar_projection.header
        self.matched_detections.radar_match_array = []
        for detection in self.detections.yolov8_inference:
            for projected_scan in self.radar_projection.projected_radar_array:
                if detection.left <= projected_scan.u <= detection.right and \
                   detection.top <= projected_scan.v <= detection.bottom:
                       
                    match = RadarDetectionMatch()
                    match.yolo_detection = detection
                    match.range = projected_scan.range
                    match.velocity = projected_scan.velocity
                    self.matched_detections.radar_match_array.append(match)
                    break
