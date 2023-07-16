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

import rclpy
from rclpy.node import Node
try:
    from radar_detection_matcher.radar_detection_matcher import RadarDetectionMatcher
except ImportError:
    from radar_detection_matcher import RadarDetectionMatcher
    
from yolov8_msgs.msg import Yolov8Inference
from projected_radar_msgs.msg import ProjectedRadarArray
from projected_radar_msgs.msg import RadarDetectionMatchArray

class RadarDetectionMatcherNode(Node):
    def __init__(self):
        super().__init__("radar_detection_matcher_node")
        self.radar_detection_matcher = RadarDetectionMatcher()

        self.create_subscription(Yolov8Inference, "/yolov8_inference", self.yolo_callback, 1)
        self.create_subscription(ProjectedRadarArray, "/projected_radar", self.radar_callback, 1)
        
        self.match_pub_ = self.create_publisher(RadarDetectionMatchArray, "/matched_detections", 1)
        
        
    def yolo_callback(self, msg: Yolov8Inference):
        self.radar_detection_matcher.detections = msg
        
    def radar_callback(self, msg: ProjectedRadarArray):
        self.radar_detection_matcher.radar_projection = msg
        self.radar_detection_matcher.match_detections()
        self.match_pub_.publish(self.radar_detection_matcher.matched_detections)

def main(args=None):
    rclpy.init(args=args)
    node = RadarDetectionMatcherNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
