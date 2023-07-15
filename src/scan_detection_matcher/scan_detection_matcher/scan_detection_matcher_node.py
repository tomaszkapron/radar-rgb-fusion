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
    from scan_detection_matcher.scan_detection_matcher import ScanDetectionMatcher
except ImportError:
    from scan_detection_matcher import ScanDetectionMatcher
    
from sensor_msgs.msg import Image
from radar_msgs.msg import RadarScan
from yolov8_msgs.msg import Yolov8Inference


class ScanDetectionMatcherNode(Node):
    def __init__(self):
        super().__init__("scan_detection_matcher_node")
        self.scan_detection_matcher = ScanDetectionMatcher()
        input_image_topic = self.declare_parameter("input_image_topic", "input_image").value
        input_radar_topic = self.declare_parameter("input_radar_topic", "input_radar").value
        self.radar_max_angle = self.declare_parameter("radar_max_angle", 60.0).value
        self.radar_scan = RadarScan()
        self.detections = Yolov8Inference()
        
        self.create_subscription(
            RadarScan,
            input_radar_topic,
            self.scan_callback,
            1
        )
        
        self.create_subscription(
            Image,
            input_image_topic,
            self.image_callback,
            1
        )
        
        self.create_subscription(
            Yolov8Inference,
            "yolov8_inference",
            self.yolo_callback,
            1
        )
        
        self.visualization_pub = self.create_publisher(Image, "output_image", 1)
            
            
    def scan_callback(self, msg: RadarScan):
        self.scan_detection_matcher.radar_scan = msg
    
    def image_callback(self, msg: Image):
        visualized = self.scan_detection_matcher.visualize_scan(
            msg,
            self.radar_max_angle
        )
        self.visualization_pub.publish(visualized)
        
    def yolo_callback(self, msg: Yolov8Inference):
        self.scan_detection_matcher.yolo_detections = msg
    
def main(args=None):
    rclpy.init(args=args)
    node = ScanDetectionMatcherNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
