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

import rclpy
from rclpy.node import Node
try:
    from street_obj_detector.street_obj_detector import StreetObjDetector
except ImportError:
    from street_obj_detector import StreetObjDetector

from sensor_msgs.msg import Image
from yolov8_msgs.msg import Yolov8Inference


class StreetObjDetectorNode(Node):
    def __init__(self):
        super().__init__("street_obj_detector_node")
        self.street_obj_detector = StreetObjDetector()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        
        param_name = self.declare_parameter('param_name', 456).value
        self.street_obj_detector.setParameters(param_name=param_name)
        
    def image_callback(self, msg):
        self.get_logger().info('I heard: msg')
        detection_image = self.street_obj_detector.detect(msg)[0]
        detections = self.street_obj_detector.detect(msg)[1]
        detections.header.stamp = self.get_clock().now().to_msg()
        self.img_pub.publish(detection_image)
        self.yolov8_pub.publish(detections)


def main(args=None):
    rclpy.init(args=args)
    node = StreetObjDetectorNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
