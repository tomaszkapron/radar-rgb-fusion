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


class RadarDetectionMatcherNode(Node):
    def __init__(self):
        super().__init__("radar_detection_matcher_node")
        self.radar_detection_matcher = RadarDetectionMatcher()



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
