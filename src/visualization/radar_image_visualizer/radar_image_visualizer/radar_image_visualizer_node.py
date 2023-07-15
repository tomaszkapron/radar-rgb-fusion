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
    from radar_image_visualizer.radar_image_visualizer import RadarImageVisualizer
except ImportError:
    from radar_image_visualizer import RadarImageVisualizer
    
from projected_radar_msgs.msg import ProjectedRadarArray
from projected_radar_msgs.msg import RadarDetectionMatchArray
from sensor_msgs.msg import Image


class RadarImageVisualizerNode(Node):
    def __init__(self):
        super().__init__("radar_image_visualizer_node")
        self.radar_image_visualizer = RadarImageVisualizer()
        
        show_radar = self.declare_parameter("show_radar_projections", True).value # TODO: handle

        self.create_subscription(ProjectedRadarArray, "/projected_radar", self.radar_callback, 1)
        self.create_subscription(RadarDetectionMatchArray, "/radar_detection_match", self.match_callback, 1)
        self.create_subscription(Image, "/input_image", self.image_callback, 1)

        self.visualization_pub = self.create_publisher(Image, "/output_image", 1)

        self.radar_arr = ProjectedRadarArray()
        self.radar_match = RadarDetectionMatchArray()

    def radar_callback(self, msg: ProjectedRadarArray):
        self.radar_arr = msg
        
    def match_callback(self, msg: RadarDetectionMatchArray):
        self.radar_match = msg

    def image_callback(self, msg: Image):
        visualized = self.radar_image_visualizer.visualize_scan(
            msg,
            self.radar_arr,
            self.radar_match
        )
        self.visualization_pub.publish(visualized)

def main(args=None):
    rclpy.init(args=args)
    node = RadarImageVisualizerNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
