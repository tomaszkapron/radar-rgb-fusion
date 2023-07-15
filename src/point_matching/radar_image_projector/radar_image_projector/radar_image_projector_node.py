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
    from radar_image_projector.radar_image_projector import RadarImageProjector
except ImportError:
    from radar_image_projector import RadarImageProjector
    
from radar_msgs.msg import RadarScan
from projected_radar_msgs.msg import ProjectedRadarArray


class RadarImageProjectorNode(Node):
    def __init__(self):
        super().__init__("radar_image_projector_node")
        
        camera_fx = self.declare_parameter('camera_matrix.fx', 100.0).value
        camera_fy = self.declare_parameter('camera_matrix.fy', 100.0).value
        camera_cx = self.declare_parameter('camera_matrix.cx', 100.0).value
        camera_cy = self.declare_parameter('camera_matrix.cy', 100.0).value
        
        self.radar_image_projector = RadarImageProjector(camera_fx, camera_fy, camera_cx, camera_cy)
        
        input_radar_topic = self.declare_parameter("input_radar_topic", "input_image").value

        self.create_subscription(
            RadarScan,
            input_radar_topic,
            self.scan_callback,
            1
        )
        
        self.projected_radar_pub_ = self.create_publisher(ProjectedRadarArray, "/projected_radar", 1)
        
    def scan_callback(self, msg: RadarScan):
        projected_radar_array = self.radar_image_projector.project_radar_scan(msg)
        self.projected_radar_pub_.publish(projected_radar_array)
        

def main(args=None):
    rclpy.init(args=args)
    node = RadarImageProjectorNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
