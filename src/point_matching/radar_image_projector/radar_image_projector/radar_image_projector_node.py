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

        print("camera_fx: ", camera_fx)
        print("camera_fy: ", camera_fy)
        print("camera_cx: ", camera_cx)

        T_x = self.declare_parameter('camera_radar_transform.translation.x', 0.0).value
        T_y = self.declare_parameter('camera_radar_transform.translation.y', 0.0).value
        T_z = self.declare_parameter('camera_radar_transform.translation.z', 0.0).value
        R_x = self.declare_parameter('camera_radar_transform.rotation.x', 0.0).value
        R_y = self.declare_parameter('camera_radar_transform.rotation.y', 0.0).value
        R_z = self.declare_parameter('camera_radar_transform.rotation.z', 0.0).value
        R_w = self.declare_parameter('camera_radar_transform.rotation.w', 0.0).value

        camera_matrix = RadarImageProjector.prepare_camera_matrix(camera_fx, camera_fy, camera_cx, camera_cy)
        transform_matrix = RadarImageProjector.prepare_transformation_matrix(T_x, T_y, T_z, R_x, R_y, R_z, R_w)
        
        self.radar_image_projector = RadarImageProjector(camera_matrix, transform_matrix)

        self.create_subscription(
            RadarScan,
            "input_radar_topic",
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
