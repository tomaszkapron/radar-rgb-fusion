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
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from radar_msgs.msg import RadarScan

CAM_MAT = np.mat([[699.4761, 0.0,        644.753403],
                  [0.0,      697.332886, 509.920041],
                  [0.0,      0.0,        1.0]])

T = np.mat([[1.0, 0.0, 0.0, 0.2],
            [0.0, 1.0, 0.0, 1.85],
            [0.0, 0.0, 1.0, 0.3],
            [0.0, 0.0, 0.0, 1.0]])
#  0.0174524,  0.9998477;
#    0.0000000, -0.9998477,  0.0174524
# T = np.mat([[1.0, 0.0, 0.0, 1.85],
#             [0.0, 0.0174524, 0.9998477, 0.0],
#             [0.0, -0.9998477, 0.0174524, 0.3],
#             [0.0, 0.0, 0.0, 1.0]])

HEIGHT = 1024
WIDTH = 1280


class ScanDetectionMatcher:
    def __init__(self) -> None:
        self.camera_identification()
        self.br = CvBridge()
        
    def linear_mapping(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def visualize_scan(self, scan: RadarScan, image: Image, radar_max_angle: float) -> Image:
        image_cv = self.br.imgmsg_to_cv2(image)
        image_with_scan = self.add_visualization(scan, image_cv, radar_max_angle)
        return self.br.cv2_to_imgmsg(image_with_scan)
    
    def add_visualization(self, scan: RadarScan, image: np.ndarray, radar_max_angle: float) -> np.ndarray:
        height, width, _ = image.shape
        self.add_radar_to_image(scan, image)
        # strip = self.create_strip(scan, width, radar_max_angle)
        
        # image_with_strip = np.concatenate((image, strip), axis=0)
        return image
        
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

    def radar_detection_to_image(self, range: float, azimuth: float) -> tuple:     
        Xr = range * np.sin(azimuth * np.pi / 180.0)
        Yr = range * np.cos(azimuth * np.pi / 180.0)
        Zr = 0.0
        
        UV  = np.dot(np.dot(CAM_MAT, T), np.transpose(np.mat([Xr, Yr, Zr, 1.0])))
        
        u = UV[0, 0]
        v = UV[1, 0]
        
        print('range: ', range, 'azimuth[deg]: ', azimuth ,'--> u: ', u, 'v: ', v)
        return (u, v)
    
    def radar_detection_to_image2(self, range: float, azimuth: float) -> tuple:     
        Xr = range * np.sin(azimuth * np.pi / 180.0)
        Zr = range * np.cos(azimuth * np.pi / 180.0)
        Yr = 0.0
        
        P_camera = np.dot(T, np.transpose(np.mat([Xr, Yr, Zr, 1.0])))
        
        X_norm = P_camera[0, 0] / P_camera[2, 0]
        Y_norm = P_camera[1, 0] / P_camera[2, 0]
        
        u = CAM_MAT[0, 0] * X_norm + CAM_MAT[0, 2]
        v = CAM_MAT[1, 1] * Y_norm + CAM_MAT[1, 2]
        
        print('range: ', range, 'azimuth[deg]: ', azimuth ,'--> u: ', u, 'v: ', v)
        return (u, v)
        
        
    def camera_identification(self):
        # Prepare
        w, h = 1280, 1024
        fx, fy = 699.4761, 697.332886

        # Go
        fov_x = np.rad2deg(2 * np.arctan2(w, 2 * fx))
        fov_y = np.rad2deg(2 * np.arctan2(h, 2 * fy))

        print("Field of View (degrees):")
        print(f"  {fov_x = :.1f}\N{DEGREE SIGN}")
        print(f"  {fov_y = :.1f}\N{DEGREE SIGN}")
        
    def add_radar_to_image(self, scan: RadarScan, image: np.ndarray) -> np.ndarray:
        dot_size = 5
        for measure in scan.returns:
            azimuth = measure.azimuth
            range = measure.range
            u, v = self.radar_detection_to_image2(range, azimuth)
            if 0 + dot_size <= u < WIDTH - dot_size and 0 + dot_size <= v < HEIGHT - dot_size:
                cv2.circle(image, (int(round(u)), int(round(v))), 5, (0, 255, 0), -1)
