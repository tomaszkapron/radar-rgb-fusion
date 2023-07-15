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
from radar_msgs.msg import RadarScan
from projected_radar_msgs.msg import ProjectedRadarArray
from projected_radar_msgs.msg import ProjectedRadar

class RadarImageProjector:
    def __init__(self, fx, fy, cx, cy) -> None:
        self.fx_ = fx
        self.fy_ = fy
        self.cx_ = cx
        self.cy_ = cy
        self.cam_mat_ = np.mat([[self.fx_, 0.0,      self.cx_],
                                [0.0,      self.fy_, self.cy_],
                                [0.0,      0.0,      1.0]])
        
        self.T = np.mat([[1.0, 0.0,        0.0,       0.2 ],
                         [0.0, 0.9981348, -0.0610485, 1.85],
                         [0.0, 0.0610485,  0.9981348, 0.3 ],
                         [0.0, 0.0,        0.0,       1.0 ]])
        
         
    def project_radar_scan(self, msg: RadarScan) -> ProjectedRadarArray:
        projectedRadarArrMsg = ProjectedRadarArray()
        projectedRadarArrMsg.header = msg.header
        
        for measure in msg.returns:
            projectedRadarMsg = ProjectedRadar()
            u, v = self.radar_detection_to_image2(measure.range, measure.azimuth)

            projectedRadarMsg.u = int(round(u))
            projectedRadarMsg.v = int(round(v))
            projectedRadarMsg.range = measure.range
            projectedRadarMsg.velocity = measure.doppler_velocity
            projectedRadarArrMsg.projected_radar_array.append(projectedRadarMsg)
            
        return projectedRadarArrMsg
    
    def radar_detection_to_image2(self, range: float, azimuth: float) -> tuple:     
        Xr = range * np.sin(azimuth * np.pi / 180.0)
        Zr = range * np.cos(azimuth * np.pi / 180.0)
        Yr = 0.0
        
        P_camera = np.dot(self.T, np.transpose(np.mat([Xr, Yr, Zr, 1.0])))
        
        X_norm = P_camera[0, 0] / P_camera[2, 0]
        Y_norm = P_camera[1, 0] / P_camera[2, 0]
        
        u = self.cam_mat_[0, 0] * X_norm + self.cam_mat_[0, 2]
        v = self.cam_mat_[1, 1] * Y_norm + self.cam_mat_[1, 2]

        return (u, v)

