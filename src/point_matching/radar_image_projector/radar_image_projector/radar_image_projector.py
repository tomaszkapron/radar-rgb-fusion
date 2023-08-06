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
    def __init__(self, cam_mat, tran_matrix) -> None:
        self.cam_mat_ = cam_mat
        self.T = tran_matrix
         
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
        Xr = range * np.sin(azimuth ) #* np.pi / 180.0)
        Zr = range * np.cos(azimuth ) #* np.pi / 180.0)
        Yr = 0.0
        
        P_camera = np.dot(self.T, np.transpose(np.mat([Xr, Yr, Zr, 1.0])))
        
        X_norm = P_camera[0, 0] / P_camera[2, 0]
        Y_norm = P_camera[1, 0] / P_camera[2, 0]
        
        u = self.cam_mat_[0, 0] * X_norm + self.cam_mat_[0, 2]
        v = self.cam_mat_[1, 1] * Y_norm + self.cam_mat_[1, 2]

        return (u, v)

    def prepare_camera_matrix(fx, fy, cx, cy):
        cam_mat_ = np.mat([[fx,  0.0, cx ],
                           [0.0, fy,  cy ],
                           [0.0, 0.0, 1.0]])
        return cam_mat_
    
    def prepare_transformation_matrix(Tx, Ty, Tz, Rx, Ry, Rz, Rw):
        def quaternion_rotation_matrix(Q):
            q0 = Q[0]
            q1 = Q[1]
            q2 = Q[2]
            q3 = Q[3]

            r00 = 2 * (q0 * q0 + q1 * q1) - 1
            r01 = 2 * (q1 * q2 - q0 * q3)
            r02 = 2 * (q1 * q3 + q0 * q2)

            r10 = 2 * (q1 * q2 + q0 * q3)
            r11 = 2 * (q0 * q0 + q2 * q2) - 1
            r12 = 2 * (q2 * q3 - q0 * q1)

            r20 = 2 * (q1 * q3 - q0 * q2)
            r21 = 2 * (q2 * q3 + q0 * q1)
            r22 = 2 * (q0 * q0 + q3 * q3) - 1

            rot_matrix = np.array([[r00, r01, r02],
                                [r10, r11, r12],
                                [r20, r21, r22]])
                                
            return rot_matrix
        
        R = quaternion_rotation_matrix([Rx, Ry, Rz, Rw])
        T = np.mat([[R[0,0], R[0,1], R[0,2], Tx],
                    [R[1,0], R[1,1], R[1,2], Ty],
                    [R[2,0], R[2,1], R[2,2], Tz],
                    [0.0, 0.0, 0.0, 1.0]])
        return T
