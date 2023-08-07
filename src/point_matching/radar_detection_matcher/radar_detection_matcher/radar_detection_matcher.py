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

from projected_radar_msgs.msg import RadarDetectionMatchArray
from projected_radar_msgs.msg import RadarDetectionMatch
from projected_radar_msgs.msg import ProjectedRadarArray

import numpy as np
from sklearn.cluster import DBSCAN

from yolov8_msgs.msg import Yolov8Inference

class RadarDetectionMatcher:
    def __init__(self) -> None:
        self.detections = Yolov8Inference()
        self.radar_projection = ProjectedRadarArray()
        self.matched_detections = RadarDetectionMatchArray()

        epsilon = 0.5
        min_samples = 1

        self.dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
        
    def match_detections(self):
        self.matched_detections.header = self.radar_projection.header
        self.matched_detections.radar_match_array = []
        for detection in self.detections.yolov8_inference:
            for projected_scan in self.radar_projection.projected_radar_array:
                if detection.left <= projected_scan.u <= detection.right and \
                   detection.top <= projected_scan.v <= detection.bottom:
                       
                    match = RadarDetectionMatch()
                    match.yolo_detection = detection
                    match.range = projected_scan.range
                    match.velocity = projected_scan.velocity
                    self.matched_detections.radar_match_array.append(match)
                    break

    def match_detections_statistically(self):
        self.matched_detections.header = self.radar_projection.header
        self.matched_detections.radar_match_array = []

        scans_in_box = []

        for detection in self.detections.yolov8_inference:
            for projected_scan in self.radar_projection.projected_radar_array:
                if detection.left <= projected_scan.u <= detection.right and \
                   detection.top <= projected_scan.v <= detection.bottom: 

                    scans_in_box.append(self.projected_scan_to_array(projected_scan))
            
            if len(scans_in_box) == 0:
                continue
            
            scans_in_box_np = np.array(scans_in_box)

            clusters = self.dbscan.fit(scans_in_box_np)
            cluster_labels = clusters.labels_
            unique_clusters, cluster_counts = np.unique(cluster_labels, return_counts=True)

            cluster_counts = cluster_counts[1:]
            unique_clusters = unique_clusters[1:]

            if len(unique_clusters) == 0:
                match = RadarDetectionMatch()
                match.yolo_detection = detection
                match.range = scans_in_box_np[0][0]
                match.velocity = scans_in_box_np[0][1]
                self.matched_detections.radar_match_array.append(match)
                continue

            cluster_with_most_points = unique_clusters[np.argmax(cluster_counts)]
            cluster_points = scans_in_box_np[cluster_labels == cluster_with_most_points]

            mean_range = np.mean(cluster_points[:, 0])
            mean_velocity = np.mean(cluster_points[:, 1])

            match = RadarDetectionMatch()
            match.yolo_detection = detection
            match.range = mean_range
            match.velocity = mean_velocity
            self.matched_detections.radar_match_array.append(match)

            scans_in_box = []

        
    def projected_scan_to_array(self, scan):
        return np.array([scan.range, scan.velocity])
            

