# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from radar_msgs.msg import RadarScan
from sensor_msgs.msg import PointCloud2
import pcl
import numpy as np
from ros2_numpy import point_cloud2
import math


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            RadarScan,
            '/RADAR_FRONT',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(
            PointCloud2,
            "radar_to_pointcloud_tracked",
            10
        )

    def calucalte_xy_points(self, arr):
        points = np.zeros(shape=(len(arr), 3))
        for idx, point in enumerate(arr):
            points[idx] = [round(point.range * math.sin(math.radians(point.azimuth)), 2),
                           round(point.range * math.cos(math.radians(point.azimuth)), 2), 0]
            # print("azimuth: " + str(int(point.azimuth)) + ", Range: " + str(int(point.range)))

        return points

    def add_dummy_points(self, step):
        range_vector = np.arange(0, 80, step)
        points = np.zeros(shape=(len(range_vector) * 2, 3))
        for idx, point in enumerate(range_vector):
            points[idx] = [round(point * math.sin(math.radians(60)), 2),
                           round(point * math.cos(math.radians(60)), 2), 0]

        for idx, point in enumerate(range_vector):
            points[idx + len(range_vector)] = [round(point * math.sin(math.radians(-60)), 2),
                                               round(point * math.cos(math.radians(-60)), 2), 0]

        return points

    def xyz_to_pointcloud(self, arr, rgb=(255, 0, 0), fill_color=False):
        if arr.shape[1] != 3:
            raise ValueError(f"Wrong xyz array shape {arr.shape} instead of (N, 3).")
        if fill_color:
            rgb_f4 = rgb[0] << 16 | rgb[1] << 8 | rgb[2]
            arr = np.hstack([arr, np.full((arr.shape[0], 1), rgb_f4)])
            output_dtype = np.dtype(
                {'names': ['x', 'y', 'z', 'rgb'], 'formats': ['<f4', '<f4', '<f4', '<f4']})
        else:
            output_dtype = np.dtype(
                {'names': ['x', 'y', 'z'], 'formats': ['<f4', '<f4', '<f4']})

        new_points = np.core.records.fromarrays(arr.T, output_dtype)
        return new_points

    def listener_callback(self, msg):
        points = self.calucalte_xy_points(arr=msg.returns)
        dummy_points = self.add_dummy_points(step=0.5)

        pcd = self.xyz_to_pointcloud(arr=points, fill_color=True)
        dummy_pcd = self.xyz_to_pointcloud(arr=dummy_points, rgb=(255, 192, 203), fill_color=True)

        pcd = np.concatenate((pcd, dummy_pcd), axis=0)
        pcd_msg = point_cloud2.array_to_pointcloud2(pcd, stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        self.publisher.publish(pcd_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
