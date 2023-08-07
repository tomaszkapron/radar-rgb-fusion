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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare("devel_launch")

    config_rviz = PathJoinSubstitution(
        [pkg_prefix, 'rviz', LaunchConfiguration('rosbag_source').perform(context) + '.rviz'])

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(config_rviz.perform(context))],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    rosbag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            "--loop",   
            "/home/tomek/mgr_ws/rosbag2_2023_04_12-17_50_39",
        ],
        output="screen",
    )

    street_obj_detector_prefix = FindPackageShare("street_obj_detector")
    street_obj_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [street_obj_detector_prefix, 'launch', 'street_obj_detector.launch.py']
            ),
        ),
        launch_arguments={
            'input_image_topic': LaunchConfiguration('input_image_topic')
        }.items()
    )

    radar_image_projector_prefix = FindPackageShare("radar_image_projector")
    radar_image_projector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [radar_image_projector_prefix, 'launch', 'radar_image_projector.launch.py']
            ),
        ),
        launch_arguments={
            'input_radar_topic': LaunchConfiguration('input_radar_topic'),
            'config_param_file_projector': LaunchConfiguration('rosbag_source').perform(context)
        }.items()
    )

    radar_detection_matcher_prefix = FindPackageShare("radar_detection_matcher")
    radar_detection_matcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [radar_detection_matcher_prefix, 'launch', 'radar_detection_matcher.launch.py']
            ),
        )
    )

    radar_image_visualizer_prefix = FindPackageShare("radar_image_visualizer")
    radar_image_visualizer_projector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution(
                [radar_image_visualizer_prefix, 'launch', 'radar_image_visualizer.launch.py']
            ),
        ),
        launch_arguments={
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height')
        }.items()
    )

    return [
        rviz2,
        # rosbag,
        street_obj_detector,
        radar_image_projector,
        radar_detection_matcher,
        radar_image_visualizer_projector
        ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('with_rviz', 'True')

    add_launch_arg('rosbag_source', 'nuscenes') # nuscenes, local
    add_launch_arg('input_image_topic', '/CAM_FRONT/image_rect_compressed') #  '/CAM_FRONT_LEFT/image_rect_compressed' ,'/camera/image_raw'
    add_launch_arg('input_radar_topic', '/RADAR_FRONT') # '/RADAR_FRONT', '/radar/raw_points_T79'
    add_launch_arg('image_width', '1600')
    add_launch_arg('image_height', '900')

    # add_launch_arg('rosbag_source', 'local')
    # add_launch_arg('input_image_topic', '/camera/image_raw')
    # add_launch_arg('input_radar_topic', '/radar/raw_points_T79')
    # add_launch_arg('image_width', '1280')
    # add_launch_arg('image_height', '1024')


    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
