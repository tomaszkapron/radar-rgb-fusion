# Copyright 2023 tomekkapron
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare("street_obj_detector")
    config_param = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('config_param_file_detector')])

    street_obj_detector_node = Node(
            name='street_obj_detector_node',
            namespace='',
            package='street_obj_detector',
            executable='street_obj_detector_node',
            parameters=[
                config_param
            ],
            remappings=[
                ('~/input_image', LaunchConfiguration('input_image_topic'))
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
            emulate_tty=True,
    )

    return [street_obj_detector_node]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
            DeclareLaunchArgument(
                'config_param_file_detector',
                default_value='param/defaults.param.yaml',
                description='Node config (relative path).'
            )
    )


    declared_arguments.append(
        DeclareLaunchArgument(
            'input_image_topic',
            default_value='/camera/image_raw'
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
