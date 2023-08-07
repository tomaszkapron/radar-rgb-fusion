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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare("radar_image_visualizer")
    config_param = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('config_param_file')])

    radar_image_visualizer_node = Node(
            name='radar_image_visualizer_node',
            namespace='',
            package='radar_image_visualizer',
            executable='radar_image_visualizer_node.py',
            parameters=[
                config_param
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
            emulate_tty=True,
            remappings=[
                ('/input_image', '/inference_image')
            ]
    )

    return [radar_image_visualizer_node]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('config_param_file', 'param/defaults.param.yaml')
    add_launch_arg('image_width', '1280')
    add_launch_arg('image_height', '1024')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
