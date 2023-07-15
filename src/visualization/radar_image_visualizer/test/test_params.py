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


import pytest
from radar_image_visualizer.radar_image_visualizer import RadarImageVisualizer


@pytest.mark.parametrize("test_input, expected", [
    (999, 123)
])
def test_default_param(test_input, expected):
    radar_image_visualizer = RadarImageVisualizer()
    assert radar_image_visualizer.printHello() == expected, "Wrong default value"


@pytest.mark.parametrize("test_input, expected", [
    (456, 456)
])
def test_custom_param(test_input, expected):
    radar_image_visualizer = RadarImageVisualizer()
    radar_image_visualizer.setParameters(param_name=test_input)
    assert radar_image_visualizer.printHello() == expected, "Wrong value after parametrization"
