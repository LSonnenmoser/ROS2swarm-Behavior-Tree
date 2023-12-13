# Copyright 2023 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import pathlib
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('robots', default_value='1',
                          description='number of robots'),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='ignition',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

robots = LaunchConfiguration('robots', default='1')

def generate_launch_description():
    # Directories

    # Paths
    ignition_launch = PathJoinSubstitution(
        [str(pathlib.Path(__file__).parent.resolve()), 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [str(pathlib.Path(__file__).parent.resolve()), 'turtlebot4_spawn.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)

    for i in range(1):
        robot_spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_spawn_launch]),
            launch_arguments=[
                ('namespace', 'robot'+str(i+1)),
                ('rviz', LaunchConfiguration('rviz')),
                ('x', LaunchConfiguration('x')),
                ('y', LaunchConfiguration('y')),
                ('z', LaunchConfiguration('z')),
                ('yaw', LaunchConfiguration('yaw'))]
        )   
        ld.add_action(robot_spawn) 


    return ld
