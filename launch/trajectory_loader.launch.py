# Copyright 2024 The Autoware Foundation
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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration('trajectory_loader_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('trajectory_loader'), 'config', 'trajectory_loader.param.yaml']
        ).perform(context)

    csv_path = LaunchConfiguration('csv_path').perform(context)
    if not csv_path:
        csv_path = PathJoinSubstitution(
            [FindPackageShare('trajectory_loader'), 'data', 'trajectory.csv']
        ).perform(context)

    trajectory_loader_node = Node(
        package='trajectory_loader',
        executable='trajectory_loader_node_exe',
        name='trajectory_loader_node',
        parameters=[
            param_path,
            {
                'csv_path': csv_path
            }
        ],
        remappings=[
            ('~/output/trajectory', LaunchConfiguration('output_trajectory_topic'))
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    return [
        trajectory_loader_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('trajectory_loader_param_file', '')
    add_launch_arg('csv_path', '')
    add_launch_arg('output_trajectory_topic', 'racing_planner/trajectory')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
