# Copyright 2022 ICube Laboratory, University of Strasbourg
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

import os

from launch import LaunchDescription
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from launch.substitutions import ThisLaunchFileDir
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Get parameters for the Servo node
    servo_yaml = load_yaml('iiwa_description', 'moveit2/iiwa_moveit2_servo_config.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    iiwa_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/iiwa.launch.py']),
        launch_arguments={
            'command_interface': 'position',
            'robot_controller': 'iiwa_arm_controller',
        }.items(),
    )

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('iiwa_description'),
            'config',
            'iiwa.config.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        'iiwa_description', 'moveit2/iiwa14.srdf'
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name='moveit_servo_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoServer',
                name='servo_node',
                parameters=[
                    servo_params,
                    robot_description,
                    robot_description_semantic,
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='iiwa_moveit2',
                plugin='iiwa_servo::JoyToServoPub',
                name='controller_to_servo_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy_node',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container, iiwa_launch, ])
