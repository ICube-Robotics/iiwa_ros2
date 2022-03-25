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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro
import yaml


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
    # Get URDF and SRDF
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

    # Get parameters for the Pose Tracking node
    pose_tracking_yaml = load_yaml(
        'iiwa_description', 'moveit2/iiwa_moveit2_pose_tracking_settings.yaml'
    )
    pose_tracking_params = {'moveit_servo': pose_tracking_yaml}

    # Get parameters for the Servo node
    servo_yaml = load_yaml(
        'iiwa_description', 'moveit2/iiwa_moveit2_pose_tracking_config.yaml'
    )
    servo_params = {'moveit_servo': servo_yaml}

    kinematics_yaml = load_yaml(
        'iiwa_description', 'moveit2/kinematics.yaml'
    )
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            'iiwa_description', 'moveit2/iiwa_joint_limits.yaml'
        )
    }

    # RViz
    rviz_config_file = (
        get_package_share_directory('iiwa_description')
        + '/rviz/iiwa.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        name='iiwa_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # A node to publish world -> panda_link0 transform
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'iiwa_base'],
    )

    pose_tracking_node = Node(
        package='iiwa_moveit2',
        executable='pose_tracking_servo_node',
        name='pose_tracking_servo_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            pose_tracking_params,
            servo_params,
            joint_limits_yaml,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory('iiwa_description'),
        'config',
        'iiwa_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='',
        parameters=[robot_description, ros2_controllers_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # Load controllers
    load_controllers = []
    for controller in ['iiwa_arm_controller', 'joint_state_broadcaster', 'ets_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner.py \
                --controller-manager /controller_manager {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            pose_tracking_node,
            ros2_control_node,
            robot_state_publisher,
        ]
        + load_controllers
    )
