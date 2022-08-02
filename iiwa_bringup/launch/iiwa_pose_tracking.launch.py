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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('iiwa_description'), 'config', 'iiwa.config.xacro']
            ),
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('iiwa_description'), 'srdf', 'iiwa.srdf.xacro']
            ),
            ' ',
            'name:=',
            'iiwa',
        ]
    )

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Get parameters for the Pose Tracking node
    pose_tracking_params = PathJoinSubstitution([
            FindPackageShare('iiwa_description'),
            'moveit2',
            'iiwa_moveit2_pose_tracking_settings.yaml',
        ]
    )

    # Get parameters for the Servo node
    servo_params = PathJoinSubstitution([
            FindPackageShare('iiwa_description'),
            'moveit2',
            'iiwa_moveit2_pose_tracking_config.yaml',
        ]
    )

    kinematics_yaml = PathJoinSubstitution([
            FindPackageShare('iiwa_description'),
            'moveit2',
            'kinematics.yaml'
        ]
    )

    joint_limits_yaml = PathJoinSubstitution([
            FindPackageShare('iiwa_description'),
            'moveit2',
            'iiwa_cartesian_limits.yaml',
        ]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
            FindPackageShare('iiwa_description'),
            'rviz',
            'iiwa.rviz'
        ]
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

    # A node to publish world -> iiwa_base transform
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
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare('iiwa_description'),
            'config',
            'iiwa_controllers.yaml',
        ]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='',
        parameters=[robot_description, ros2_controllers_path],
        output='both',
    )

    # Load controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    external_torque_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ets_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['iiwa_arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            pose_tracking_node,
            ros2_control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            external_torque_broadcaster_spawner,
            robot_controller_spawner,
        ]
    )
