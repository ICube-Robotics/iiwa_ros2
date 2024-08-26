# Copyright 2024 ICube Laboratory, University of Strasbourg
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
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="iiwa.rviz",
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )

    # Initialize Arguments
    description_package = 'iiwa_description'
    prefix = LaunchConfiguration('prefix')
    start_rviz = LaunchConfiguration('start_rviz')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_sim')

    moveit_config = (
        MoveItConfigsBuilder("iiwa_description")
        .robot_description(file_path="config/iiwa.urdf.xacro",
            mappings={
                "prefix": prefix,
                "base_frame_file": base_frame_file,
                "description_package": description_package,
                "namespace": namespace
            },
        )
        .robot_description_semantic(file_path="config/iiwa.srdf.xacro",
            mappings={
                "name": "iiwa",
                "prefix": prefix,
                "description_package": description_package,
            },
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")  # "config/bio_ik_kinematics.yaml"
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/hard_joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("iiwa_description_moveit_config")
        .yaml("config/iiwa_moveit2_servo_config.yaml")
        .to_dict()
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "iiwa_arm"}

    # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("iiwa_description"), "rviz", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(start_rviz),
    )

    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen"
    )

    nodes = [
        rviz_node,
        servo_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
