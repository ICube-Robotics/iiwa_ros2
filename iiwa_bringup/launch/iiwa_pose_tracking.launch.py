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
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
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
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/hard_joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"]
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
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

    # Start the actual move_group node/action server
    pose_tracking_node = Node(
        package="iiwa_moveit2",
        executable="pose_tracking_servo_node",
        output="screen",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "use_sim_time": use_sim,
                "base_frame": 'iiwa_base',
                "end_effector_frame": 'tool0',
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

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
    )

    # Static TF
    '''
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )
    '''

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("iiwa_description_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    iiwa_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_arm_controller", "-c", "/controller_manager"],
    )

    nodes = [
        rviz_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        iiwa_arm_controller_spawner,
        # static_tf_node,
        pose_tracking_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
