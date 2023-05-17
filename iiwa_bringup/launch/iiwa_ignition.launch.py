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

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    set_ign_path = SetEnvironmentVariable(
        name = 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value = os.path.join(
            get_package_prefix('ign_ros2_control'),
            'lib'
        )
    )

    iiwa_simulation_world = os.path.join(
        get_package_share_directory('iiwa_description'),
        'ignition/worlds',
        'empty.sdf')

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                          'launch',
                          'ign_gazebo.launch.py')]
        ),
        launch_arguments=[('ign_args', [' -r -v 4 ' + iiwa_simulation_world])]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('iiwa_description'),
                    'config',
                    'iiwa7.config.xacro',
                ]
            ),
            ' ',
            'use_sim:=true',
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-world', 'empty',
                   '-name', 'iiwa7',
                   '-string', robot_description_content],
        output='screen',
    )

    load_jst_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'iiwa_arm_controller'],
        output='screen'
    )

    return LaunchDescription(
        [
            set_ign_path,
            ignition,
            node_robot_state_publisher,
            spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_jst_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_jst_controller,
                    on_exit=[load_arm_controller],
                )
            ),
        ]
    )
