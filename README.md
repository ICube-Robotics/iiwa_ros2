# IIWA_ROS2 #
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![DOI](https://zenodo.org/badge/470651211.svg)](https://zenodo.org/badge/latestdoi/470651211)
[![CI](https://github.com/ICube-Robotics/iiwa_ros2/actions/workflows/ci.yaml/badge.svg)](https://github.com/ICube-Robotics/iiwa_ros2/actions/workflows/ci.yaml)

ROS2 stack for KUKA iiwa 14 collaborative robots. This package contains launch and configuration setups to quickly get started using the driver.

## Features ##
- integration with `ros2_control`
- robot drivers for KUKA Fast Robot Interface (FRI) protocol for position, velocity and torque control
- dedicated sensors and broadcasters to get data from the robot
- dedicated controllers
- integration with Gazebo
- integration with Moveit2 (OMPL, PILZ and servo)

## Available Packages in this Repository ##
- `iiwa_bringup` - launch and run-time configurations
- `iiwa_controllers` - implementation of dedicated controllers
- `iiwa_description` - robot description and configuration files
- `iiwa_hardware` - hardware interfaces for communication with the robot
- `iiwa_moveit2` - some tools for Moveit2 integration

## Getting Started
***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```shell
    cd ~/ros2_ws
    git clone https://github.com/ICube-Robotics/iiwa_ros2.git src/iiwa_ros2
    vcs import src < src/iiwa_ros2.repos
    rosdep install --ignore-src --from-paths . -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
**NOTE:** The `iiwa_ros2.repos` file contains links to ros2 packages that need to be source-built to use their newest features.

## Usage

:warning: **SAFETY FIRST**:warning:
*An industrial robot is not a toy and you may harm yourself due to misuse. In general it is best practice to test your code at first in simulation and then in low speed (**T1**) mode. Before using the robot, make yourself familiar with the safety instructions provided by the KUKA manuals.*

### On the Robot side:
**Step 1:** The used drivers allow the communication with the KUKA *iiwa* robot using KUKA's **Fast Robot Interface (FRI)**. Therefore, the `Fast Robot Interface Extension` needs to be installed and configured on the robot.

**HINT:** In the proposed default setup of this package, the robot and the control PC are communicating through FRI on the `KUKA Option Network Interface` (KONI) with the following setup:
- Robot : `IP = 192.170.10.2`, `SubnetMask = FFFFFF00`
- Control PC : `IP = 192.170.10.5`, `SubnetMask = FFFFFF00`

For further instructions concerning the installation and setup of FRI, please refer to KUKA FRI documentation.

**Step 2:** This step consists in installing the `iiwa_ros2.java` application from the `iiwa_sunrise` directory in the `application` package of your robot Sunrise Project. This application allows you to establish a communication with the control PC and initialize one of the following control modes:
- `POSITION` - position and velocity commands can be passed to the robot and executed, the robot sends its current status
- `TORQUE` - torque commands can be passed to the robot and executed, the robot sends its current status
- `MONITORING` - no commands can be passed to the robot, the robot only sends its current status

**NOTE:** Depending on you application, the following parameters need to be tuned in the application:
- `INITIAL_POSITION` (default: same as `iiwa_description/config/initial_positions.yaml`) - the initial joint configuration of the robot/
- `CLIENT_IP` (default: `192.170.10.5`) - IP of the control PC allowed to send data to the robot.
- `TS` (default: 5ms) - Communication period. The robot throws an Error if no data received during the specified period.

**NOTE:** For torque mode, there has to be a command value at least all 5ms.


**Step 3:** To control the robot using `iiwa_ros2` execute the application on the robot and select the desired control mode.

**NOTE:** All security modes (T1, T2, AUTO) are supported.

### On ROS2 side:
The `iiwa_bringup` package contains 3 main launch files: 2 examples and the main driver launcher
- `joy_servo_teleop.launch.py` - launches a fake robot controlled by a joystick using `moveit_servo`
- `iiwa_pose_tracking.launch.py` - launches a fake robot tracking a pose pusblished in topic `\target_pose` using pose tracking capabilities of`moveit_servo`
- `iiwa.launch.py` - is the main launcher giving access to all feaures of the driver.

The arguments for launch files can be listed using
```shell
ros2 launch iiwa_bringup <launch_file_name>.launch.py --show-args
```
The most relevant arguments of `iiwa.launch.py` are:

- `runtime_config_package` (default: "iiwa_description") - name of the package with the controller's configuration in `config` folder. Usually the argument is not set, it enables use of a custom setup.
- `controllers_file` (default: "iiwa_controllers.yaml"- YAML file with the controllers configuration.
- `description_package` (default: "iiwa_description") - Description package with robot URDF/xacro files. Usually the argument is not set, it enables use of a custom description.
- `description_file` (default: "iiwa.config.xacro") - URDF/XACRO description file with the robot.
- `prefix` (default: "") - Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated. Expected format `<prefix>/`.
- `namespace` (default: "/") - Namespace of launched nodes, useful for multi-robot setup. If changed than also the namespace in the controllers configuration needs to be updated. Expected format `<ns>/`.
- `use_sim` (default: "false") - Start robot in Gazebo simulation.
- `use_fake_hardware` (default: "true") - Start robot with fake hardware mirroring command to its states.
- `use_planning` (default: "false") - Start robot with Moveit2 `move_group` planning configuration for Pilz and OMPL.
- `use_servoing` (default: "false") - Start robot with Moveit2 servoing.
- `robot_controller` (default: "iiwa_arm_controller") - Robot controller to start.
- `start_rviz` (default: "true") - Start RViz2 automatically with this launch file.
- `robot_ip` (default: "192.170.10.2") - Robot IP of FRI interface.
- `robot_port` (default: "30200") - Robot port of FRI interface.
- `initial_positions_file` (default: "initial_positions.yaml") - Configuration file of robot initial positions for simulation.
- `command_interface` (default: "position") - Robot command interface [position|velocity|effort].
- `base_frame_file` (default: "base_frame.yaml") - Configuration file of robot base frame wrt the World frame.

As an example, to run the `velocity_controller` on the real hardware with default ip and port, run

```shell
ros2 launch iiwa_bringup iiwa.launch.py use_fake_hardware:="false" command_interface:="velocity" robot_controller:="velocity_controller"
```

**HINT**: list all loaded controllers using `ros2 control list_controllers` command.

**NOTE**: The package can simulate hardware with the ros2_control `FakeSystem`. This is the default behavior. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see ros2_control documentation for more details.

### Example commands for setup testing
1. Start the simulated hardware, in a sourced terminal run
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py
    ```
    add the parameter `use_fake_hardware:="false"` to control the real robot, or `use_sim:="true"` to start a simulated robot in Gazebo.
2. Send joint trajectory goals to the hardware by using a demo node from [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) package by running
    ```shell
    ros2 launch iiwa_bringup iiwa_test_joint_trajectory_controller.launch.py
    ```

After a few seconds the robot should move.

## Practical information
### Domain setup
As by default ROS2 streams all data on the network, in order to avoid message interference, it is preferred to isolate the communications by defining domains per project/application.

To do so run `export ROS_DOMAIN_ID= [your_domain_id]`, with `[your_domain_id]` between 0 and 255.

### Running with Gazebo
In order for Gazebo to find the robot model from the `iiwa_ros2` stack it needs to be referenced in the `GAZEBO_MODEL_PATH` environment variable. To do so, run:
```shell
$ source /usr/share/gazebo/setup.sh
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/iiwa_ros2
```
**NOTE**: If you encounter issues with spawning the robot to Gazebo making it crash, make sure your models are well referenced.
## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)
