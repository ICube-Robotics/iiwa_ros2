#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_dev/iiwa_ros2/install/setup.bash"
source "/usr/share/gazebo/setup.sh"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ros2_dev/iiwa_ros2/
exec "$@"
