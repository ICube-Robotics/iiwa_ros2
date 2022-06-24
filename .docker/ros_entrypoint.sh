#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_dev/iiwa_ros2/install/setup.bash"
exec "$@"
