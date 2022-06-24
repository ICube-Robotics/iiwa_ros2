ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

RUN apt update && \
    apt install -y libeigen3-dev
RUN mkdir ros2_dev && \
    cd ros2_dev && \
    git clone https://github.com/ICube-Robotics/iiwa_ros2 && \
    cd iiwa_ros2 && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths . --skip-keys moveit -y -r && \
    colcon build --symlink-install
