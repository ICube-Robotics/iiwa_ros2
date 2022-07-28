ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

RUN apt update && \
    apt install -y libeigen3-dev
RUN mkdir ros2_dev && \
    cd ros2_dev && \
    apt update && \
    git clone https://github.com/ICube-Robotics/iiwa_ros2 && \
    cd iiwa_ros2 && \
    vcs import < iiwa_ros2.repos && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --symlink-install
COPY .docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
