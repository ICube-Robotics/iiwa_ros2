ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/iiwa_ros2
RUN apt update && \
    apt install -y libeigen3-dev
RUN cd ros2_dev && \
    apt update && \
    cd iiwa_ros2 && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    vcs import < iiwa_ros2.repos && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --symlink-install
COPY .docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
