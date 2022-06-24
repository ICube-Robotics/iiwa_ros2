# IIWA ROS2 Docker Containers
Provides a basic preconfigured docker container for development purposes. Two versions are available:
- local: build docker using the local version of the stack
- remote: build docker using the Github repository (no need to clone the stack locally)

To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

```shell
$ docker build --tag iiwa_ros2:humble --file .docker/remote/Dockerfile .
$ docker run iiwa_ros2:humble ros2 launch iiwa_bringup iiwa.launch.py
```

### Run with GUI
To run the docker image with GUI, use the [rocker tool](https://github.com/osrf/rocker):
```shell
$ sudo apt install python3-rocker
$ rocker --net=host --x11 iiwa_ros2:humble ros2 launch iiwa_bringup iiwa.launch.py
```

### Run with bash
To interact with the environment, run docker using:
```shell
$ docker run -it iiwa_ros2:humble
```
and inside docker run:
```shell
$ cd ros2_dev/iiwa_ros2/
$ source install/setup.bash
$ ros2 launch iiwa_bringup iiwa.launch.py
```
The `iiwa_ros2` nodes should now be running.
