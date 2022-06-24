# IIWA ROS2 Docker Containers
Provides a basic preconfigured docker container for development purposes. To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

```shell
$ docker build --tag iiwa_ros2:humble --file .docker/Dockerfile .
$ docker run -it iiwa_ros2:humble
```

### Example
To check the setup, run inside docker:
```shell
$ cd ros2_dev/iiwa_ros2/
$ source install/setup.bash
$ ros2 launch iiwa_bringup iiwa.launch.py
```
The `iiwa_ros2` nodes should now be running.
