## 1. Description

This package handles the bringing up of the KUKA iiwa robot and showing it in Rviz. 

## 2. Requirements
The following packages needs to be installed:
- joint_state_publisher
- robot_state_publisher




## 3. Run

To visualize the URDF model of the robot in Rviz, launch robot_state_publisher and publish desired joint configuration, run in two terminals :

```
$ ros2 launch iiwa_bringup rvizDummy.launch.py
```
```
$ ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{name:[joint_a1,joint_a2,joint_a3,joint_a4,joint_a5,joint_a6,joint_a7],position:[0,0,0,0,0,0,0]}"
```

To visualize the URDF model of the robot in Rviz with incrementally generated joint states by dummy_robot_states node:

```
$ ros2 launch iiwa_bringup rvizDummy.launch.py
```


