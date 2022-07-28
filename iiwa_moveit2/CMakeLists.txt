cmake_minimum_required(VERSION 3.10.2)
project(iiwa_moveit2)

# Component Nodes (Shared libraries) ############################
set(SERVO_CONTROLLER_INPUT servo_controller_input)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  moveit_core
  pluginlib
  control_msgs
  moveit_servo
)

# Add executable for using a controller
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/joystick_servo.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "iiwa_servo::JoyToServoPub")

# Add executable for pose tracking
add_executable(pose_tracking_servo_node src/pose_tracking_servo.cpp)
ament_target_dependencies(pose_tracking_servo_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add executable for pose tracking
add_executable(servo_node src/servo_node.cpp)
ament_target_dependencies(servo_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# install executables
install(TARGETS
  pose_tracking_servo_node
  servo_node
  DESTINATION lib/${PROJECT_NAME})

# Install Libraries
install(
  TARGETS
    ${SERVO_CONTROLLER_INPUT}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
