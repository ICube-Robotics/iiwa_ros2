// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Adapted from https://github.com/ros-planning/moveit2/blob/foxy/moveit_ros/moveit_servo/src/cpp_interface_demo/pose_tracking_demo.cpp
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_tracking_servo_node");

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr & node, const std::string & topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(
      topic, 1, std::bind(&StatusMonitor::statusCB, this, std::placeholders::_1));
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_) {
      status_ = latest_status;
      const auto & status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pose_tracking_servo_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
  if (servo_parameters == nullptr) {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
    exit(EXIT_FAILURE);
  }

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node,
    "robot_description");
  if (!planning_scene_monitor->getPlanningScene()) {
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->providePlanningSceneService();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
    planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
    planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
    false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startPublishingPlanningScene(
    planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  // Wait for Planning Scene Monitor to setup
  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

  // Make a publisher for sending pose commands
  auto target_pose_pub =
    node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 1 /* queue */);

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  Eigen::Vector3d lin_tol{0.001, 0.001, 0.001};
  double rot_tol = 0.01;

  // Get the current EE transform
  geometry_msgs::msg::TransformStamped current_ee_tf;
  tracker.getCommandFrameTransform(current_ee_tf);

  // Convert it to a Pose
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = current_ee_tf.header.frame_id;
  target_pose.pose.position.x = current_ee_tf.transform.translation.x;
  target_pose.pose.position.y = current_ee_tf.transform.translation.y;
  target_pose.pose.position.z = current_ee_tf.transform.translation.z;
  target_pose.pose.orientation = current_ee_tf.transform.rotation;

  // Modify it a little bit
  target_pose.pose.position.x += 0.1;

  // resetTargetPose() can be used to clear the target pose and wait for a new one,
  // e.g. when moving between multiple waypoints

  tracker.resetTargetPose();

  // Publish target pose
  target_pose.header.stamp = node->now();
  target_pose_pub->publish(target_pose);

  // Run the pose tracking in a new thread
  std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol] {
      moveit_servo::PoseTrackingStatusCode tracking_status =
      tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
      RCLCPP_INFO_STREAM(
        LOGGER, "Pose tracker exited with status: "
          << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
    });

  rclcpp::Rate loop_rate(1000);
  for (size_t i = 0; i < 500; ++i) {
    // Modify the pose target a little bit each cycle
    // This is a dynamic pose target
    target_pose.pose.position.z += 0.0004;
    target_pose.header.stamp = node->now();
    target_pose_pub->publish(target_pose);

    loop_rate.sleep();
  }

  // Make sure the tracker is stopped and clean up
  move_to_pose_thread.join();

  // Kill executor thread before shutdown
  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
