// Copyright 2024, ICube Laboratory, University of Strasbourg
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
// Adapted from https://github.com/moveit/moveit2/blob/main/moveit_ros/moveit_servo/demos/cpp_interface/demo_pose.cpp

#include <atomic>
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

static const std::string NODE_NAME = "pose_tracking_servo_node";
static const rclcpp::Logger LOGGER = rclcpp::get_logger(NODE_NAME);

using namespace moveit_servo;

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
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(NODE_NAME);
  moveit::setNodeLoggerName(NODE_NAME);

  // Declare parameters
  node->declare_parameter("base_frame", "iiwa_base");
  node->declare_parameter("end_effector_frame", "iiwa_tool");

  std::string base_frame = node->get_parameter("base_frame").as_string();
  std::string end_effector_frame = node->get_parameter("end_effector_frame").as_string();

  if (base_frame.empty() || end_effector_frame.empty()) {
    RCLCPP_ERROR(LOGGER, "Parameters 'base_frame' and 'end_effector_frame' must be specified.");
    return EXIT_FAILURE;
  }

  // Get the servo parameters.
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  // The publisher to send trajectory message to the robot controller.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());
  // Create the servo object
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(node, servo_params);
  Servo servo = Servo(node, servo_param_listener, planning_scene_monitor);

  // Helper function to get the current pose of a specified frame.
  const auto get_current_pose = [](const std::string& target_frame, const moveit::core::RobotStatePtr& robot_state) {
    return robot_state->getGlobalLinkTransform(target_frame);
  };

  // Wait for some time, so that the planning scene is loaded in rviz.
  // This is just for convenience, should not be used for sync in real application.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Get the robot state and joint model group info.
  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  // Set the command type for servo.
  servo.setCommandType(CommandType::POSE);

  // The dynamically updated target pose.
  PoseCommand target_pose;
  target_pose.frame_id = base_frame;

  // Initializing the target pose as end effector pose, this can be any pose.
  target_pose.pose = get_current_pose(end_effector_frame, robot_state);

  // servo loop will exit upon reaching this pose.
  Eigen::Isometry3d terminal_pose = target_pose.pose;
  terminal_pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  terminal_pose.translate(Eigen::Vector3d(0.0, 0.0, -0.1));

  // The target pose (frame being tracked) moves by this step size each iteration.
  Eigen::Vector3d linear_step_size{ 0.0, 0.0, -0.001 };
  Eigen::AngleAxisd angular_step_size(0.01, Eigen::Vector3d::UnitZ());

  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());

  rclcpp::WallRate servo_rate(1 / servo_params.publish_period);

  // create command queue to build trajectory message and add current robot state
  std::deque<KinematicState> joint_cmd_rolling_window;
  KinematicState current_state = servo.getCurrentRobotState();
  updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, node->now());

  bool satisfies_linear_tolerance = false;
  bool satisfies_angular_tolerance = false;
  bool stop_tracking = false;
  while (!stop_tracking && rclcpp::ok())
  {
    // check if goal reached
    target_pose.pose = get_current_pose(end_effector_frame, robot_state);
    satisfies_linear_tolerance |= target_pose.pose.translation().isApprox(terminal_pose.translation(),
                                                                          servo_params.pose_tracking.linear_tolerance);
    satisfies_angular_tolerance |=
        target_pose.pose.rotation().isApprox(terminal_pose.rotation(), servo_params.pose_tracking.angular_tolerance);
    stop_tracking = satisfies_linear_tolerance && satisfies_angular_tolerance;

    // Dynamically update the target pose.
    if (!satisfies_linear_tolerance)
    {
      target_pose.pose.translate(linear_step_size);
    }
    if (!satisfies_angular_tolerance)
    {
      target_pose.pose.rotate(angular_step_size);
    }

    // get next servo command
    KinematicState joint_state = servo.getNextJointState(robot_state, target_pose);
    StatusCode status = servo.getStatus();
    if (status != StatusCode::INVALID)
    {
      updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency, node->now());
      if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
      {
        trajectory_outgoing_cmd_pub->publish(msg.value());
      }
      if (!joint_cmd_rolling_window.empty())
      {
        robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
        robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
      }
    }

    servo_rate.sleep();
  }

  RCLCPP_INFO_STREAM(LOGGER, "REACHED : " << stop_tracking);
  RCLCPP_INFO(LOGGER, "Exiting demo.");
  rclcpp::shutdown();
}