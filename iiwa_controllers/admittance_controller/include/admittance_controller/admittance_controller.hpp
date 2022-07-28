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

#ifndef ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "admittance_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "external_torque_sensor_broadcaster/external_torque_sensor.hpp"


namespace admittance_controller
{
using CmdType = trajectory_msgs::msg::JointTrajectory;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * \brief Admittance controller for a set of joints.
 *
 * This class computes the compliant frame given by the admittance relationship and applies it to the defined joints.
 *
 * \param joints Names of the joints to control.
 * \param interface_name Name of the interface to command.
 *
 * Subscribes to:
 * - \b proxy (trajectory_msgs::msg::JointTrajectory) : The trajectory of the proxy to follow.
 */
class AdmittanceController : public controller_interface::ControllerInterface
{
public:
  ADMITTANCE_CONTROLLER_PUBLIC
  AdmittanceController();

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  /**
   * @brief command_interface_configuration This controller requires the position or velocity command
   * interfaces for the controlled joints
   */
  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief command_interface_configuration This controller requires the position and velocity
   * state interfaces for the controlled joints. A measurement of the interaction force using a
   * dedicated sensor is also required.
   */
  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;
  std::vector<double> mass_;
  std::vector<double> internal_velocity_;
  std::vector<double> internal_position_;

  std::string sensor_name_;
  std::array<std::string, 7> interface_names_;
  std::unique_ptr<semantic_components::ExternalTorqueSensor> external_torque_sensor_;


  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  std::string logger_name_;

};

}  // namespace admittance_controller

#endif  // ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
