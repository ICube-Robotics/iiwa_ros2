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

#ifndef IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROLLER_HPP_
#define IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "impedance_controller/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace impedance_controller
{
using CmdType = trajectory_msgs::msg::JointTrajectory;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * \brief Impedance controller for a set of joints.
 *
 * This class computes the torque given by the impedance relationship and applies it to the defined joints.
 *
 * \param joints Names of the joints to control.
 * \param interface_name Name of the interface to command.
 *
 * Subscribes to:
 * - \b proxy (trajectory_msgs::msg::JointTrajectory) : The trajectory of the proxy to follow.
 */
class ImpedanceController : public controller_interface::ControllerInterface
{
public:
  IMPEDANCE_CONTROLLER_PUBLIC
  ImpedanceController();

  IMPEDANCE_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  /**
   * @brief command_interface_configuration This controller requires the effort command
   * interfaces for the controlled joints
   */
  IMPEDANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief command_interface_configuration This controller requires the position and velocity
   * state interfaces for the controlled joints
   */
  IMPEDANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  IMPEDANCE_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  IMPEDANCE_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  IMPEDANCE_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  IMPEDANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<double> stiffness_;
  std::vector<double> damping_;
  std::vector<double> mass_;


  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  std::string logger_name_;

};

}  // namespace impedance_controller

#endif  // IMPEDANCE_CONTROLLER__IMPEDANCE_CONTROLLER_HPP_
