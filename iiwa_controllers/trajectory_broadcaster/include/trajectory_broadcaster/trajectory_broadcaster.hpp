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

#ifndef TRAJECTORY_BROADCASTER__TRAJECTORY_BROADCASTER_HPP_
#define TRAJECTORY_BROADCASTER__TRAJECTORY_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "trajectory_broadcaster/visibility_control.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace trajectory_broadcaster
{
using MsgType = trajectory_msgs::msg::JointTrajectory;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TrajectoryBroadcaster : public controller_interface::ControllerInterface
{
public:
  TRAJECTORY_BROADCASTER_PUBLIC
  TrajectoryBroadcaster();

  TRAJECTORY_BROADCASTER_PUBLIC
  CallbackReturn on_init() override;

  TRAJECTORY_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  TRAJECTORY_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  TRAJECTORY_BROADCASTER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  TRAJECTORY_BROADCASTER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  TRAJECTORY_BROADCASTER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  TRAJECTORY_BROADCASTER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;

  using StatePublisher = realtime_tools::RealtimePublisher<MsgType>;
  rclcpp::Publisher<MsgType>::SharedPtr state_publisher_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
};

}  // namespace trajectory_broadcaster

#endif  // TRAJECTORY_BROADCASTER__TRAJECTORY_BROADCASTER_HPP_
