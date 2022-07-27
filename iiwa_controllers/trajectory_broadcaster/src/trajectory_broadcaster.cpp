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

#include "trajectory_broadcaster/trajectory_broadcaster.hpp"

#include <memory>
#include <string>

namespace trajectory_broadcaster
{
TrajectoryBroadcaster::TrajectoryBroadcaster()
: controller_interface::ControllerInterface()
{
}

CallbackReturn TrajectoryBroadcaster::on_init()
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TrajectoryBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the joints to be controlled
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }

  try
  {
    // register ft sensor data publisher
    state_publisher_ = get_node()->create_publisher<MsgType>(
      "/proxy", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(state_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TrajectoryBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
TrajectoryBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * 2);
  for (const auto & joint_name : joint_names_)
  {
      conf.names.push_back(joint_name + "/position");
      conf.names.push_back(joint_name + "/velocity");
  }
  return conf;
}

CallbackReturn TrajectoryBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //initialize emg state msg
  auto & traj_msg = realtime_publisher_->msg_;
  traj_msg.header.stamp = get_node()->now();
  traj_msg.joint_names = joint_names_;
  traj_msg.points.resize(1);
  traj_msg.points[0].positions.resize(joint_names_.size(),std::numeric_limits<double>::quiet_NaN());
  traj_msg.points[0].velocities.resize(joint_names_.size(),std::numeric_limits<double>::quiet_NaN());
  traj_msg.points[0].time_from_start.nanosec = 0.005*1e9;

  return CallbackReturn::SUCCESS;
}

CallbackReturn TrajectoryBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TrajectoryBroadcaster::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    auto & traj_msg = realtime_publisher_->msg_;
    traj_msg.header.stamp = get_node()->now();
    traj_msg.joint_names = joint_names_;
    for (size_t iindex = 0; iindex < joint_names_.size(); ++iindex){
      traj_msg.points[0].positions[iindex] = state_interfaces_[2*iindex].get_value();
      traj_msg.points[0].velocities[iindex] = state_interfaces_[2*iindex+1].get_value();
    }
    traj_msg.points[0].time_from_start = period;

    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace trajectory_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  trajectory_broadcaster::TrajectoryBroadcaster,
  controller_interface::ControllerInterface)
