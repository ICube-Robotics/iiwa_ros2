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

#include "external_torque_sensor_broadcaster/external_torque_sensor_broadcaster.hpp"

#include <memory>
#include <string>

namespace external_torque_sensor_broadcaster
{
ExternalTorqueSensorBroadcaster::ExternalTorqueSensorBroadcaster()
: controller_interface::ControllerInterface()
{
}

CallbackReturn ExternalTorqueSensorBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("sensor_name", "");
    auto_declare<std::string>("interface_names.joint_a1", "");
    auto_declare<std::string>("interface_names.joint_a2", "");
    auto_declare<std::string>("interface_names.joint_a3", "");
    auto_declare<std::string>("interface_names.joint_a4", "");
    auto_declare<std::string>("interface_names.joint_a5", "");
    auto_declare<std::string>("interface_names.joint_a6", "");
    auto_declare<std::string>("interface_names.joint_a7", "");

  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ExternalTorqueSensorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  interface_names_[0] = get_node()->get_parameter("interface_names.joint_a1").as_string();
  interface_names_[1] = get_node()->get_parameter("interface_names.joint_a2").as_string();
  interface_names_[2] = get_node()->get_parameter("interface_names.joint_a3").as_string();
  interface_names_[3] = get_node()->get_parameter("interface_names.joint_a4").as_string();
  interface_names_[4] = get_node()->get_parameter("interface_names.joint_a5").as_string();
  interface_names_[5] = get_node()->get_parameter("interface_names.joint_a6").as_string();
  interface_names_[6] = get_node()->get_parameter("interface_names.joint_a7").as_string();

  const bool no_interface_names_defined =
    std::count(interface_names_.begin(), interface_names_.end(), "") == 7;

  if (sensor_name_.empty() && no_interface_names_defined) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'sensor_name' or at least one "
      "'interface_names.joint_a[x]' parameter has to be specified.");
    return CallbackReturn::ERROR;
  }

  if (!sensor_name_.empty() && !no_interface_names_defined) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "both 'sensor_name' and "
      "'interface_names.[force|torque].[x|y|z]' parameters can not be specified together.");
    return CallbackReturn::ERROR;
  }

  if (!sensor_name_.empty()) {
    external_torque_sensor_ = std::make_unique<semantic_components::ExternalTorqueSensor>(
      semantic_components::ExternalTorqueSensor(sensor_name_));
  } else {
    external_torque_sensor_ = std::make_unique<semantic_components::ExternalTorqueSensor>(
      semantic_components::ExternalTorqueSensor(
        interface_names_[0], interface_names_[1], interface_names_[2], interface_names_[3],
        interface_names_[4], interface_names_[5], interface_names_[6]));
  }

  try {
    // register ft sensor data publisher
    sensor_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/external_torques", rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<StatePublisher>(sensor_state_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ExternalTorqueSensorBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
ExternalTorqueSensorBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = external_torque_sensor_->get_state_interface_names();
  return state_interfaces_config;
}

CallbackReturn ExternalTorqueSensorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  external_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExternalTorqueSensorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  external_torque_sensor_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ExternalTorqueSensorBroadcaster::update(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  if (realtime_publisher_ && realtime_publisher_->trylock()) {
    external_torque_sensor_->get_values_as_message(realtime_publisher_->msg_);
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace external_torque_sensor_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  external_torque_sensor_broadcaster::ExternalTorqueSensorBroadcaster,
  controller_interface::ControllerInterface)
