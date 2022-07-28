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

#include "admittance_controller/admittance_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace admittance_controller
{
using hardware_interface::LoanedCommandInterface;

AdmittanceController::AdmittanceController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

controller_interface::return_type AdmittanceController::init(
  const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<double>>("stiffness", std::vector<double>());
    auto_declare<std::vector<double>>("damping", std::vector<double>());
    auto_declare<std::vector<double>>("mass", std::vector<double>());
    auto_declare<std::string>("sensor_name", "");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn AdmittanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the joints to be controlled
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }
  // getting the admittance parameters

  // TODO (mcbed): define damping from damping ratio
  stiffness_ = get_node()->get_parameter("stiffness").as_double_array();
  damping_ = get_node()->get_parameter("damping").as_double_array();
  mass_ = get_node()->get_parameter("mass").as_double_array();

  if (stiffness_.empty()) {
    stiffness_.resize(joint_names_.size(), 50.0);
  }
  if (damping_.empty()) {
    damping_.resize(joint_names_.size(), 10.0);
  }
  if (mass_.empty()) {
    mass_.resize(joint_names_.size(), 0.0);
  }

  if ((stiffness_.size() != damping_.size()) || (stiffness_.size() != mass_.size())) {
    RCLCPP_ERROR(get_node()->get_logger(), "incoherent size of admittance parameters");
    return CallbackReturn::FAILURE;
  }

  for (auto i = 0ul; i < stiffness_.size(); i++) {
    if (stiffness_[i] < 0 || damping_[i] < 0 || mass_[i] < 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "wrong admittance parameters");
      return CallbackReturn::FAILURE;
    }
  }

  sensor_name_ = get_node()->get_parameter("sensor_name").as_string();
  if (sensor_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter has to be specified.");
    return CallbackReturn::FAILURE;
  } else {
    external_torque_sensor_ = std::make_unique<semantic_components::ExternalTorqueSensor>(
      semantic_components::ExternalTorqueSensor(sensor_name_));
  }
  // the desired position of the proxy point are queried from the proxy topic
  // and passed to update via a rt pipe
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/proxy", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) {rt_command_ptr_.writeFromNonRT(msg);});

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
// As admittance control targets the position or velocity interface.
controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() const
{
  // TODO (mcbed): choose position or velocity command interface
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return conf;
}
// Admittance control requires both velocity and position states, as well as
// interaction force sesning. For this reason there can be directly defined here
// without the need of getting as parameters.
// The state interfaces are then deployed to all targeted joints.
controller_interface::InterfaceConfiguration
AdmittanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  auto sensor_state_names = external_torque_sensor_->get_state_interface_names();
  conf.names.reserve(joint_names_.size() * 2 + sensor_state_names.size());
  for (const auto & joint_name : joint_names_) {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & sensor_name : sensor_state_names) {
    conf.names.push_back(sensor_name);
  }

  return conf;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template<typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names) {
    for (auto & command_interface : unordered_interfaces) {
      if (
        (command_interface.get_name() == joint_name) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn AdmittanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_EFFORT, ordered_interfaces) ||
    command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position command interfaces, got %zu",
      joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  external_torque_sensor_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}
// When deactivating the controller, the effort command on all joints is set to 0
CallbackReturn AdmittanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(mcbed): only if velocity command
  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    command_interfaces_[index].set_value(0.0);
  }

  external_torque_sensor_->release_interfaces();

  return CallbackReturn::SUCCESS;
}
// main control loop function getting the state interface and writing to the command interface
controller_interface::return_type AdmittanceController::update()
{
  // getting the data from the subscriber using the rt pipe
  auto proxy = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!proxy || !(*proxy)) {
    return controller_interface::return_type::OK;
  }

  //checking proxy data validity
  if ((*proxy)->joint_names.size() != joint_names_.size() ||
    (*proxy)->points[0].positions.size() != joint_names_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000, "command size does not match number of interfaces");
    return controller_interface::return_type::ERROR;
  }

  auto sensor_data = external_torque_sensor_->get_torques();
  std::vector<double> external_torques(sensor_data.begin(), sensor_data.end());
  //Admittance control loop
  for (auto index = 0ul; index < joint_names_.size(); ++index) {
    // the stats are given in the same order as defines in state_interface_configuration

    double q = state_interfaces_[2 * index].get_value();
    double qv = state_interfaces_[2 * index + 1].get_value();
    double qd = (*proxy)->points[0].positions[index];

    double qdv = 0;
    if ((*proxy)->points[0].velocities.size() == joint_names_.size()) {
      qdv = (*proxy)->points[0].velocities[index];
    }

    double qda = 0;
    if ((*proxy)->points[0].accelerations.size() == joint_names_.size()) {
      qda = (*proxy)->points[0].accelerations[index];
    }

    double u = qda + 1 / mass_[index] *
      (stiffness_[index] * (qd - q) + damping_[index] * (qdv - qv) + external_torques[index]);
    // TODO (mcbed): simple or double integration for position or velocity interface
    command_interfaces_[index].set_value(internal_position_[index]);

  }

  return controller_interface::return_type::OK;
}

}  // namespace admittance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  admittance_controller::AdmittanceController, controller_interface::ControllerInterface)
