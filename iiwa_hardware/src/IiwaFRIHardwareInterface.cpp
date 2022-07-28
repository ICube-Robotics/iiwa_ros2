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

#include "iiwa_hardware/IiwaFRIHardwareInterface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace iiwa_hardware
{
// ------------------------------------------------------------------------------------------
CallbackReturn IiwaFRIHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_external_torque_sensor_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  internal_command_position.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // iiwaRobot has currently exactly 3 state and 1 command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaFRIHardwareInterface"),
        "Joint '%s' has %li command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (hw_command_mode_.empty()) {
      hw_command_mode_ = joint.command_interfaces[0].name;

      if (hw_command_mode_ != hardware_interface::HW_IF_POSITION &&
        hw_command_mode_ != hardware_interface::HW_IF_VELOCITY &&
        hw_command_mode_ != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("IiwaFRIHardwareInterface"),
          "Joint '%s' have %s unknown command interfaces.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    if (hw_command_mode_ != joint.command_interfaces[0].name) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaFRIHardwareInterface"),
        "Joint '%s' has %s command interfaces. Expected %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hw_command_mode_.c_str());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaFRIHardwareInterface"),
        "Joint '%s' has %li state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaFRIHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaFRIHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaFRIHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
IiwaFRIHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));

  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));

  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));

  }
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name,
        &hw_states_external_torque_sensor_[i]));

  }

  return state_interfaces;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
IiwaFRIHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    if (hw_command_mode_ == hardware_interface::HW_IF_POSITION) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    } else if (hw_command_mode_ == hardware_interface::HW_IF_VELOCITY) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    } else if (hw_command_mode_ == hardware_interface::HW_IF_EFFORT) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
    }
  }

  return command_interfaces;
}
// ------------------------------------------------------------------------------------------
CallbackReturn IiwaFRIHardwareInterface::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "Starting ...please wait...");

  std::string p_ip = info_.hardware_parameters["robot_ip"];
  int p_port = stoi(info_.hardware_parameters["robot_port"]);
  robotClient_.setVelocityFilterCutOffFreq(
    stod(
      info_.hardware_parameters[
        "velocity_filter_cutoff_freq"]));
  robotClient_.setTorqueFilterCutOffFreq(
    stod(
      info_.hardware_parameters["torque_filter_cutoff_freq"]));

  // set default value for sensor
  for (auto i = 0ul; i < hw_states_external_torque_sensor_.size(); i++) {
    if (std::isnan(hw_states_external_torque_sensor_[i])) {
      hw_states_external_torque_sensor_[i] = 0;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger(
      "IiwaFRIHardwareInterface"), "Connecting FRI to port= %i and ip= %s", p_port, p_ip.c_str());

  robotClient_.connect(p_port, p_ip.c_str());

  RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
CallbackReturn IiwaFRIHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("IiwaFRIHardwareInterface"), "Stopping ...please wait...");

  robotClient_.disconnect();

  RCLCPP_INFO(
    rclcpp::get_logger("IiwaFRIHardwareInterface"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type IiwaFRIHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // read FIR and copy positions to hw_states_
  if (!robotClient_.updateFromRobot()) {
    return hardware_interface::return_type::ERROR;
  }
  if (robotClient_.getCurrentControllerState() != KUKA::FRI::IDLE &&
    robotClient_.getCurrentControllerState() != KUKA::FRI::MONITORING_WAIT)
  {
    robotClient_.getRobotJointPosition(hw_states_position_);
    robotClient_.getRobotJointVelocity(hw_states_velocity_);
    robotClient_.getRobotJointTorque(hw_states_effort_);
    robotClient_.getRobotJointExternalTorque(hw_states_external_torque_sensor_);


    if (internal_command_position[0] != internal_command_position[0]) {
      internal_command_position = hw_states_position_;
    }
  }

  return hardware_interface::return_type::OK;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type IiwaFRIHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // write hw_commands_ to FRI
  bool isNan = false;
  for (auto i = 0ul; i < hw_commands_.size(); i++) {
    if (hw_commands_[i] != hw_commands_[i]) {
      isNan = true;
    }
  }

  if (robotClient_.getCurrentControllerState() == KUKA::FRI::COMMANDING_ACTIVE && !isNan) {
    if (hw_command_mode_ == hardware_interface::HW_IF_POSITION) {
      robotClient_.setTargetJointPosition(hw_commands_);
    } else if (hw_command_mode_ == hardware_interface::HW_IF_VELOCITY) {
      for (auto i = 0ul; i < hw_commands_.size(); i++) {
        internal_command_position[i] = internal_command_position[i] +
          robotClient_.getRobotStatus().sampleTime * hw_commands_[i];
      }
      robotClient_.setTargetJointPosition(internal_command_position);
    } else if (hw_command_mode_ == hardware_interface::HW_IF_EFFORT) {
      robotClient_.setTargetJointTorque(hw_commands_);
      robotClient_.setTargetJointPosition(hw_states_position_);
    }
  }
  if (!robotClient_.updateToRobot()) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace iiwa_hardware
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  iiwa_hardware::IiwaFRIHardwareInterface, hardware_interface::SystemInterface)
