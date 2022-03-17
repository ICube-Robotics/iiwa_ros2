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

#ifndef IIWA_HARDWARE__IIWA_HI_FRI
#define IIWA_HARDWARE__IIWA_HI_FRI

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "iiwa_hardware/visibility_control.h"

#include "iiwa_hardware/IRDFClient.h"


namespace iiwa_hardware
{
class IiwaFRIPositionHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IiwaFRIPositionHardwareInterface);

  IIWA_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  IIWA_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  IIWA_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  IIWA_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  IIWA_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  IIWA_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  IIWA_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Communication 
  KUKA::FRI::IRDFClient robotClient_;         

  // Store the command for the simulated robot
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_states_external_torque_sensor_;

};

}  // namespace iiwa_hardware

#endif  // IIWA_HARDWARE__IIWA_HI_FRI