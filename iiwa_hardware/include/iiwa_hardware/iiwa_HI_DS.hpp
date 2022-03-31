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

#ifndef IIWA_HARDWARE__IIWA_HI_DS
#define IIWA_HARDWARE__IIWA_HI_DS

#include <memory>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "iiwa_hardware/visibility_control.h"

typedef int SOCKET;
#define INVALID_SOCKET -1
#define SOCKET_ERROR   -1
#define MAXBUFLEN 16000
#define NO_FLAGS_SET 0



namespace iiwa_hardware
{
class IiwaDirectServoPositionHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IiwaDirectServoPositionHardwareInterface);

  IIWA_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  IIWA_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  IIWA_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  IIWA_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  IIWA_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  IIWA_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  IIWA_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Communication
  sockaddr_in destSockAddr_from_;
  socklen_t fromlen_;
  sockaddr_in destSockAddr_to_;
  SOCKET destSocket_;
  unsigned long counter_;
  bool socketConnected_;
  int socket_status_;
  char buffer_[MAXBUFLEN];
  std::string buffRecal_ = "";

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<double> debug_states_;
};

}  // namespace iiwa_hardware

#endif  // IIWA_HARDWARE__IIWA_HI_DS
