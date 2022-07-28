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

#ifndef SEMANTIC_COMPONENTS__EXTERNAL_TORQUE_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__EXTERNAL_TORQUE_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace semantic_components
{
class ExternalTorqueSensor : public SemanticComponentInterface<std_msgs::msg::Float64MultiArray>
{
public:
  /// Constructor for iiwa ETS with 7 values
  explicit ExternalTorqueSensor(const std::string & name)
  : SemanticComponentInterface(name, 7)
  {
    // If ETS use standard names
    interface_names_.emplace_back(name_ + "/" + "external_torque.joint_a1");
    interface_names_.emplace_back(name_ + "/" + "external_torque.joint_a2");
    interface_names_.emplace_back(name_ + "/" + "external_torque.joint_a3");
    interface_names_.emplace_back(name_ + "/" + "external_torque.joint_a4");
    interface_names_.emplace_back(name_ + "/" + "external_torque.joint_a5");
    interface_names_.emplace_back(name_ + "/" + "external_torque.joint_a6");
    interface_names_.emplace_back(name_ + "/" + "external_torque.joint_a7");

    // Set all interfaces existing
    std::fill(existing_axes_.begin(), existing_axes_.end(), true);

    // Set default force and torque values to NaN
    std::fill(torques_.begin(), torques_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  /// Constructor for ETS with custom interface names.

  ExternalTorqueSensor(
    const std::string & interface_torque_a1, const std::string & interface_torque_a2,
    const std::string & interface_torque_a3, const std::string & interface_torque_a4,
    const std::string & interface_torque_a5, const std::string & interface_torque_a6,
    const std::string & interface_torque_a7)
  : SemanticComponentInterface("", 7)
  {
    auto check_and_add_interface = [this](const std::string & interface_name, const int index) {
        if (!interface_name.empty()) {
          interface_names_.emplace_back(interface_name);
          existing_axes_[index] = true;
        } else {
          existing_axes_[index] = false;
        }
      };

    check_and_add_interface(interface_torque_a1, 0);
    check_and_add_interface(interface_torque_a2, 1);
    check_and_add_interface(interface_torque_a3, 2);
    check_and_add_interface(interface_torque_a4, 3);
    check_and_add_interface(interface_torque_a5, 4);
    check_and_add_interface(interface_torque_a6, 5);
    check_and_add_interface(interface_torque_a7, 6);


    // Set default force and torque values to NaN
    std::fill(torques_.begin(), torques_.end(), std::numeric_limits<double>::quiet_NaN());
  }

  virtual ~ExternalTorqueSensor() = default;

  /// Return torque.
  /**
   * Return torques of the iiwa ETS.
   *
   * \return array of size 7 with torque values.
   */
  std::array<double, 7> get_torques()
  {
    // find out how many force interfaces are being used
    // torque interfaces will be found from the next index onward
    auto torque_interface_counter = 0;
    for (size_t i = 0; i < 7; ++i) {
      if (existing_axes_[i]) {
        torques_[i] = state_interfaces_[torque_interface_counter].get().get_value();
        ++torque_interface_counter;
      }
    }
    return torques_;
  }

  /// Return Float64MultiArray message with torques.
  /**
   * Constructs and return a tprque message from the current values.
   * The method assumes that the interface names on the construction are in the following order:
   *   torque a1, ... , torque a7
   * \return Float64MultiArray message from values;
   */
  bool get_values_as_message(std_msgs::msg::Float64MultiArray & message)
  {
    // call get_troque() to update with the latest values
    get_torques();

    // update the message values
    std::vector<double> data(torques_.begin(), torques_.end());
    message.data = data;

    return true;
  }

protected:
  std::array<double, 7> torques_;
  std::array<bool, 7> existing_axes_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__EXTERNAL_TORQUE_SENSOR_HPP_
