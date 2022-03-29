
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

#include "iiwa_hardware/iiwa_HI_DS.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>

#include <boost/algorithm/string.hpp>

#include "rclcpp/rclcpp.hpp"

namespace iiwa_hardware
{
  // ------------------------------------------------------------------------------------------
CallbackReturn IiwaDirectServoPositionHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
{
  return CallbackReturn::ERROR;
}


  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // iiwaRobot has currently exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"),
        "Joint '%s' has %d state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  socketConnected_ = false;

  return CallbackReturn::SUCCESS;
}
  // ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
IiwaDirectServoPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}
  // ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
IiwaDirectServoPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}
  // ------------------------------------------------------------------------------------------
CallbackReturn IiwaDirectServoPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "Starting ...please wait...");

  std::string p_ip  = info_.hardware_parameters["robot_ip"];
  int p_port = stoi(info_.hardware_parameters["robot_port"]);

  RCLCPP_INFO(rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"),"Connecting to port= %i and ip= %s", p_port, p_ip);


  // IP address, port, etc., setting
  // memset(&destSockAddr_, 0, sizeof(destSockAddr_));
  // in_addr_t destAddr=inet_addr(p_ip.c_str());
  // memcpy(&destSockAddr_.sin_addr, &destAddr, sizeof(destAddr));
  unsigned short port=p_port;
  sockaddr_in addr;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);
	addr.sin_family = AF_INET;

  inet_pton(AF_INET, p_ip.c_str(), &destSockAddr_to_.sin_addr.s_addr);
	destSockAddr_to_.sin_family = AF_INET;
	destSockAddr_to_.sin_port = htons(port);

  fromlen_ = sizeof(destSockAddr_from_);

  // Socket creation
  destSocket_=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (destSocket_ == INVALID_SOCKET) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"),
        "ERROR: socket unsuccessful");
      return CallbackReturn::ERROR;
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  if (int err = setsockopt(destSocket_, SOL_SOCKET, SO_RCVTIMEO,(char*)&tv,sizeof(tv))) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"),
        "ERROR: setsockopt unsuccessful : %i ", err);
      return CallbackReturn::ERROR;
  }

  if (bind(destSocket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"),
        "ERROR: Bind failed");
      return CallbackReturn::ERROR;
    }

  std::ostringstream vts;
  if (!hw_commands_.empty()){
    std::copy(hw_commands_.begin(), hw_commands_.end()-1, std::ostream_iterator<int>(vts, ","));
    vts << hw_commands_.back();
  }
  std::string msg_data_send = ">iiwa_joint_goal," + vts.str();

  while(!socketConnected_){
    socket_status_=sendto(destSocket_, msg_data_send.c_str(), msg_data_send.length(), NO_FLAGS_SET, reinterpret_cast<const sockaddr*>( &destSockAddr_to_), sizeof(destSockAddr_to_));
    if (socket_status_ != msg_data_send.length()) {
        RCLCPP_FATAL(
          rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "ERROR: sendto unsuccessful in first send.");
        return CallbackReturn::ERROR;
    }
    socket_status_=recvfrom(destSocket_, buffer_, MAXBUFLEN, NO_FLAGS_SET, reinterpret_cast<sockaddr*>(&destSockAddr_from_) ,&fromlen_);
    if (socket_status_ == SOCKET_ERROR) {
        RCLCPP_FATAL(
          rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "ERROR: recvfrom unsuccessful in first recv.");
        return CallbackReturn::ERROR;
    }
    else if(socket_status_ > 0){
      std::cout << "got data: " << buffer_ << std::endl;
      buffRecal_.append(buffer_,socket_status_);
      std::vector <std::string> pack;
      boost::split(pack,buffer_,boost::is_any_of(">"));
      std::cout << "pack data: " << pack.size() << std::endl;

      std::vector <std::string> state_elements;
      boost::split(state_elements,pack[1],boost::is_any_of(","));
      if(state_elements[0] == "iiwa_joint_state" && state_elements.size() == hw_states_.size()+1)
          for(int i=1;i<state_elements.size();i++) hw_states_[i-1]=atof(state_elements[i].c_str());

      socketConnected_ = true;
    }
  }
    std::cout << "Initial joint position: " << hw_states_[0] <<", " << hw_states_[1] <<", " << hw_states_[2] <<", " << hw_states_[3] <<", " << hw_states_[4] <<", " << hw_states_[5] <<", " << hw_states_[6] << std::endl;

  hw_commands_ = hw_states_;

  RCLCPP_INFO(
    rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn IiwaDirectServoPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "Stopping ...please wait...");

  // stopping routine
  std::string msg_data_send = ">shutdown";

  socket_status_=sendto(destSocket_, msg_data_send.c_str(), msg_data_send.length(), NO_FLAGS_SET, reinterpret_cast<const sockaddr*>( &destSockAddr_to_), sizeof(destSockAddr_to_));
  if (socket_status_ != msg_data_send.length()) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "ERROR: sendto unsuccessful.");
      return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}
  // ------------------------------------------------------------------------------------------
hardware_interface::return_type IiwaDirectServoPositionHardwareInterface::read()
{
  // RCLCPP_INFO(rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "Reading...");
  socket_status_=recvfrom(destSocket_, buffer_, MAXBUFLEN, NO_FLAGS_SET, reinterpret_cast<sockaddr*>(&destSockAddr_from_) ,&fromlen_);
  if (socket_status_ == SOCKET_ERROR ) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "ERROR: recvfrom unsuccessful.");
      return hardware_interface::return_type::ERROR;
  }
  std::vector <std::string> pack;
  boost::split(pack,buffer_,boost::is_any_of(">"));

  std::vector <std::string> state_elements;
      boost::split(state_elements,pack[1],boost::is_any_of(","));
      if(state_elements[0] == "iiwa_joint_state" && state_elements.size() == hw_states_.size()+1)
          for(int i=1;i<state_elements.size();i++) hw_states_[i-1]=atof(state_elements[i].c_str());

  // RCLCPP_INFO(rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}
  // ------------------------------------------------------------------------------------------
hardware_interface::return_type IiwaDirectServoPositionHardwareInterface::write()
{
  // RCLCPP_INFO(rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "Writing...");

  std::ostringstream vts;
  if (!hw_commands_.empty()){
    std::copy(hw_commands_.begin(), hw_commands_.end()-1, std::ostream_iterator<double>(vts, ","));
    vts << hw_commands_.back();
  }
  std::string msg_data_send = ">iiwa_joint_goal," + vts.str();

  socket_status_=sendto(destSocket_, msg_data_send.c_str(), msg_data_send.length(), NO_FLAGS_SET, reinterpret_cast<const sockaddr*>( &destSockAddr_to_), sizeof(destSockAddr_to_));
  if (socket_status_ != msg_data_send.length()) {
      RCLCPP_FATAL(
        rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "ERROR: sendto unsuccessful.");
      return hardware_interface::return_type::ERROR;
  }

  // std::cout << "Sending: " << msg_data_send << std::endl;

  // RCLCPP_INFO(
  //   rclcpp::get_logger("IiwaDirectServoPositionHardwareInterface"), "Sending: %s",msg_data_send);

  return hardware_interface::return_type::OK;
}

}  // namespace iiwa_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  iiwa_hardware::IiwaDirectServoPositionHardwareInterface, hardware_interface::SystemInterface)
