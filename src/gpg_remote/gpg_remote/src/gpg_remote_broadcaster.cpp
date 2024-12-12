// Copyright 2022 Wouter Caarls, PUC-RIO
// Copyright 2021 ros2_control development team
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

#include "gpg_remote/gpg_remote_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

GPGRemoteBroadcaster::GPGRemoteBroadcaster() {}

controller_interface::CallbackReturn GPGRemoteBroadcaster::on_init()
{
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GPGRemoteBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration GPGRemoteBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.push_back("line/level0");
  state_interfaces_config.names.push_back("line/level1");
  state_interfaces_config.names.push_back("line/level2");
  state_interfaces_config.names.push_back("line/level3");
  state_interfaces_config.names.push_back("line/level4");
  state_interfaces_config.names.push_back("battery/voltage");

  return state_interfaces_config;
}

controller_interface::CallbackReturn GPGRemoteBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_publisher_ = get_node()->create_publisher<gpg_remote_msgs::msg::State>(
      "state", rclcpp::SystemDefaultsQoS());

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPGRemoteBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GPGRemoteBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GPGRemoteBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto msg = gpg_remote_msgs::msg::State();
  
  for (size_t ii=0; ii != 5; ++ii)
    msg.line.push_back((int)state_interfaces_[ii].get_value());
  msg.battery = state_interfaces_[5].get_value();
  
  state_publisher_->publish(msg);

  return controller_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  GPGRemoteBroadcaster, controller_interface::ControllerInterface)
