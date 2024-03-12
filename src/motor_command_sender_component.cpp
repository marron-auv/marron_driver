// Copyright (c) 2024 MARRON Project
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

#include <marron_driver/motor_command_sender_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace marron_driver
{
MotorCommandSenderComponent::MotorCommandSenderComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("marron_kvh1750_driver_component", options)
{
  serial_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", rclcpp::QoS{100});
  command_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    "command", rclcpp::QoS{100},
    [this](const std_msgs::msg::UInt8MultiArray & msg) { command_callback(msg); });
}

MotorCommandSenderComponent::~MotorCommandSenderComponent() {}

std_msgs::msg::UInt8MultiArray MotorCommandSenderComponent::to_ascii(const std::string & msg)
{
  std_msgs::msg::UInt8MultiArray ascii;
  ascii.layout.dim.push_back(std_msgs::build<std_msgs::msg::MultiArrayDimension>()
                               .label("ascii")
                               .size(msg.length())
                               .stride(msg.length()));
  for (size_t i = 0; i <= msg.length(); i++) {
    ascii.data.push_back(static_cast<int>(msg[i]));
  }
  return ascii;
}

void MotorCommandSenderComponent::command_callback(const std_msgs::msg::UInt8MultiArray & msg)
{
  // if (msg.layout.dim.size() != 1 || msg.layout.dim[0].size != 8 || msg.layout.dim[0].stride != 8) {
  //   RCLCPP_ERROR_STREAM(get_logger(), "Size of the array should be 8");
  //   return;
  // }
  if (msg.data.size() != 8) {
    RCLCPP_ERROR_STREAM(get_logger(), "Size of the data should be 8");
    return;
  }
  std::string command_string = "";
  for (int i = 0; i <= 7; i++) {
    if (i != 7) {
      command_string = command_string + std::to_string(i) + "," + std::to_string(msg.data[i]) + ",";
    } else {
      command_string = command_string + std::to_string(i) + "," + std::to_string(msg.data[i]);
    }
  }
  RCLCPP_INFO_STREAM(get_logger(), "Command string " << command_string);
  serial_pub_->publish(to_ascii(command_string));
}
}  // namespace marron_driver

RCLCPP_COMPONENTS_REGISTER_NODE(marron_driver::MotorCommandSenderComponent)
