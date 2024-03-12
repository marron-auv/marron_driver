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

void MotorCommandSenderComponent::command_callback(const std_msgs::msg::UInt8MultiArray & msg) {}
}  // namespace marron_driver

RCLCPP_COMPONENTS_REGISTER_NODE(marron_driver::MotorCommandSenderComponent)
