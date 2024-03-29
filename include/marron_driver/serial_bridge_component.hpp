// Copyright 2021 LeoDrive, Copyright 2021 the Autoware Foundation
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

#ifndef MARRON_DRIVER__SERIAL_BRIDGE_COMPONENT_HPP_
#define MARRON_DRIVER__SERIAL_BRIDGE_COMPONENT_HPP_

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "msg_converters/converters.hpp"
#include "serial_driver/serial_driver.hpp"

using std_msgs::msg::UInt8MultiArray;

namespace drivers
{
namespace serial_driver
{

/// \brief SerialBridgeComponent class which can send and receive serial data
class SerialBridgeComponent : public rclcpp::Node
{
public:
  /// \brief Default constructor
  /// \param[in] options Options for the node
  explicit SerialBridgeComponent(const rclcpp::NodeOptions & options);

  /// \brief Destructor - required to manage owned IoContext
  ~SerialBridgeComponent();

private:
  void init_port();
  void configure_topics();
  /// \brief Callback for sending a raw serial message
  void subscriber_callback(const UInt8MultiArray::SharedPtr msg);
  /// \breif Callback for when serial data are received
  void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);
  void get_params();

  std::unique_ptr<IoContext> m_owned_ctx{};
  std::string m_device_name{};
  std::unique_ptr<SerialPortConfig> m_device_config;
  std::unique_ptr<SerialDriver> m_serial_driver;
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr m_publisher;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr m_subscriber;
  std::thread port_initialize_thread;
  std::exception_ptr port_initialize_exception;
};  // class SerialBridgeComponent

}  // namespace serial_driver
}  // namespace drivers

#endif  // MARRON_DRIVER__SERIAL_BRIDGE_COMPONENT_HPP_
