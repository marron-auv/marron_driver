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

#ifndef MARRON_DRIVER__DRIVER_COMPONENT_HPP_
#define MARRON_DRIVER__DRIVER_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace marron_driver
{
class KVH1750DriverComponent : public rclcpp::Node
{
public:
  MARRON_DRIVER_PUBLIC
  explicit KVH1750DriverComponent(const rclcpp::NodeOptions & options);
  ~KVH1750DriverComponent();
};
}  // namespace marron_driver

#endif  // MARRON_DRIVER__DRIVER_COMPONENT_HPP_
