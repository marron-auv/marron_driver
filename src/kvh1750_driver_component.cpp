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

#include <marron_driver/kvh1750_driver_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace marron_driver
{
KVH1750DriverComponent::KVH1750DriverComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("marron_kvh1750_driver_component", options)
{
}

KVH1750DriverComponent::~KVH1750DriverComponent() {}
}  // namespace marron_driver

RCLCPP_COMPONENTS_REGISTER_NODE(marron_driver::KVH1750DriverComponent)
