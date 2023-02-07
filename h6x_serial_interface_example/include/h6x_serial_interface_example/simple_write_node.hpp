// Copyright 2023 HarvestX Inc.
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

#pragma once

#include <h6x_serial_interface/h6x_serial_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace h6x_serial_interface_example
{
class SimpleWriteNode : public rclcpp::Node
{
private:
  using PortHandler = h6x_serial_interface::PortHandler;

  PortHandler::UniquePtr port_handler_;
  rclcpp::TimerBase::SharedPtr write_timer_;

public:
  SimpleWriteNode() = delete;
  explicit SimpleWriteNode(const rclcpp::NodeOptions &);
  ~SimpleWriteNode();

private:
  void onWritTimer();
};
}  // namespace h6x_serial_interface_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(h6x_serial_interface_example::SimpleWriteNode)
