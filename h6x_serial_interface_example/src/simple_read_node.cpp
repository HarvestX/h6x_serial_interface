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

#include "h6x_serial_interface_example/simple_read_node.hpp"

namespace h6x_serial_interface_example
{
SimpleReadNode::SimpleReadNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("simple_read_node", options)
{
  const int baudrate = this->declare_parameter<int>("baudrate", 115200);
  const std::string dev = this->declare_parameter<std::string>("dev", "/dev/ttyUSB0");

  this->port_handler_ = std::make_unique<PortHandler>(dev);

  using namespace h6x_serial_interface;  // NOLINT
  if (!this->port_handler_->configure(baudrate)) {
    exit(EXIT_FAILURE);
  }

  if (!this->port_handler_->open()) {
    exit(EXIT_FAILURE);
  }

  using namespace std::chrono_literals;  // NOLINT
  this->read_timer_ = this->create_wall_timer(500ms, std::bind(&SimpleReadNode::onReadTimer, this));
}

SimpleReadNode::~SimpleReadNode()
{
  this->port_handler_->close();
  this->port_handler_.reset();
}

void SimpleReadNode::onReadTimer()
{
  char buf[128];
  this->port_handler_->read(buf, sizeof(buf));
  RCLCPP_INFO(this->get_logger(), "Read: %s", buf);
}
}  // namespace h6x_serial_interface_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(h6x_serial_interface_example::SimpleReadNode)
