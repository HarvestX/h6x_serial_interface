/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "h6x_serial_interface_example/simple_read_node.hpp"

#include <h6x_serial_interface/libserial_helper.hpp>

namespace h6x_serial_interface_example
{
SimpleReadNode::SimpleReadNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("simple_read_node", options),
  port_handler_(this->declare_parameter<std::string>("dev", "/dev/ttyUSB0"))
{
  const int baudrate = this->declare_parameter<int>("baudrate", 115200);
  const int timeout_ms = this->declare_parameter<int>("timeout_ms", 100);
  const int spin_ms = this->declare_parameter<int>("spin_ms", 1000);

  if (!this->port_handler_.configure(baudrate, timeout_ms)) {
    exit(EXIT_FAILURE);
  }

  if (!this->port_handler_.open()) {
    exit(EXIT_FAILURE);
  }

  this->read_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(spin_ms), std::bind(&SimpleReadNode::onReadTimer, this));
}

SimpleReadNode::~SimpleReadNode() {this->port_handler_.close();}

void SimpleReadNode::onReadTimer()
{
  char buf[128];
  const auto l = this->port_handler_.read(buf, 11);  // for 'Hello World'
  if (l <= 0) {
    return;
  }
  const auto s = std::string{buf, static_cast<std::size_t>(l)};
  RCLCPP_INFO(this->get_logger(), "read [%ld] %s", l, s.c_str());
}
}  // namespace h6x_serial_interface_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(h6x_serial_interface_example::SimpleReadNode)
