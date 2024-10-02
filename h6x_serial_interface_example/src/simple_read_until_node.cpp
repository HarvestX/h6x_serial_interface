/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "h6x_serial_interface_example/simple_read_until_node.hpp"

#include <h6x_serial_interface/libserial_helper.hpp>

namespace h6x_serial_interface_example
{
SimpleReadUntilNode::SimpleReadUntilNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("simple_read_until_node", options),
  port_handler_(this->declare_parameter<std::string>("dev", "/dev/ttyUSB0")),
  delimiter_(this->declare_parameter<char>("delimiter", 'd'))
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
    std::chrono::milliseconds(spin_ms), std::bind(&SimpleReadUntilNode::onReadTimer, this));
}

SimpleReadUntilNode::~SimpleReadUntilNode() {this->port_handler_.close();}

void SimpleReadUntilNode::onReadTimer()
{
  std::stringstream ss;
  this->port_handler_.readUntil(ss, this->delimiter_);

  RCLCPP_INFO(this->get_logger(), "read_until [%c]: %s", this->delimiter_, ss.str().c_str());
}
}  // namespace h6x_serial_interface_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(h6x_serial_interface_example::SimpleReadUntilNode)
