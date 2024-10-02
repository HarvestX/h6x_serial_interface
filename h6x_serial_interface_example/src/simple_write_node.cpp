/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "h6x_serial_interface_example/simple_write_node.hpp"

#include <h6x_serial_interface/libserial_helper.hpp>
#include <string>

namespace h6x_serial_interface_example
{
SimpleWriteNode::SimpleWriteNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("simple_write_node", options),
  port_handler_(this->declare_parameter<std::string>("dev", "/dev/ttyUSB0")),
  word_(this->declare_parameter<std::string>("word", "Hello World"))
{
  const int baudrate = this->declare_parameter<int>("baudrate", 115200);
  const int spin_ms = this->declare_parameter<int>("spin_ms", 1000);

  if (!this->port_handler_.configure(baudrate, PortHandler::NO_TIMEOUT)) {
    exit(EXIT_FAILURE);
  }

  if (!this->port_handler_.open()) {
    exit(EXIT_FAILURE);
  }

  this->write_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(spin_ms), std::bind(&SimpleWriteNode::onWritTimer, this));
}

SimpleWriteNode::~SimpleWriteNode() {this->port_handler_.close();}

void SimpleWriteNode::onWritTimer()
{
  this->port_handler_.write(this->word_.data(), this->word_.length());
  RCLCPP_INFO(this->get_logger(), "Send: %s", this->word_.c_str());
}
}  // namespace h6x_serial_interface_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(h6x_serial_interface_example::SimpleWriteNode)
