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

#include "h6x_serial_interface_example/simple_write_node.hpp"

#include <h6x_serial_interface/libserial_helper.hpp>

namespace h6x_serial_interface_example
{
SimpleWriteNode::SimpleWriteNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("simple_write_node", options)
{
  const int baudrate = this->declare_parameter<int>("baudrate", 115200);
  const std::string dev = this->declare_parameter<std::string>("dev", "/dev/ttyUSB0");

  try {
    this->serial_port_.Open(dev);
  } catch (const LibSerial::OpenFailed & e) {
    RCLCPP_ERROR(this->get_logger(), "open: %s: %s", dev.c_str(), e.what());
    exit(EXIT_FAILURE);
  }

  try {
    this->serial_port_.SetBaudRate(h6x_serial_interface::getBaudrate(baudrate));
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "baudrate: %d: %s", baudrate, e.what());
    this->serial_port_.Close();
    exit(EXIT_FAILURE);
  }

  using namespace std::chrono_literals;  // NOLINT
  this->write_timer_ =
    this->create_wall_timer(500ms, std::bind(&SimpleWriteNode::onWritTimer, this));
}

SimpleWriteNode::~SimpleWriteNode() { this->serial_port_.Close(); }

void SimpleWriteNode::onWritTimer()
{
  char buf[] = "Hello World\r";
  this->serial_port_.Write(buf);
  RCLCPP_INFO(this->get_logger(), "Send: %s", buf);
}
}  // namespace h6x_serial_interface_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(h6x_serial_interface_example::SimpleWriteNode)
