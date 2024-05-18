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

#include "h6x_serial_interface_example/simple_read_until_node.hpp"

#include <h6x_serial_interface/libserial_helper.hpp>

namespace h6x_serial_interface_example
{
SimpleReadUntilNode::SimpleReadUntilNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("simple_read_until_node", options),
  timeout_ms_(this->declare_parameter<int>("timeout_ms", 100))
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
    exit(EXIT_FAILURE);
  }

  using namespace std::chrono_literals;  // NOLINT
  this->read_timer_ =
    this->create_wall_timer(20ms, std::bind(&SimpleReadUntilNode::onReadTimer, this));
}

SimpleReadUntilNode::~SimpleReadUntilNode() { this->serial_port_.Close(); }

void SimpleReadUntilNode::onReadTimer()
{
  std::vector<char> buf;
  char c;
  buf.reserve(serial_port_.GetNumberOfBytesAvailable());
  try {
    while (c != '\r') {
      this->serial_port_.ReadByte(c, this->timeout_ms_);
      buf.emplace_back(c);
    }
  } catch (const LibSerial::ReadTimeout & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    return;
  }
  buf.emplace_back('\0');

  RCLCPP_INFO(this->get_logger(), "Read: %s", buf.data());
}
}  // namespace h6x_serial_interface_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(h6x_serial_interface_example::SimpleReadUntilNode)
