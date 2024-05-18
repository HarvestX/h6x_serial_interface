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

#include <h6x_serial_interface/libserial_helper.hpp>

namespace h6x_serial_interface_example
{
SimpleReadNode::SimpleReadNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("simple_read_node", options),
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
    this->serial_port_.Close();
    exit(EXIT_FAILURE);
  }

  using namespace std::chrono_literals;  // NOLINT
  this->read_timer_ = this->create_wall_timer(500ms, std::bind(&SimpleReadNode::onReadTimer, this));
}

SimpleReadNode::~SimpleReadNode() { this->serial_port_.Close(); }

void SimpleReadNode::onReadTimer()
{
  char buf[128];
  char * buf_ptr = buf;
  size_t byte_avail = serial_port_.GetNumberOfBytesAvailable();
  if (byte_avail == 0) {
    RCLCPP_ERROR(this->get_logger(), "No date");
  }
  try {
    while (buf_ptr < &buf[127] && byte_avail) {
      this->serial_port_.ReadByte(*buf_ptr, this->timeout_ms_);
      byte_avail--;
      buf_ptr++;
    }
  } catch (const LibSerial::ReadTimeout & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    return;
  }
  *buf_ptr = '\0';
  RCLCPP_INFO(this->get_logger(), "Read: %s", buf);
}
}  // namespace h6x_serial_interface_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(h6x_serial_interface_example::SimpleReadNode)
