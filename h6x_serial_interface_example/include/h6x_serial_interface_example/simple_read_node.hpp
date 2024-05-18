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

#include <libserial/SerialPort.h>

#include <rclcpp/rclcpp.hpp>

namespace h6x_serial_interface_example
{
class SimpleReadNode : public rclcpp::Node
{
private:
  const int timeout_ms_;
  LibSerial::SerialPort serial_port_;
  rclcpp::TimerBase::SharedPtr read_timer_;

public:
  SimpleReadNode() = delete;
  explicit SimpleReadNode(const rclcpp::NodeOptions &);
  ~SimpleReadNode();

private:
  void onReadTimer();
};
}  // namespace h6x_serial_interface_example
