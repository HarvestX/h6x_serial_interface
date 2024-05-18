/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_SERIAL_INTERFACE_EXAMPLE_SIMPLE_WRITE_NODE_HPP__
#define ____H6X_SERIAL_INTERFACE_EXAMPLE_SIMPLE_WRITE_NODE_HPP__

#include <h6x_serial_interface/h6x_serial_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace h6x_serial_interface_example
{
class SimpleWriteNode : public rclcpp::Node
{
private:
  using PortHandler = h6x_serial_interface::PortHandler;
  PortHandler port_handler_;
  const std::string word_;

  rclcpp::TimerBase::SharedPtr write_timer_;

public:
  SimpleWriteNode() = delete;
  explicit SimpleWriteNode(const rclcpp::NodeOptions &);
  ~SimpleWriteNode();

private:
  void onWritTimer();
};
}  // namespace h6x_serial_interface_example
#endif
