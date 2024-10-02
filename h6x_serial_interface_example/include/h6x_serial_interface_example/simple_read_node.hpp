/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_SERIAL_INTERFACE_EXAMPLE_SIMPLE_READ_NODE_HPP__
#define ____H6X_SERIAL_INTERFACE_EXAMPLE_SIMPLE_READ_NODE_HPP__

#include <h6x_serial_interface/h6x_serial_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace h6x_serial_interface_example
{
class SimpleReadNode : public rclcpp::Node
{
private:
  using PortHandler = h6x_serial_interface::PortHandler;
  PortHandler port_handler_;
  rclcpp::TimerBase::SharedPtr read_timer_;

public:
  SimpleReadNode() = delete;
  explicit SimpleReadNode(const rclcpp::NodeOptions &);
  ~SimpleReadNode();

private:
  void onReadTimer();
};
}  // namespace h6x_serial_interface_example
#endif
