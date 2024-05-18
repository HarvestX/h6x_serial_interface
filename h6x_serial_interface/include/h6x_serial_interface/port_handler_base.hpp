/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_SERIAL_INTERFACE_PORT_HANDLER_BASE_HPP__
#define ____H6X_SERIAL_INTERFACE_PORT_HANDLER_BASE_HPP__

#include <stddef.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace h6x_serial_interface
{
class PortHandlerBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PortHandlerBase)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(PortHandlerBase)

public:
  PortHandlerBase() {}

  virtual ssize_t read(char * const, const size_t) = 0;
  virtual ssize_t readUntil(std::stringstream &, const char) = 0;
  virtual ssize_t write(const char * const, const size_t) = 0;
};
}  // namespace h6x_serial_interface
#endif
