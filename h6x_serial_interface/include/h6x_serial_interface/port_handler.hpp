/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_SERIAL_INTERFACE_PORT_HANDLER_HPP__
#define ____H6X_SERIAL_INTERFACE_PORT_HANDLER_HPP__

#include <libserial/SerialPort.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "h6x_serial_interface/port_handler_base.hpp"

namespace h6x_serial_interface
{
class PortHandler final : public PortHandlerBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(PortHandler)
  static const int NO_TIMEOUT = 0;

private:
  const std::string dev_;
  int timeout_ms_ = NO_TIMEOUT;
  LibSerial::SerialPort port_;

public:
  explicit PortHandler(const std::string &);
  ~PortHandler();
  bool configure(const int = 115200, const int = 10);
  bool open(void);
  bool close(void);
  ssize_t flashInputBuffer(void) noexcept;
  ssize_t flashOutputBuffer(void) noexcept;

  ssize_t read(char * const, const size_t) override;
  ssize_t readUntil(std::stringstream &, const char = '\r') override;
  ssize_t write(const char * const, const size_t) override;

private:
  bool checkPort(void) const noexcept;
  static const rclcpp::Logger getLogger(void) noexcept;
};
}  // namespace h6x_serial_interface
#endif
