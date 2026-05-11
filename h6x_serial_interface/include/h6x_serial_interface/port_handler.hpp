/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_SERIAL_INTERFACE_PORT_HANDLER_HPP__
#define ____H6X_SERIAL_INTERFACE_PORT_HANDLER_HPP__

#include <libserial/SerialPort.h>

#include <atomic>
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
  std::unique_ptr<LibSerial::SerialPort> port_;
  std::atomic<bool> io_error_{false};

public:
  explicit PortHandler(const std::string &);
  ~PortHandler() noexcept;

  bool hasIoError() const noexcept {return io_error_.load();}
  void clearIoError() noexcept {io_error_.store(false);}
  bool checkPort(void) const noexcept;
  bool configure(const int = 115200, const int = 10);
  bool open(void);
  bool close(void);
  bool flashInputBuffer(void) noexcept;
  bool flashOutputBuffer(void) noexcept;

  ssize_t read(char * const, const size_t) override;
  ssize_t readUntil(std::stringstream &, const char = '\r') override;
  ssize_t write(const char * const, const size_t) override;

private:
  static const rclcpp::Logger getLogger(void) noexcept;
};
}  // namespace h6x_serial_interface
#endif
