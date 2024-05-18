/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __H6X_SERIAL_INTERFACE_SERIAL_SOCKET__
#define __H6X_SERIAL_INTERFACE_SERIAL_SOCKET__

#include <boost/asio.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <utility>

namespace h6x_serial_interface
{
class SerialSocket
{
private:
  using Clock = std::chrono::steady_clock;
  using Timer = boost::asio::basic_waitable_timer<Clock>();
  boost::asio::io_service service_;
  boost::asio::serial_port socket_;

public:
  SerialSocket();
  bool open(const std::string & dev, int baudrate = 115200);
  bool close(void);

  ssize_t read(char * buf, size_t len, int timeout_ms = -1);

private:
  static const rclcpp::Logger getLogger(void) noexcept;
};
}  // namespace h6x_serial_interface

#endif