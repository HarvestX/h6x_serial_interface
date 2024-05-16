/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __H6X_SERIAL_INTERFACE_SERIAL_SOCKET__
#define __H6X_SERIAL_INTERFACE_SERIAL_SOCKET__

#include <boost/asio.hpp>
#include <chrono>
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
  bool configure(const int baud = 115200, const int timeout_ms = 20);
  bool open(const std::string & dev);
  bool close(void);

  ssize_t read(char * buf, size_t len);
};
}  // namespace h6x_serial_interface

#include "serial_socket_impl.hpp"

#endif