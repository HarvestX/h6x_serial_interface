/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <boost/asio.hpp>
#include <chrono>
#include <utility>

namespace h6x_serial_interface
{
class SerialSocket
{
private:
  boost::asio::io_service service_;
  boost::asio::serial_port socket_;

public:
  bool configure(const int baud = 115200);
  bool open(const std::string & dev);
  bool close(void);
};
};  // namespace h6x_serial_interface

#include "socket_handler_impl.hpp"