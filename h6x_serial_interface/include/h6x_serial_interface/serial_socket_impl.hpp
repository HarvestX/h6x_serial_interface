/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __H6X_SERIAL_INTERFACE_SERIAL_SOCKET_IMPL__
#define __H6X_SERIAL_INTERFACE_SERIAL_SOCKET_IMPL__

#include "serial_socket.hpp"

namespace h6x_serial_interface
{
bool SerialSocket::configure(const int baud, const int timeout_ms)
{
  using boost::asio::serial_port_base;
  return true;
}

bool SerialSocket::open(const std::string & dev) { return true; }

bool SerialSocket::close(void) { return true; }

ssize_t SerialSocket::read(char * buf, size_t len) { return -1; }
}  // namespace h6x_serial_interface

#endif