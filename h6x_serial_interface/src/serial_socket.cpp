/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "h6x_serial_interface/serial_socket.hpp"

namespace h6x_serial_interface
{
SerialSocket::SerialSocket() : socket_{service_} {}

bool SerialSocket::open(const std::string & dev, const int baud)
{
  try {
    using boost::asio::serial_port_base;
    if (!this->socket_.is_open()) {
      this->socket_.open(dev);
      this->socket_.set_option(serial_port_base::baud_rate(baud));
      this->socket_.set_option(serial_port_base::character_size(8));
      this->socket_.set_option(serial_port_base::parity(serial_port_base::parity::none));
      this->socket_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
      this->socket_.set_option(
        serial_port_base::flow_control(serial_port_base::flow_control::none));
    }
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), e.what());
    return false;
  }
  return true;
}

bool SerialSocket::close(void)
{
  try {
    if (this->socket_.is_open()) {
      this->socket_.cancel();
      this->socket_.close();
    }
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), e.what());
    return false;
  }
  return true;
}

ssize_t SerialSocket::read(char * buf, size_t len, int timeout_ms) {
   return -1; }

const rclcpp::Logger SerialSocket::getLogger(void) noexcept
{
  return rclcpp::get_logger("SerialSocket");
}
}  // namespace h6x_serial_interface
