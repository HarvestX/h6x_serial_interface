/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "h6x_serial_interface/port_handler.hpp"

#include "h6x_serial_interface/libserial_helper.hpp"

namespace h6x_serial_interface
{
PortHandler::PortHandler(const std::string & dev)
: dev_(dev)
{
  RCLCPP_INFO(this->getLogger(), dev_.c_str());
}

PortHandler::~PortHandler() {this->close();}

bool PortHandler::configure(const int baudrate, const int timeout_ms)
{
  if (!this->open()) {
    return false;
  }

  try {
    this->port_.SetBaudRate(getBaudrate(baudrate));
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->getLogger(), "baudrate [%d]: %s", baudrate, e.what());
    return false;
  }

  this->timeout_ms_ = timeout_ms;
  return true;
}

bool PortHandler::open()
{
  try {
    if (!this->port_.IsOpen()) {
      this->port_.Open(this->dev_);
    }
  } catch (const LibSerial::OpenFailed & e) {
    RCLCPP_ERROR(this->getLogger(), "open [%s]: %s", this->dev_.c_str(), e.what());
    return false;
  }

  return true;
}

bool PortHandler::close()
{
  try {
    this->port_.Close();
  } catch (const LibSerial::AlreadyOpen & e) {
    RCLCPP_WARN(this->getLogger(), "close [%s]: %s", this->dev_.c_str(), e.what());
    return false;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->getLogger(), "close [%s]: %s", this->dev_.c_str(), e.what());
    return false;
  }
  return true;
}

ssize_t PortHandler::read(char * const buf, const size_t size)
{
  if (!this->checkPort()) {
    return (ssize_t)-1;
  }

  char * buf_ptr = buf;
  try {
    while (static_cast<std::size_t>(buf_ptr - buf) < size) {
      this->port_.ReadByte(*buf_ptr, this->timeout_ms_);
      buf_ptr++;
    }
  } catch (const LibSerial::ReadTimeout & e) {
    return (ssize_t)-1;
  }

  return size;
}

ssize_t PortHandler::readUntil(std::stringstream & buf, const char delimiter)
{
  if (!this->checkPort()) {
    return (ssize_t)-1;
  }

  ssize_t size = 0;
  char c;
  try {
    while (c != delimiter) {
      this->port_.ReadByte(c, this->timeout_ms_);
      buf << c;
      size++;
    }
  } catch (const LibSerial::ReadTimeout & e) {
    return (ssize_t)-1;
  } catch (const std::runtime_error & e) {
    std::cerr << e.what() << std::endl;
    return (ssize_t)-1;
  }

  return size;
}

ssize_t PortHandler::write(char const * const buf, const size_t size)
{
  if (!this->checkPort()) {
    return (ssize_t)-1;
  }

  try {
    this->port_.Write(std::string(buf, size));
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->getLogger(), e.what());
    return (ssize_t)-1;
  }

  return static_cast<ssize_t>(size);
}

bool PortHandler::checkPort(void) const noexcept
{
  if (!this->port_.IsOpen()) {
    RCLCPP_ERROR(this->getLogger(), "port [%s] is not opened", this->dev_.c_str());
    return false;
  }

  return true;
}

bool PortHandler::flashInputBuffer(void) noexcept
{
  if (!this->checkPort()) {
    RCLCPP_ERROR(this->getLogger(), "Port is not open");
    return false;
  }

  try {
    this->port_.FlushInputBuffer();
  } catch (const std::runtime_error & e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

  return true;
}

bool PortHandler::flashOutputBuffer(void) noexcept
{
  if (!this->checkPort()) {
    RCLCPP_ERROR(this->getLogger(), "Port is not open");
    return false;
  }

  try {
    this->port_.FlushOutputBuffer();
  } catch (const std::runtime_error & e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

  return true;
}

const rclcpp::Logger PortHandler::getLogger(void) noexcept
{
  return rclcpp::get_logger("PortHandler");
}
}  // namespace h6x_serial_interface
