// Copyright 2022 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "h6x_serial_interface/port_handler.hpp"


namespace h6x_serial_interface
{
bool PortHandler::configure(const std::string & dev, const int baudrate)
{
  using namespace boost::asio;  // NOLINT
  this->dev_ = dev;
  try {
    this->port_ = std::make_unique<serial_port>(this->io_);
    this->port_->open(this->dev_);
    this->port_->set_option(serial_port_base::baud_rate(baudrate));
    this->port_->set_option(serial_port_base::character_size(8));
    this->port_->set_option(serial_port_base::parity(serial_port_base::parity::none));
    this->port_->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    this->port_->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    this->port_->close();
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
    return false;
  }
  return true;
}

bool PortHandler::open()
{
  if (this->dev_.empty()) {
    RCLCPP_ERROR(this->getLogger(), "Port handler not configured");
    return false;
  }

  try {
    if (!this->port_->is_open()) {
      this->port_->open(this->dev_);
    }
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
    return false;
  }
  return true;
}

bool PortHandler::close()
{
  try {
    if (this->port_->is_open()) {
      this->port_->cancel();
      this->port_->close();
    }
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
    return false;
  }
  return true;
}

ssize_t PortHandler::read(char * const buf, const size_t size) const
{
  if (!this->port_->is_open()) {
    RCLCPP_ERROR(this->getLogger(), "%s: not opened", this->dev_.c_str());
    return -1;
  }

  try {
    return this->port_->read_some(boost::asio::buffer(buf, size));
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
  }
  return -1;
}

ssize_t PortHandler::readUntil(boost::asio::streambuf & buf, const char delimiter) const
{
  if (!this->port_->is_open()) {
    RCLCPP_ERROR(this->getLogger(), "%s: not opened", this->dev_.c_str());
    return -1;
  }

  try {
    return boost::asio::read_until(*this->port_, buf, delimiter);
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
  }
  return -1;
}


ssize_t PortHandler::write(char const * const buf, const size_t size) const
{
  if (!this->port_->is_open()) {
    RCLCPP_ERROR(this->getLogger(), "%s: not opened", this->dev_.c_str());
    return -1;
  }

  try {
    return this->port_->write_some(boost::asio::buffer(buf, size));
  } catch (const boost::system::system_error & e) {
    RCLCPP_ERROR(this->getLogger(), "%s %s", this->dev_.c_str(), e.what());
  }
  return -1;
}

const rclcpp::Logger PortHandler::getLogger() noexcept
{
  return rclcpp::get_logger("PortHandler");
}
}  // namespace h6x_serial_interface
