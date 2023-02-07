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
PortHandler::PortHandler(
  const std::string & port_name,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger,
  const int baudrate)
: BAUDRATE(baudrate),
  PORT_NAME(port_name),
  socket_fd_(-1),
  logging_interface_(logger)
{
}

bool PortHandler::openPort()
{
  const speed_t cflag_baud = this->getCFlagBaud(this->BAUDRATE);
  this->closePort();
  if (cflag_baud <= 0) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set baudrate: %d", this->BAUDRATE);
    return false;
  }

  return this->setupPort(cflag_baud);
}

void PortHandler::closePort()
{
  if (this->socket_fd_ != -1) {
    close(this->socket_fd_);
  }
  this->socket_fd_ = -1;
}

bool PortHandler::setupPort(const speed_t cflag_baud)
{
  struct termios new_tio;
  this->socket_fd_ =
    open(this->PORT_NAME.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (socket_fd_ < 0) {
    RCLCPP_ERROR(
      this->getLogger(), "open(%s) failed: %s", this->PORT_NAME.c_str(),
      strerror(errno));
    return false;
  }

  // Clear struct for new port settings
  bzero(&new_tio, sizeof(new_tio));

  new_tio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
  new_tio.c_iflag = IGNPAR;
  new_tio.c_oflag = 0;
  new_tio.c_lflag = 0;
  new_tio.c_cc[VTIME] = 0;
  new_tio.c_cc[VMIN] = 0;

  // Clean buffer and activate settings
  if (tcflush(this->socket_fd_, TCIFLUSH) == -1) {
    RCLCPP_ERROR(this->getLogger(), "tcflush() failed: %s", strerror(errno));
    return false;
  }
  if (tcsetattr(this->socket_fd_, TCSANOW, &new_tio) == -1) {
    RCLCPP_ERROR(this->getLogger(), "tcsetattr() failed: %s", strerror(errno));
    return false;
  }
  return true;
}

ssize_t PortHandler::getBytesAvailable() const
{
  int bytes_available;
  const int result = ioctl(this->socket_fd_, FIONREAD, &bytes_available);
  if (result == -1) {
    RCLCPP_ERROR(this->getLogger(), "ioctl() failed %s", strerror(errno));
    return -1;
  }
  return static_cast<ssize_t>(bytes_available);
}

ssize_t PortHandler::readPort(char * packet, const size_t length) const
{
  const ssize_t ret = read(this->socket_fd_, packet, length);
  if (ret == -1) {
    RCLCPP_ERROR(this->getLogger(), "read() failed: %s", strerror(errno));
  }
  return ret;
}

ssize_t PortHandler::writePort(const char * packet, const size_t length) const
{
  const ssize_t ret = write(this->socket_fd_, packet, length);
  if (ret == -1) {
    RCLCPP_ERROR(this->getLogger(), "write failed: %s", strerror(errno));
  }
  return ret;
}

speed_t PortHandler::getCFlagBaud(const int baudrate) const noexcept
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}

const rclcpp::Logger PortHandler::getLogger() const noexcept
{
  return this->logging_interface_->get_logger();
}
}  // namespace h6x_serial_interface
