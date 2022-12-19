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

#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "h6x_serial_interface/port_handler_base.hpp"

namespace h6x_serial_interface
{
class PortHandler final : public PortHandlerBase
{
public:
  using UniquePtr = std::unique_ptr<PortHandler>;
  const int BAUDRATE;
  const std::string PORT_NAME;

private:
  int socket_fd_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;

public:
  PortHandler() = delete;
  explicit PortHandler(
    const std::string &,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr);

  bool openPort();
  void closePort();

  ssize_t getBytesAvailable() const override;
  ssize_t readPort(char * const, const size_t) const override;
  ssize_t writePort(const char * const, const size_t) const override;

private:
  bool setupPort(const speed_t);
  speed_t getCFlagBaud(const int) const noexcept;
  const rclcpp::Logger getLogger() const noexcept;
};
}  // namespace h6x_serial_interface
