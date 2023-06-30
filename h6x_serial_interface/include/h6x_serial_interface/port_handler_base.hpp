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

#pragma once

#include <unistd.h>
#include <stddef.h>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace h6x_serial_interface
{
class PortHandlerBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PortHandlerBase)
  RCLCPP_UNIQUE_PTR_DEFINITIONS(PortHandlerBase)

public:
  PortHandlerBase() {}

  virtual ssize_t read(char * const, const size_t) const = 0;
  virtual ssize_t readUntil(std::stringstream &, const char) const = 0;
  virtual ssize_t write(const char * const, const size_t) const = 0;
};
}  // namespace h6x_serial_interface
