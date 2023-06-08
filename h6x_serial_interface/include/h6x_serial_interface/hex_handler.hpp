// Copyright 2023 HarvestX Inc.
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

#include <cinttypes>
#include <cstddef>

namespace h6x_serial_interface
{
class HexHandler
{
public:
  static bool char2hex(const char &, uint8_t &) noexcept;
  static bool hex2char(const uint8_t &, char &) noexcept;

  static size_t bin2hex(uint8_t const * const, const size_t, char * const, const size_t) noexcept;
  static size_t hex2bin(char *, const size_t, uint8_t *, const size_t) noexcept;
};
}  // namespace h6x_serial_interface
