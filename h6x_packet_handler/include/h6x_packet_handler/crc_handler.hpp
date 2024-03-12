// Copyright 2024 HarvestX Inc.
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

#include <array>
#include <cinttypes>
#include <cstddef>

namespace h6x_packet_handler
{
class CrcHandler
{
public:
  static const uint8_t crc8_ccitt_small_table[16];
  static uint8_t crc8_ccitt(uint8_t val, char const * const buf, size_t len);
};
}  // namespace h6x_packet_handler
