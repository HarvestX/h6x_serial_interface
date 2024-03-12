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

#include "h6x_packet_handler/crc_handler.hpp"

namespace h6x_packet_handler
{
const uint8_t CrcHandler::crc8_ccitt_small_table[16] = {
  0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d};

uint8_t CrcHandler::crc8_ccitt(uint8_t val, char const * const buf, size_t len)
{
  size_t i;
  const char * p = buf;

  for (i = 0; i < len; i++) {
    val ^= p[i];
    val = (val << 4) ^ crc8_ccitt_small_table[val >> 4];
    val = (val << 4) ^ crc8_ccitt_small_table[val >> 4];
  }
  return val;
}
}  // namespace h6x_packet_handler
