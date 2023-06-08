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

#include "h6x_serial_interface/hex_handler.hpp"

namespace h6x_serial_interface
{
bool HexHandler::char2hex(const char & c_in, uint8_t & i8_out) noexcept
{
  if (c_in >= '0' && c_in <= '9') {
    i8_out = c_in - '0';
  } else if (c_in >= 'a' && c_in <= 'f') {
    i8_out = c_in - 'a' + 10;
  } else if (c_in >= 'A' && c_in <= 'F') {
    i8_out = c_in - 'A' + 10;
  } else {
    return false;
  }

  return true;
}

bool HexHandler::hex2char(const uint8_t & i8_in, char & c_out) noexcept
{
  if (i8_in <= 9) {
    c_out = i8_in + '0';
  } else if (i8_in <= 15) {
    c_out = i8_in - 10 + 'a';
  } else {
    return false;
  }
  return true;
}

size_t HexHandler::bin2hex(
  uint8_t const * const buf, const size_t buf_len,
  char * const hex, const size_t hex_len) noexcept
{
  if (hex_len < (buf_len * 2 + 1)) {
    return 0;
  }

  for (size_t i = 0; i < buf_len; ++i) {
    if (!hex2char(buf[i] >> 4, hex[2 * i])) {
      return 0;
    }
    if (!hex2char(buf[i] & 0x0F, hex[2 * i + 1])) {
      return 0;
    }
  }

  hex[2 * buf_len] = '\0';
  return 2 * buf_len;
}
}  // namespace h6x_serial_interface
