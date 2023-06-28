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
#include <cassert>

namespace h6x_serial_interface
{
class HexHandler
{
public:
  static bool char2hex(const char &, uint8_t &) noexcept;
  static bool hex2char(const uint8_t &, char &) noexcept;

  static size_t bin2hex(uint8_t const * const, const size_t, char * const, const size_t) noexcept;
  static size_t hex2bin(char const *, const size_t, uint8_t *, const size_t) noexcept;

  template<typename T>
  static bool hex2int(char const * const hex, const size_t hex_len, T & ret)
  {
    uint8_t dec;
    if (sizeof(T) * 2 != hex_len) {
      return false;
    }
    ret = 0;

    for (size_t i = 0; i < hex_len - 1; ++i) {
      if (!HexHandler::char2hex(hex[i], dec)) {
        return false;
      }
      ret |= 0x0F & dec;
      ret <<= 4;
    }
    if (!HexHandler::char2hex(hex[hex_len - 1], dec)) {
      return false;
    }
    ret |= 0x0F & dec;

    return true;
  }

  template<typename T>
  static bool int2hex(const T & in, char * const hex, const size_t hex_len)
  {
    uint8_t dec;
    if (sizeof(T) * 2 != hex_len) {
      return false;
    }

    for (size_t i = 0; i < hex_len; ++i) {
      dec = 0x0F & in >> (4 * (hex_len - i - 1));
      if (!HexHandler::hex2char(dec, hex[i])) {
        return false;
      }
    }

    return true;
  }
};
}  // namespace h6x_serial_interface
