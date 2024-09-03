/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_PACKET_HANDLER_HEX_HANDLER_HPP__
#define ____H6X_PACKET_HANDLER_HEX_HANDLER_HPP__

#include <cassert>
#include <cinttypes>
#include <cstddef>

namespace h6x_packet_handler
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
}  // namespace h6x_packet_handler
#endif
