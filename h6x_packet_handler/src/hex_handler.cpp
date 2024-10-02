/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "h6x_packet_handler/hex_handler.hpp"

namespace h6x_packet_handler
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
    c_out = i8_in - 10 + 'A';
  } else {
    return false;
  }
  return true;
}

size_t HexHandler::bin2hex(
  uint8_t const * const buf, const size_t buf_len, char * const hex, const size_t hex_len) noexcept
{
  if (hex_len < buf_len * 2) {
    return 0;
  }

  for (size_t i = 0; i < buf_len; ++i) {
    if (!HexHandler::hex2char(buf[i] >> 4, hex[2 * i])) {
      return 0;
    }
    if (!HexHandler::hex2char(buf[i] & 0x0F, hex[2 * i + 1])) {
      return 0;
    }
  }

  return 2 * buf_len;
}

size_t HexHandler::hex2bin(
  char const * hex, const size_t hex_len, uint8_t * buf, const size_t buf_len) noexcept
{
  uint8_t dec;

  if (buf_len < hex_len / 2 + hex_len % 2) {
    return 0;
  }

  if (hex_len % 2) {
    if (!HexHandler::char2hex(hex[0], dec)) {
      return 0;
    }

    buf[0] = dec;
    hex++;
    buf++;
  }

  for (size_t i = 0; i < hex_len / 2; ++i) {
    if (!HexHandler::char2hex(hex[2 * i], dec)) {
      return 0;
    }

    buf[i] = dec << 4;

    if (!char2hex(hex[2 * i + 1], dec)) {
      return 0;
    }

    buf[i] |= dec;
  }

  return hex_len / 2 + hex_len % 2;
}
}  // namespace h6x_packet_handler
