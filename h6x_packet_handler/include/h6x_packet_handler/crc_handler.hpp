/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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
