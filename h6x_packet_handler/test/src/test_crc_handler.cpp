/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gmock/gmock.h>

#include <h6x_packet_handler/crc_handler.hpp>

using namespace h6x_packet_handler;  // NOLINT

TEST(TestCrcHandler, crc8)
{
  uint8_t crc;
  char buf[] = "#A00000001FFFFFFFF0000000000000002";
  crc = CrcHandler::crc8_ccitt(0, buf, 34);
  ASSERT_EQ(crc, 0x25);
}
