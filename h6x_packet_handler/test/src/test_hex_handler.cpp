/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gmock/gmock.h>

#include <h6x_packet_handler/hex_handler.hpp>

using namespace h6x_packet_handler;  // NOLINT

TEST(TestHexHandler, char2hex)
{
  uint8_t res;
  ASSERT_TRUE(HexHandler::char2hex('1', res));
  ASSERT_EQ(res, 1);

  ASSERT_TRUE(HexHandler::char2hex('f', res));
  ASSERT_EQ(res, 15);

  ASSERT_TRUE(HexHandler::char2hex('F', res));
  ASSERT_EQ(res, 15);

  // Invalid char
  ASSERT_FALSE(HexHandler::char2hex('G', res));
}

TEST(TestHexHandler, hex2char)
{
  char res;
  ASSERT_TRUE(HexHandler::hex2char(1, res));
  ASSERT_EQ(res, '1');

  ASSERT_TRUE(HexHandler::hex2char(10, res));
  ASSERT_EQ(res, 'A');

  ASSERT_TRUE(HexHandler::hex2char(15, res));
  ASSERT_EQ(res, 'F');

  ASSERT_FALSE(HexHandler::hex2char(16, res));
}

TEST(TestHexHandler, bin2hex)
{
  uint8_t bin[2] = {255, 254};
  char hex[4];

  ASSERT_EQ(HexHandler::bin2hex(bin, sizeof(bin), hex, sizeof(hex)), size_t(4));
  ASSERT_EQ(hex[0], 'F');
  ASSERT_EQ(hex[1], 'F');
  ASSERT_EQ(hex[2], 'F');
  ASSERT_EQ(hex[3], 'E');
}

TEST(TestHexHandler, hex2bin_even)
{
  char hex[4] = {'F', 'F', 'F', 'E'};
  uint8_t bin[2];
  ASSERT_EQ(HexHandler::hex2bin(hex, sizeof(hex), bin, sizeof(bin)), size_t(2));
  ASSERT_EQ(bin[0], uint8_t(255));
  ASSERT_EQ(bin[1], uint8_t(254));
}

TEST(TestHexHandler, hex2bin_odd)
{
  char hex[3] = { /* 'F',*/ 'F', 'F', 'E'};
  uint8_t bin[2];
  ASSERT_EQ(HexHandler::hex2bin(hex, sizeof(hex), bin, sizeof(bin)), size_t(2));
  ASSERT_EQ(bin[0], uint8_t(15));
  ASSERT_EQ(bin[1], uint8_t(254));
}

TEST(TestHexHandler, hex2int)
{
  char hex_1b[2] = {'F', 'F'};
  int8_t i8_ret;
  ASSERT_TRUE(HexHandler::hex2int<int8_t>(hex_1b, sizeof(hex_1b), i8_ret));
  ASSERT_EQ(i8_ret, int8_t(-1));

  uint8_t u8_ret;
  ASSERT_TRUE(HexHandler::hex2int<uint8_t>(hex_1b, sizeof(hex_1b), u8_ret));
  ASSERT_EQ(u8_ret, uint8_t(255));

  int16_t i16_ret;
  ASSERT_FALSE(HexHandler::hex2int<int16_t>(hex_1b, sizeof(hex_1b), i16_ret));

  char hex_2b[4] = {'F', 'F', 'F', 'F'};
  ASSERT_TRUE(HexHandler::hex2int<int16_t>(hex_2b, sizeof(hex_2b), i16_ret));
  ASSERT_EQ(i16_ret, int16_t(-1));

  uint16_t u16_ret;
  ASSERT_TRUE(HexHandler::hex2int<uint16_t>(hex_2b, sizeof(hex_2b), u16_ret));
  ASSERT_EQ(u16_ret, uint16_t(65535));

  int32_t i32_ret;
  ASSERT_FALSE(HexHandler::hex2int<int32_t>(hex_2b, sizeof(hex_2b), i32_ret));

  char hex_4b[8] = {'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F'};
  ASSERT_TRUE(HexHandler::hex2int<int32_t>(hex_4b, sizeof(hex_4b), i32_ret));
  ASSERT_EQ(i32_ret, int32_t(-1));

  uint32_t u32_ret;
  ASSERT_TRUE(HexHandler::hex2int<uint32_t>(hex_4b, sizeof(hex_4b), u32_ret));
  ASSERT_EQ(u32_ret, uint32_t(4294967295));
}

TEST(TestHexHandler, int2hex)
{
  int8_t in_i8 = -2;
  char hex_1b[2];
  ASSERT_TRUE(HexHandler::int2hex<int8_t>(in_i8, hex_1b, sizeof(hex_1b)));
  ASSERT_THAT(hex_1b, ::testing::ElementsAre('F', 'E'));

  uint8_t in_u8 = 254;
  ASSERT_TRUE(HexHandler::int2hex<uint8_t>(in_u8, hex_1b, sizeof(hex_1b)));
  ASSERT_THAT(hex_1b, ::testing::ElementsAre('F', 'E'));

  int16_t in_i16 = -2;
  char hex_2b[4];
  ASSERT_TRUE(HexHandler::int2hex<int16_t>(in_i16, hex_2b, sizeof(hex_2b)));
  ASSERT_THAT(hex_2b, ::testing::ElementsAre('F', 'F', 'F', 'E'));

  uint16_t in_u16 = 65534;
  ASSERT_TRUE(HexHandler::int2hex<uint16_t>(in_u16, hex_2b, sizeof(hex_2b)));
  ASSERT_THAT(hex_2b, ::testing::ElementsAre('F', 'F', 'F', 'E'));

  int32_t in_i32 = -2;
  char hex_4b[8];
  ASSERT_TRUE(HexHandler::int2hex<int32_t>(in_i32, hex_4b, sizeof(hex_4b)));
  ASSERT_THAT(hex_4b, ::testing::ElementsAre('F', 'F', 'F', 'F', 'F', 'F', 'F', 'E'));

  uint32_t in_u32 = 4294967294;
  ASSERT_TRUE(HexHandler::int2hex<uint32_t>(in_u32, hex_4b, sizeof(hex_4b)));
  ASSERT_THAT(hex_4b, ::testing::ElementsAre('F', 'F', 'F', 'F', 'F', 'F', 'F', 'E'));
}
