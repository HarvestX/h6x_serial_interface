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

#include <gtest/gtest.h>
#include <h6x_serial_interface/hex_handler.hpp>

using namespace h6x_serial_interface;  // NOLINT


TEST(TestHexHandler, char2hex) {
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

TEST(TestHexHandler, hex2char) {
  char res;
  ASSERT_TRUE(HexHandler::hex2char(1, res));
  ASSERT_EQ(res, '1');

  ASSERT_TRUE(HexHandler::hex2char(10, res));
  ASSERT_EQ(res, 'A');

  ASSERT_TRUE(HexHandler::hex2char(15, res));
  ASSERT_EQ(res, 'F');

  ASSERT_FALSE(HexHandler::hex2char(16, res));
}

TEST(TestHexHandler, bin2hex) {
  uint8_t bin[2] = {255, 254};
  char hex[4];

  ASSERT_EQ(HexHandler::bin2hex(bin, sizeof(bin), hex, sizeof(hex)), size_t(4));
  ASSERT_EQ(hex[0], 'F');
  ASSERT_EQ(hex[1], 'F');
  ASSERT_EQ(hex[2], 'F');
  ASSERT_EQ(hex[3], 'E');
}


TEST(TestHexHandler, hex2bin_even) {
  char hex[4] = {'F', 'F', 'F', 'E'};
  uint8_t bin[2];
  ASSERT_EQ(HexHandler::hex2bin(hex, sizeof(hex), bin, sizeof(bin)), size_t(2));
  ASSERT_EQ(bin[0], uint8_t(255));
  ASSERT_EQ(bin[1], uint8_t(254));
}

TEST(TestHexHandler, hex2bin_odd) {
  char hex[3] = { /* 'F',*/ 'F', 'F', 'E'};
  uint8_t bin[2];
  ASSERT_EQ(HexHandler::hex2bin(hex, sizeof(hex), bin, sizeof(bin)), size_t(2));
  ASSERT_EQ(bin[0], uint8_t(15));
  ASSERT_EQ(bin[1], uint8_t(254));
}
