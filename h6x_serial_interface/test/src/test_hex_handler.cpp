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
  uint8_t res = 0;
  ASSERT_TRUE(HexHandler::char2hex('1', res));
  ASSERT_EQ(res, 1);

  ASSERT_TRUE(HexHandler::char2hex('f', res));
  ASSERT_EQ(res, 15);

  ASSERT_TRUE(HexHandler::char2hex('F', res));
  ASSERT_EQ(res, 15);

  // Invalid char
  ASSERT_FALSE(HexHandler::char2hex('G', res));
}
