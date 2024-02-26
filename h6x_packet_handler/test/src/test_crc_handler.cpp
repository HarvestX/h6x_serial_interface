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

#include <gmock/gmock.h>
#include <h6x_packet_handler/crc_handler.hpp>

using namespace h6x_packet_handler;

TEST(TestCrcHandler, crc8) {
  uint8_t crc;
  char buf[] = "#A00000001FFFFFFFF0000000000000002";
  crc = CrcHandler::crc8_ccitt(0, buf, 34);
  ASSERT_EQ(crc, 0x25);
}
