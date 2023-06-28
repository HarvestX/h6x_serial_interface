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

#include <algorithm>
#include <array>
#include <numeric>
#include <string>
#include <utility>

#include <h6x_serial_interface/hex_handler.hpp>
#include <h6x_serial_interface/packet_state_base.hpp>
#include <h6x_serial_interface/port_handler_base.hpp>

namespace h6x_serial_interface
{
class TxPacketBase : public PacketStateBase
{
public:
  TxPacketBase()
  : PacketStateBase() {}
  virtual bool get(std::string &) noexcept = 0;
};

template<std::size_t ASCII_STX_LEN, std::size_t ASCII_DATA_LEN, std::size_t ASCII_ETX_LEN>
class TxPacket : public TxPacketBase
{
protected:
  static const size_t ASCII_STX_SIZE = ASCII_STX_LEN;
  static const size_t ASCII_DATA_SIZE = ASCII_DATA_LEN;
  static const size_t ASCII_ETX_SIZE = ASCII_ETX_LEN;  // crc + CR

  static const size_t ASCII_BUF_SIZE = ASCII_STX_SIZE + ASCII_DATA_SIZE + ASCII_ETX_SIZE;

  std::array<uint8_t, ASCII_DATA_SIZE / 2> bin_data;
  std::array<char, ASCII_BUF_SIZE> ascii_buf;

public:
  TxPacket() = delete;
  explicit TxPacket(const std::array<char, ASCII_STX_SIZE> & id)
  : TxPacketBase()
  {
    this->bin_data.fill(0);
    this->ascii_buf.fill('\0');
    std::copy(id.begin(), id.end(), this->ascii_buf.begin());
    this->ascii_buf.at(ASCII_BUF_SIZE - 1) = '\r';
  }

  bool get(std::string & ret) noexcept override
  {
    if (!this->isOK()) {
      return false;
    }

    HexHandler::bin2hex(
      this->bin_data.data(), this->bin_data.size(),
      &this->ascii_buf[ASCII_STX_SIZE], ASCII_DATA_SIZE);

    switch (this->ASCII_ETX_SIZE - sizeof('\r')) {
      case 0:
        // DO NOTHING
        break;
      case 2:
        this->setCrc8(this->ascii_buf);
        break;
      default:
        break;
    }

    this->consumed();
    ret = std::string(this->ascii_buf.data(), this->ascii_buf.size());
    return true;
  }

protected:
  inline static void setCrc8(std::array<char, ASCII_BUF_SIZE> & buf) noexcept
  {
    const uint8_t calc_crc = std::accumulate(
      buf.data(), std::prev(buf.end(), ASCII_ETX_SIZE), 0,
      [](uint8_t acc, const char & c) {return acc ^= static_cast<uint8_t>(c);});

    HexHandler::int2hex<uint8_t>(calc_crc, &buf[ASCII_BUF_SIZE - ASCII_ETX_SIZE], 2);
  }
};
}  // namespace h6x_serial_interface
