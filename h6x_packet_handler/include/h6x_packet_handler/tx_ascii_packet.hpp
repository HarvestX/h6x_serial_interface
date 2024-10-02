/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_PACKET_HANDLER_TX_ASCII_PACKET_HPP__
#define ____H6X_PACKET_HANDLER_TX_ASCII_PACKET_HPP__

#include <algorithm>
#include <array>
#include <numeric>
#include <string>
#include <utility>

#include "h6x_packet_handler/crc_handler.hpp"
#include "h6x_packet_handler/hex_handler.hpp"
#include "h6x_packet_handler/tx_packet_base.hpp"

namespace h6x_packet_handler
{
template<std::size_t ASCII_STX_LEN, std::size_t ASCII_DATA_LEN, std::size_t ASCII_ETX_LEN>
class TxPacket : public TxPacketBase
{
public:
  static const size_t ASCII_STX_SIZE = ASCII_STX_LEN;
  static const size_t ASCII_DATA_SIZE = ASCII_DATA_LEN;
  static const size_t ASCII_ETX_SIZE = ASCII_ETX_LEN;

  static const size_t ASCII_BUF_SIZE = ASCII_STX_SIZE + ASCII_DATA_SIZE + ASCII_ETX_SIZE;

protected:
  std::array<uint8_t, ASCII_DATA_SIZE / 2 + ASCII_DATA_SIZE % 2> bin_data;
  std::array<char, ASCII_BUF_SIZE> ascii_buf;
  const std::string CAP_;

public:
  TxPacket() = delete;
  explicit TxPacket(const std::array<char, ASCII_STX_SIZE> & id, const std::string cap = "\r")
  : TxPacketBase(), CAP_(cap)
  {
    this->bin_data.fill(0);
    this->ascii_buf.fill('\0');
    std::copy(id.begin(), id.end(), this->ascii_buf.begin());
  }

  bool get(std::string & ret) noexcept override
  {
    if (!this->isOK()) {
      return false;
    }

    HexHandler::bin2hex(
      this->bin_data.data(), this->bin_data.size(), &this->ascii_buf[ASCII_STX_SIZE],
      ASCII_DATA_SIZE);

    switch (this->ASCII_ETX_SIZE) {
      case 0:
        // DO NOTHING
        break;
      case 2:
        this->setCrc8(this->ascii_buf);
        break;
      default:
        break;
    }

    this->consume();
    ret = std::string(this->ascii_buf.data(), this->ascii_buf.size()) + this->CAP_;
    return true;
  }

protected:
  inline static void setCrc8(std::array<char, ASCII_BUF_SIZE> & buf) noexcept
  {
    const uint8_t calc_crc =
      CrcHandler::crc8_ccitt(0, buf.data(), ASCII_STX_SIZE + ASCII_DATA_SIZE);

    HexHandler::int2hex<uint8_t>(calc_crc, &buf[ASCII_BUF_SIZE - ASCII_ETX_SIZE], 2);
  }

  template<typename T>
  inline void set1ByteData(const size_t && idx, const T & val)
  {
    static_assert(sizeof(T) == 1, "Sizeof T should be 1-byte");
    assert(idx + sizeof(T) <= this->bin_data.size());

    this->bin_data[idx + 0] = (val >> 0) & 0xFF;
  }

  template<typename T>
  inline void set2ByteData(const size_t && idx, const T & val)
  {
    static_assert(sizeof(T) == 2, "Sizeof T should be 2-byte");
    assert(idx + sizeof(T) <= this->bin_data.size());

    this->bin_data[idx + 0] = (val >> 8) & 0xFF;
    this->bin_data[idx + 1] = (val >> 0) & 0xFF;
  }

  template<typename T>
  inline void set4ByteData(const size_t && idx, const T & val)
  {
    static_assert(sizeof(T) == 4, "Sizeof T should be 4-byte");
    assert(idx + sizeof(T) <= this->bin_data.size());

    this->bin_data[idx + 0] = (val >> 24) & 0xFF;
    this->bin_data[idx + 1] = (val >> 16) & 0xFF;
    this->bin_data[idx + 2] = (val >> 8) & 0xFF;
    this->bin_data[idx + 3] = (val >> 0) & 0xFF;
  }
};
}  // namespace h6x_packet_handler
#endif
