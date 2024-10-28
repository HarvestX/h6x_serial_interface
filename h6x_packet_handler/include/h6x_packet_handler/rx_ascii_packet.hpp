/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_PACKET_HANDLER_RX_ASCII_PACKET_HPP__
#define ____H6X_PACKET_HANDLER_RX_ASCII_PACKET_HPP__

#include <array>
#include <h6x_packet_handler/hex_handler.hpp>
#include <h6x_packet_handler/packet_state_base.hpp>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "h6x_packet_handler/crc_handler.hpp"

namespace h6x_packet_handler
{

template<std::size_t ASCII_STX_LEN, std::size_t ASCII_DATA_LEN, std::size_t ASCII_ETX_LEN>
class RxPacket : public PacketStateBase
{
public:
  static const size_t ASCII_STX_SIZE = ASCII_STX_LEN;
  static const size_t ASCII_DATA_SIZE = ASCII_DATA_LEN;
  static const size_t ASCII_ETX_SIZE = ASCII_ETX_LEN;  // crc

  static const size_t ASCII_BUF_SIZE = ASCII_STX_SIZE + ASCII_DATA_SIZE + ASCII_ETX_SIZE;

protected:
  std::array<uint8_t, ASCII_DATA_SIZE / 2 + ASCII_DATA_SIZE % 2> bin_data;
  std::array<uint8_t, ASCII_ETX_SIZE> crc_data;
  const std::array<char, ASCII_STX_SIZE> STX_ID;

public:
  RxPacket() = delete;
  explicit RxPacket(const std::array<char, ASCII_STX_SIZE> & id)
  : STX_ID(id)
  {
    this->bin_data.fill(0);
    this->crc_data.fill(0);
  }

  virtual bool set(const std::string & buf) noexcept {return this->setBase(buf);}

protected:
  bool setBase(const std::string & buf) noexcept
  {
    if (
      buf.size() != ASCII_BUF_SIZE || !this->checkPrefix(buf) || !this->checkCRC(buf) ||
      !this->convert(buf))
    {
      return false;
    }
    this->makeOK();
    return true;
  }

  inline bool checkPrefix(const std::string & buf) const noexcept
  {
    return std::strncmp(&buf[0], this->STX_ID.data(), ASCII_STX_SIZE) == 0;
  }

  virtual inline bool checkCRC(const std::string & buf) const noexcept
  {
    bool crc_res = false;
    switch (this->ASCII_ETX_SIZE) {
      case 0:
        crc_res = true;  // CRC Packet not contained
        break;
      case 2:
        crc_res = this->checkCrc8(buf);
        break;
      default:
        break;
    }
    return crc_res;
  }

  virtual bool convert(const std::string & buf) noexcept
  {
    HexHandler::hex2bin(
      &buf[ASCII_STX_SIZE], this->ASCII_DATA_SIZE, this->bin_data.data(), this->bin_data.size());
    std::copy(
      buf.begin() + ASCII_BUF_SIZE - ASCII_ETX_SIZE, buf.begin() + ASCII_BUF_SIZE,
      this->crc_data.begin());
    return true;
  }

  static inline bool checkCrc8(const std::string & buf) noexcept
  {
    uint8_t crc;

    if (!HexHandler::hex2int<uint8_t>(&buf[ASCII_BUF_SIZE - ASCII_ETX_SIZE], 2, crc)) {
      return false;
    }

    const uint8_t calc_crc =
      CrcHandler::crc8_ccitt(0, buf.data(), ASCII_STX_SIZE + ASCII_DATA_SIZE);

    return calc_crc == crc;
  }

  template<typename T>
  inline T get1byteData(const size_t && idx)
  {
    static_assert(sizeof(T) == 1, "Sizeof T should be 1-byte");
    assert(idx + sizeof(T) <= this->bin_data.size());

    T ret = 0;
    ret |= (this->bin_data[idx + 0] & 0xFF) << 0;
    return ret;
  }

  template<typename T>
  inline T get2byteData(const size_t && idx)
  {
    static_assert(sizeof(T) == 2, "Sizeof T should be 2-byte");
    assert(idx + sizeof(T) <= this->bin_data.size());

    T ret = 0;
    ret |= (this->bin_data[idx + 0] & 0xFF) << 8;
    ret |= (this->bin_data[idx + 1] & 0xFF) << 0;
    return ret;
  }

  template<typename T>
  inline T get4byteData(const size_t && idx)
  {
    static_assert(sizeof(T) == 4, "Sizeof T should be 4-byte");
    assert(idx + sizeof(T) <= this->bin_data.size());

    T ret = 0;
    ret |= (this->bin_data[idx + 0] & 0xFF) << 24;
    ret |= (this->bin_data[idx + 1] & 0xFF) << 16;
    ret |= (this->bin_data[idx + 2] & 0xFF) << 8;
    ret |= (this->bin_data[idx + 3] & 0xFF) << 0;

    return ret;
  }

  template<typename T>
  inline T get8byteData(const size_t && idx)
  {
    static_assert(sizeof(T) == 8, "Sizeof T should by 8-byte");
    assert(idx + sizeof(T) <= this->bin_data.size());

    T ret = 0;
    ret |= (this->bin_data[idx + 0] & 0xFF) << 56;
    ret |= (this->bin_data[idx + 1] & 0xFF) << 48;
    ret |= (this->bin_data[idx + 2] & 0xFF) << 40;
    ret |= (this->bin_data[idx + 3] & 0xFF) << 32;

    ret |= (this->bin_data[idx + 4] & 0xFF) << 24;
    ret |= (this->bin_data[idx + 5] & 0xFF) << 16;
    ret |= (this->bin_data[idx + 6] & 0xFF) << 8;
    ret |= (this->bin_data[idx + 7] & 0xFF) << 0;
  }
};
}  // namespace h6x_packet_handler
#endif
