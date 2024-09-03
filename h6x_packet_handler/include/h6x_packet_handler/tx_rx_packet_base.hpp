/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_PACKET_HANDLER_TX_RX_PACKET_BASE_HPP__
#define ____H6X_PACKET_HANDLER_TX_RX_PACKET_BASE_HPP__

#include <array>
#include <numeric>
#include <string>
#include <utility>

#include "h6x_packet_handler/rx_packet_base.hpp"
#include "h6x_packet_handler/tx_packet_base.hpp"

namespace h6x_packet_handler
{

template<class TxPacketBase, class RxPacketBase>
class TxRxPacketBase : public PacketStateBase
{
protected:
  TxPacketBase tx_packet;
  RxPacketBase rx_packet;

public:
  TxRxPacketBase() {this->makeOK();}

  bool isWaitingResponse() {return this->isEmpty();}

  bool getTx(std::string & ret)
  {
    // While waiting the response, user cannot send any data.
    if (this->isWaitingResponse()) {
      return false;
    }

    const bool res = this->tx_packet.get(ret);
    if (res) {
      this->makeEmpty();
    }

    return res;
  }

  bool setRx(const std::string & buf)
  {
    const auto ret = this->rx_packet.set(buf);

    if (!ret) {
      return false;
    }

    this->rx_packet.consume();
    this->makeOK();
    return this->isResponseOK();
  }

  virtual bool isResponseOK() = 0;
};
}  // namespace h6x_packet_handler
#endif
