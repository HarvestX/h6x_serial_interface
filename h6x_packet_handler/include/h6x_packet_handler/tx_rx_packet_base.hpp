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

#include <array>
#include <numeric>
#include <string>
#include <utility>

#include "h6x_packet_handler/rx_packet_base.hpp"
#include "h6x_packet_handler/tx_packet_base.hpp"

namespace h6x_packet_handler
{

template <class TxPacketBase, class RxPacketBase>
class TxRxPacketBase : public PacketStateBase
{
protected:
  TxPacketBase tx_packet;
  RxPacketBase rx_packet;

public:
  TxRxPacketBase() { this->makeOK(); }

  bool isWaitingResponse() { return this->isEmpty(); }

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
