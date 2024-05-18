/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>

#include "h6x_packet_handler/packet_state_base.hpp"

namespace h6x_packet_handler
{
class TxPacketBase : public PacketStateBase
{
public:
  TxPacketBase() : PacketStateBase() {}
  virtual bool get(std::string &) noexcept = 0;
};
}  // namespace h6x_packet_handler
