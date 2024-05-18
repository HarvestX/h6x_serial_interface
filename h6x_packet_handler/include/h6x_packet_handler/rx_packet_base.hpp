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
class RxPacketBase : public PacketStateBase
{
public:
  RxPacketBase() : PacketStateBase() {}

  virtual bool set(const std::string &) noexcept = 0;
};
}  // namespace h6x_packet_handler
