/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_PACKET_HANDLER_PACKET_STATE_BASE_HPP__
#define ____H6X_PACKET_HANDLER_PACKET_STATE_BASE_HPP__

namespace h6x_packet_handler
{
class PacketStateBase
{
protected:
  enum class State { EMPTY, OK };

private:
  State state_;

public:
  PacketStateBase() {this->state_ = State::EMPTY;}

  inline bool isOK() {return this->state_ == State::OK;}

  inline bool isEmpty() {return this->state_ == State::EMPTY;}

  inline void consume() {this->state_ = State::EMPTY;}

protected:
  inline void makeOK()
  {
    if (this->state_ == State::OK) {
      return;
    }
    this->state_ = State::OK;
  }

  inline void makeEmpty()
  {
    if (this->state_ == State::EMPTY) {
      return;
    }
    this->state_ = State::EMPTY;
  }
};
}  // namespace h6x_packet_handler
#endif
