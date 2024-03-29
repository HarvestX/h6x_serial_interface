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

namespace h6x_packet_handler
{
class PacketStateBase
{
protected:
  enum class State { EMPTY, OK };

private:
  State state_;

public:
  PacketStateBase() { this->state_ = State::EMPTY; }

  inline bool isOK() { return this->state_ == State::OK; }

  inline bool isEmpty() { return this->state_ == State::EMPTY; }

  inline void consume() { this->state_ = State::EMPTY; }

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
