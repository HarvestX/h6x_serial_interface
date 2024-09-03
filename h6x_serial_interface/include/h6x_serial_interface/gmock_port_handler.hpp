/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_SERIAL_INTERFACE_GMOCK_PORT_HANDLER_HPP__
#define ____H6X_SERIAL_INTERFACE_GMOCK_PORT_HANDLER_HPP__

#include <gmock/gmock.h>

#include <string>

#include "h6x_serial_interface/port_handler_base.hpp"

using PortHandlerBase = h6x_serial_interface::PortHandlerBase;

ACTION_P(StrCpyToArg0, str)
{
  strcpy(arg0, str);  // NOLINT
}

ACTION_P(StreamCpyToArg0, str) {
  arg0 << str;
}

class MockPortHandler : public PortHandlerBase
{
public:
  MockPortHandler()
  : PortHandlerBase() {}

  MOCK_METHOD(ssize_t, read, (char * const, const std::size_t), (override));
  MOCK_METHOD(ssize_t, readUntil, (std::stringstream &, const char), (override));
  MOCK_METHOD(ssize_t, write, (const char * const, const size_t), (override));
};
#endif
