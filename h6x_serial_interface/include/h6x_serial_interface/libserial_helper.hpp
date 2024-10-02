/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ____H6X_SERIAL_INTERFACE_LIBSERIAL_HELPER_HPP__
#define ____H6X_SERIAL_INTERFACE_LIBSERIAL_HELPER_HPP__

#include <libserial/SerialStream.h>

namespace h6x_serial_interface
{
LibSerial::BaudRate getBaudrate(const int baud) noexcept;
}  // namespace h6x_serial_interface
#endif
