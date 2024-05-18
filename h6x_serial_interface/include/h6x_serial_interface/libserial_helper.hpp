/*
 * Copyright (c) 2024 HarvestX Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <libserial/SerialStream.h>

namespace h6x_serial_interface
{
LibSerial::BaudRate getBaudrate(const int baud) noexcept;
}  // namespace h6x_serial_interface
