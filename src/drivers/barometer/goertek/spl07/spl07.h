/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "SPL07_Compensation.hpp"

#include <px4_platform_common/i2c_spi_buses.h>

#define SPL07_ADDR_DATA      0x00
#define SPL07_ADDR_PRS_CFG   0x06
#define SPL07_ADDR_TMP_CFG   0x07
#define SPL07_ADDR_MEAS_CFG  0x08
#define SPL07_ADDR_CFG_REG   0x09
#define SPL07_ADDR_RESET     0x0c
#define SPL07_ADDR_ID        0x0d
#define SPL07_ADDR_CAL       0x10

#define SPL07_VALUE_RESET   0x09
#define SPL07_VALUE_ID      0x11

namespace spl07
{

static constexpr uint8_t CALIBRATION_SIZE = 21;
static constexpr uint8_t DATA_SIZE = 6;
static constexpr uint8_t COEF_RDY = 1 << 7;
static constexpr uint8_t SENSOR_RDY = 1 << 6;
static constexpr uint8_t TMP_RDY = 1 << 5;
static constexpr uint8_t PRS_RDY = 1 << 4;
static constexpr uint8_t P_SHIFT = 1 << 2;
static constexpr uint8_t CONTINUOUS_PRESSURE_TEMPERATURE = 7;

class ISPL07
{
public:
	virtual ~ISPL07() = default;
	virtual int init() = 0;
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;
	// Read exactly len register bytes; return OK or a negative bus error.
	virtual int read(uint8_t addr, uint8_t *buf, uint8_t len) = 0;
	virtual uint32_t get_device_id() const = 0;
};

} // namespace spl07

#if defined(CONFIG_SPI)
extern spl07::ISPL07 *spl07_spi_interface(uint8_t busnum, uint32_t device, int bus_frequency, spi_mode_e spi_mode);
#endif
#if defined(CONFIG_I2C)
extern spl07::ISPL07 *spl07_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
#endif
