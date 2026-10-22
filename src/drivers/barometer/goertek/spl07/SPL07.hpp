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

#include "spl07.h"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>

class SPL07 : public I2CSPIDriver<SPL07>
{
public:
	SPL07(const I2CSPIDriverConfig &config, spl07::ISPL07 *interface);
	~SPL07() override;

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int init();
	void print_status() override;
	void RunImpl();

private:
	int calibrate();
	int collect();

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
	spl07::ISPL07 *_interface;
	spl07::calibration_s _cal{};
	bool _temperature_ready{false};

	// 32 Hz pressure with 16x oversampling (standard precision), 32 Hz temperature with 1x oversampling.
	// Nominal conversion time: 32 * (27.6 + 3.6) ms = 998.4 ms per second (datasheet section 7.3).
	static constexpr uint8_t PRESSURE_CONFIG = (5 << 4) | 4;
	static constexpr uint8_t TEMPERATURE_CONFIG = 5 << 4;
	static constexpr float PRESSURE_SCALE = 253952.f;
	static constexpr float TEMPERATURE_SCALE = 524288.f;
	static constexpr uint32_t SAMPLE_INTERVAL_US = 1000000 / 32;

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
};
