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

#include "SPL07.hpp"

SPL07::SPL07(const I2CSPIDriverConfig &config, spl07::ISPL07 *interface) :
	I2CSPIDriver(config),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

SPL07::~SPL07()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	delete _interface;
}

int SPL07::calibrate()
{
	uint8_t buf[spl07::CALIBRATION_SIZE] {};
	const int ret = _interface->read(SPL07_ADDR_CAL, buf, sizeof(buf));

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	_cal = spl07::decode_calibration(buf);
	return PX4_OK;
}

int SPL07::init()
{
	uint8_t id = 0;

	// Probe before resetting: other barometers share the same I2C addresses.
	if (_interface->read(SPL07_ADDR_ID, &id, 1) != PX4_OK || id != SPL07_VALUE_ID) {
		return -EIO;
	}

	if (_interface->set_reg(SPL07_VALUE_RESET, SPL07_ADDR_RESET) != PX4_OK) {
		perf_count(_comms_errors);
		return -EIO;
	}

	// Coefficients take up to 40 ms after reset. Allow 100 ms for the sensor to become ready.
	const hrt_abstime ready_deadline = hrt_absolute_time() + 100000;
	bool ready = false;

	do {
		px4_usleep(10000);
		uint8_t status = 0;

		if (_interface->read(SPL07_ADDR_MEAS_CFG, &status, 1) != PX4_OK) {
			perf_count(_comms_errors);
			return -EIO;
		}

		if ((status & (spl07::COEF_RDY | spl07::SENSOR_RDY)) == (spl07::COEF_RDY | spl07::SENSOR_RDY)) {
			ready = true;
			break;
		}

	} while (hrt_absolute_time() < ready_deadline);

	if (!ready) {
		PX4_DEBUG("sensor or coefficients not ready");
		return -ETIMEDOUT;
	}

	const int ret = calibrate();

	if (ret != PX4_OK) {
		return ret;
	}

	// Pressure shift is required for 16x oversampling. FIFO stays disabled for direct register reads.
	if (_interface->set_reg(PRESSURE_CONFIG, SPL07_ADDR_PRS_CFG) != PX4_OK
	    || _interface->set_reg(TEMPERATURE_CONFIG, SPL07_ADDR_TMP_CFG) != PX4_OK
	    || _interface->set_reg(spl07::P_SHIFT, SPL07_ADDR_CFG_REG) != PX4_OK
	    || _interface->set_reg(spl07::CONTINUOUS_PRESSURE_TEMPERATURE, SPL07_ADDR_MEAS_CFG) != PX4_OK) {
		perf_count(_comms_errors);
		return -EIO;
	}

	ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
	return PX4_OK;
}

void SPL07::RunImpl()
{
	collect();
}

int SPL07::collect()
{
	perf_begin(_sample_perf);
	uint8_t status = 0;

	if (_interface->read(SPL07_ADDR_MEAS_CFG, &status, 1) != PX4_OK) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	// Publish only fresh pressure data, and wait for the first temperature result before compensating.
	if (!(status & spl07::PRS_RDY) || (!_temperature_ready && !(status & spl07::TMP_RDY))) {
		perf_cancel(_sample_perf);
		return -EAGAIN;
	}

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	uint8_t data[spl07::DATA_SIZE] {};

	if (_interface->read(SPL07_ADDR_DATA, data, sizeof(data)) != PX4_OK) {
		perf_count(_comms_errors);
		perf_cancel(_sample_perf);
		return -EIO;
	}

	_temperature_ready = true;
	const float pressure_scaled = spl07::decode_raw(data) / PRESSURE_SCALE;
	const float temperature_scaled = spl07::decode_raw(data + 3) / TEMPERATURE_SCALE;

	sensor_baro_s sensor_baro{};
	sensor_baro.timestamp_sample = timestamp_sample;
	sensor_baro.device_id = _interface->get_device_id();
	sensor_baro.pressure = spl07::compensate_pressure(_cal, pressure_scaled, temperature_scaled);
	sensor_baro.temperature = spl07::compensate_temperature(_cal, temperature_scaled);
	sensor_baro.error_count = perf_event_count(_comms_errors);
	sensor_baro.timestamp = hrt_absolute_time();
	_sensor_baro_pub.publish(sensor_baro);

	perf_end(_sample_perf);
	return PX4_OK;
}

void SPL07::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
