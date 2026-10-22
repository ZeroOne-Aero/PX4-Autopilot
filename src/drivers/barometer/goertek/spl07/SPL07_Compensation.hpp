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

#include <stdint.h>

namespace spl07
{

struct calibration_s {
	int16_t c0, c1;
	int32_t c00, c10;
	int16_t c01, c11, c20, c21, c30, c31, c40;
};

// All coefficients and ADC results are two's complement values (datasheet sections 4.6 and 7.11).
inline int32_t sign_extend(uint32_t value, unsigned bits)
{
	const int32_t sign_bit = 1 << (bits - 1);
	return static_cast<int32_t>(value ^ sign_bit) - sign_bit;
}

inline calibration_s decode_calibration(const uint8_t *buf)
{
	calibration_s cal{};
	cal.c0 = sign_extend((uint32_t(buf[0]) << 4) | (buf[1] >> 4), 12);
	cal.c1 = sign_extend((uint32_t(buf[1] & 0x0f) << 8) | buf[2], 12);
	cal.c00 = sign_extend((uint32_t(buf[3]) << 12) | (uint32_t(buf[4]) << 4) | (buf[5] >> 4), 20);
	cal.c10 = sign_extend((uint32_t(buf[5] & 0x0f) << 16) | (uint32_t(buf[6]) << 8) | buf[7], 20);
	cal.c01 = sign_extend((uint32_t(buf[8]) << 8) | buf[9], 16);
	cal.c11 = sign_extend((uint32_t(buf[10]) << 8) | buf[11], 16);
	cal.c20 = sign_extend((uint32_t(buf[12]) << 8) | buf[13], 16);
	cal.c21 = sign_extend((uint32_t(buf[14]) << 8) | buf[15], 16);
	cal.c30 = sign_extend((uint32_t(buf[16]) << 8) | buf[17], 16);
	cal.c31 = sign_extend((uint32_t(buf[18]) << 4) | (buf[19] >> 4), 12);
	cal.c40 = sign_extend((uint32_t(buf[19] & 0x0f) << 8) | buf[20], 12);
	return cal;
}

inline int32_t decode_raw(const uint8_t *buf)
{
	return sign_extend((uint32_t(buf[0]) << 16) | (uint32_t(buf[1]) << 8) | buf[2], 24);
}

inline float compensate_pressure(const calibration_s &cal, float pressure_scaled, float temperature_scaled)
{
	// Datasheet section 4.6.1, evaluated with Horner's method to retain the c40*P^4 and c31*T*P^3 terms.
	const float pressure_polynomial = cal.c10 + pressure_scaled * (cal.c20 + pressure_scaled *
					  (cal.c30 + pressure_scaled * cal.c40));
	const float temperature_polynomial = cal.c01 + pressure_scaled * (cal.c11 + pressure_scaled *
					     (cal.c21 + pressure_scaled * cal.c31));
	return cal.c00 + pressure_scaled * pressure_polynomial + temperature_scaled * temperature_polynomial;
}

inline float compensate_temperature(const calibration_s &cal, float temperature_scaled)
{
	return cal.c0 * 0.5f + cal.c1 * temperature_scaled;
}

} // namespace spl07
