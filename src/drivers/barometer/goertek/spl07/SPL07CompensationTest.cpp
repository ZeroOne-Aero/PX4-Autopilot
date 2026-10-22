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

#include <gtest/gtest.h>

#include "SPL07_Compensation.hpp"

TEST(SPL07Compensation, SignedCoefficientFields)
{
	// Fixed register image with shared nibbles and signed extrema for each coefficient width.
	const uint8_t registers[] = {
		0xff, 0xf8, 0x00, 0x80, 0x00, 0x07, 0xff, 0xff, 0x80, 0x00, 0x7f,
		0xff, 0xff, 0xff, 0x00, 0x01, 0xff, 0xfe, 0x7f, 0xf8, 0x00
	};
	const auto cal = spl07::decode_calibration(registers);
	EXPECT_EQ(cal.c0, -1);
	EXPECT_EQ(cal.c1, -2048);
	EXPECT_EQ(cal.c00, -524288);
	EXPECT_EQ(cal.c10, 524287);
	EXPECT_EQ(cal.c01, -32768);
	EXPECT_EQ(cal.c11, 32767);
	EXPECT_EQ(cal.c20, -1);
	EXPECT_EQ(cal.c21, 1);
	EXPECT_EQ(cal.c30, -2);
	EXPECT_EQ(cal.c31, 2047);
	EXPECT_EQ(cal.c40, -2048);
}

TEST(SPL07Compensation, ZeroAndMinusOneCoefficients)
{
	uint8_t registers[21] {};
	auto cal = spl07::decode_calibration(registers);
	EXPECT_EQ(cal.c0, 0);
	EXPECT_EQ(cal.c1, 0);
	EXPECT_EQ(cal.c00, 0);
	EXPECT_EQ(cal.c10, 0);
	EXPECT_EQ(cal.c01, 0);
	EXPECT_EQ(cal.c11, 0);
	EXPECT_EQ(cal.c20, 0);
	EXPECT_EQ(cal.c21, 0);
	EXPECT_EQ(cal.c30, 0);
	EXPECT_EQ(cal.c31, 0);
	EXPECT_EQ(cal.c40, 0);

	for (auto &byte : registers) {
		byte = 0xff;
	}

	cal = spl07::decode_calibration(registers);
	EXPECT_EQ(cal.c0, -1);
	EXPECT_EQ(cal.c1, -1);
	EXPECT_EQ(cal.c00, -1);
	EXPECT_EQ(cal.c10, -1);
	EXPECT_EQ(cal.c01, -1);
	EXPECT_EQ(cal.c11, -1);
	EXPECT_EQ(cal.c20, -1);
	EXPECT_EQ(cal.c21, -1);
	EXPECT_EQ(cal.c30, -1);
	EXPECT_EQ(cal.c31, -1);
	EXPECT_EQ(cal.c40, -1);
}

TEST(SPL07Compensation, SignedADCResults)
{
	const uint8_t zero[] = {0, 0, 0};
	const uint8_t maximum[] = {0x7f, 0xff, 0xff};
	const uint8_t minimum[] = {0x80, 0, 0};
	const uint8_t minus_one[] = {0xff, 0xff, 0xff};
	EXPECT_EQ(spl07::decode_raw(zero), 0);
	EXPECT_EQ(spl07::decode_raw(maximum), 8388607);
	EXPECT_EQ(spl07::decode_raw(minimum), -8388608);
	EXPECT_EQ(spl07::decode_raw(minus_one), -1);
}

TEST(SPL07Compensation, FourthOrderPressureTerm)
{
	spl07::calibration_s cal{};
	cal.c40 = 3;
	EXPECT_FLOAT_EQ(spl07::compensate_pressure(cal, 2.f, 0.f), 48.f);
	EXPECT_FLOAT_EQ(spl07::compensate_pressure(cal, -2.f, 0.f), 48.f);
}

TEST(SPL07Compensation, ThirdOrderTemperatureCrossTerm)
{
	spl07::calibration_s cal{};
	cal.c31 = 3;
	EXPECT_FLOAT_EQ(spl07::compensate_pressure(cal, 2.f, 4.f), 96.f);
	EXPECT_FLOAT_EQ(spl07::compensate_pressure(cal, -2.f, 4.f), -96.f);
}

TEST(SPL07Compensation, CompleteDatasheetPolynomial)
{
	spl07::calibration_s cal{};
	cal.c00 = 100000;
	cal.c10 = 100;
	cal.c20 = -10;
	cal.c30 = 3;
	cal.c40 = -2;
	cal.c01 = 4;
	cal.c11 = -5;
	cal.c21 = 6;
	cal.c31 = -7;
	cal.c0 = 50;
	cal.c1 = -2;
	EXPECT_FLOAT_EQ(spl07::compensate_pressure(cal, 2.f, 3.f), 100038.f);
	EXPECT_FLOAT_EQ(spl07::compensate_temperature(cal, 3.f), 19.f);
	EXPECT_FLOAT_EQ(spl07::compensate_temperature(cal, -3.f), 31.f);
}
