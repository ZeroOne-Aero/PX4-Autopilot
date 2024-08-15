/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

// PX4 includes
#include <px4_platform_common/module_params.h>
#include <lib/pure_pursuit/PurePursuit.hpp>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/rover_differential_guidance_status.h>

// Standard libraries
#include <lib/pid/pid.h>
#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <math.h>

using namespace matrix;

/**
 * @brief Enum class for the different states of guidance.
 */
enum class GuidanceState {
	SPOT_TURNING, // The vehicle is currently turning on the spot.
	DRIVING,      // The vehicle is currently driving.
	STOPPED  // The vehicle is stopped.
};

/**
 * @brief Class for differential rover guidance.
 */
class RoverDifferentialGuidance : public ModuleParams
{
public:
	/**
	 * @brief Constructor for RoverDifferentialGuidance.
	 * @param parent The parent ModuleParams object.
	 */
	RoverDifferentialGuidance(ModuleParams *parent);
	~RoverDifferentialGuidance() = default;

	struct differential_setpoint {
		float throttle{0.f};
		float yaw_rate{0.f};
		bool closed_loop_yaw_rate{false};
	};

	/**
	 * @brief Compute guidance for the vehicle.
	 * @param yaw The yaw orientation of the vehicle in radians.
	 * @param actual_speed The velocity of the vehicle in m/s.
	 * @param dt The time step in seconds.
	 * @param nav_state Navigation state of the rover.
	 */
	RoverDifferentialGuidance::differential_setpoint computeGuidance(float yaw, float actual_speed,
			int nav_state);

	/**
	 * @brief Update global/ned waypoint coordinates
	 */
	void updateWaypoints();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	// uORB subscriptions
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _mission_result_sub{ORB_ID(mission_result)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};

	// uORB publications
	uORB::Publication<rover_differential_guidance_status_s> _rover_differential_guidance_status_pub{ORB_ID(rover_differential_guidance_status)};
	rover_differential_guidance_status_s _rover_differential_guidance_status{};

	// Variables
	MapProjection _global_ned_proj_ref{}; // Transform global to ned coordinates.
	GuidanceState _currentState{GuidanceState::DRIVING}; // The current state of guidance.
	PurePursuit _pure_pursuit{this}; // Pure pursuit library
	hrt_abstime _timestamp{0};
	float _max_yaw_rate{0.f};


	// Waypoints
	Vector2d _curr_pos{};
	Vector2f _curr_pos_ned{};
	Vector2d _prev_wp{};
	Vector2f _prev_wp_ned{};
	Vector2d _curr_wp{};
	Vector2f _curr_wp_ned{};
	Vector2d _next_wp{};
	Vector2d _home_position{};

	// Controllers
	PID_t _pid_heading; // The PID controller for the heading
	PID_t _pid_throttle; // The PID controller for velocity

	// Constants
	static constexpr float TURN_MAX_VELOCITY = 0.2f; // Velocity threshhold for starting the spot turn [m/s]

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RD_HEADING_P>) _param_rd_p_gain_heading,
		(ParamFloat<px4::params::RD_HEADING_I>) _param_rd_i_gain_heading,
		(ParamFloat<px4::params::RD_SPEED_P>) _param_rd_p_gain_speed,
		(ParamFloat<px4::params::RD_SPEED_I>) _param_rd_i_gain_speed,
		(ParamFloat<px4::params::RD_MAX_SPEED>) _param_rd_max_speed,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad,
		(ParamFloat<px4::params::RD_MAX_JERK>) _param_rd_max_jerk,
		(ParamFloat<px4::params::RD_MAX_ACCEL>) _param_rd_max_accel,
		(ParamFloat<px4::params::RD_MISS_SPD_DEF>) _param_rd_miss_spd_def,
		(ParamFloat<px4::params::RD_MAX_YAW_RATE>) _param_rd_max_yaw_rate,
		(ParamFloat<px4::params::RD_TRANS_TRN_DRV>) _param_rd_trans_trn_drv,
		(ParamFloat<px4::params::RD_TRANS_DRV_TRN>) _param_rd_trans_drv_trn

	)
};
