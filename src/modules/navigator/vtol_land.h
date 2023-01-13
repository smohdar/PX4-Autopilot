/***************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
/**
 * @file vtol_land.h
 *
 * Helper class to do a VTOL landing using a loiter down to altitude landing pattern.
 *
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"
#include "safe_point_land.hpp"

#include <dataman_client/DatamanClient.hpp>

#include <lib/mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/wind.h>

#include <px4_platform_common/module_params.h>

class VtolLand : public MissionBlock, public ModuleParams
{
public:
	VtolLand(Navigator *navigator);
	~VtolLand() = default;

	void on_activation() override;
	void on_active() override;
	void on_inactive() override;

	void setRtlApproach(RtlPosition rtl_position, loiter_point_s loiter_pos);

	rtl_time_estimate_s calc_rtl_time_estimate();

private:

	enum class vtol_land_state {
		MOVE_TO_LOITER = 0,
		LOITER_DOWN,
		TRANSITION_TO_MC,
		LAND,
		IDLE
	} _land_state;

	RtlPosition _rtl_position;
	loiter_point_s _land_approach;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_RETURN_ALT>) _param_return_alt_rel_m,
		(ParamFloat<px4::params::RTL_LOITER_RAD>) _param_rtl_loiter_rad
	)

	void set_loiter_position();

	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};		/**< home position subscription */
};
