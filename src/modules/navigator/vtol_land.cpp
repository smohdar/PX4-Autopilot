/****************************************************************************
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
 * @file vtol_land.cpp
 *
 * Helper class to do a VTOL landing using a loiter down to altitude landing pattern.
 *
 */

#include "vtol_land.h"

#include <float.h>

#include "navigator.h"
#include "navigation.h"

#include "modules/dataman/dataman.h"

static constexpr float _min_loiter_time_before_land = 10.0f;

VtolLand::VtolLand(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	_rtl_position.lat = static_cast<double>(NAN);
	_rtl_position.lon = static_cast<double>(NAN);
	_land_approach.lat = static_cast<double>(NAN);
	_land_approach.lon = static_cast<double>(NAN);
	_land_approach.height_m = NAN;
}

void
VtolLand::on_activation()
{
	_global_pos_sub.update();
	_home_pos_sub.update();

	if (!PX4_ISFINITE(_rtl_position.lat) || !PX4_ISFINITE(_rtl_position.lon)) {
		// We don't have a valid rtl pposition, use the home position instead.
		_rtl_position.lat = _home_pos_sub.get().lat;
		_rtl_position.lon = _home_pos_sub.get().lon;
		_rtl_position.alt = _home_pos_sub.get().alt;
		_rtl_position.yaw = _home_pos_sub.get().yaw;
	}

	if (!PX4_ISFINITE(_rtl_position.alt)) {
		// Not a valid rtl land altitude Assume same altitude as home position.
		_rtl_position.alt = _home_pos_sub.get().alt;
	}

	if (!PX4_ISFINITE(_land_approach.lat) || !PX4_ISFINITE(_land_approach.lon)) {

		_land_approach.lat = _rtl_position.lat;
		_land_approach.lon = _rtl_position.lon;
	}

	if (!PX4_ISFINITE(_land_approach.height_m) || _land_approach.height_m <= 0) {
		_land_approach.height_m = _param_return_alt_rel_m.get();
	}

	if (!PX4_ISFINITE(_land_approach.loiter_radius_m) || fabsf(_land_approach.loiter_radius_m) <= FLT_EPSILON) {
		_land_approach.loiter_radius_m = _param_rtl_loiter_rad.get();
	}

	_land_state = vtol_land_state::MOVE_TO_LOITER;

	set_loiter_position();
}

void VtolLand::on_inactive()
{
	_global_pos_sub.update();
	_home_pos_sub.update();

}

void
VtolLand::on_active()
{
	_global_pos_sub.update();
	_home_pos_sub.update();

	if (is_mission_item_reached_or_completed()) {
		switch	(_land_state) {
		case vtol_land_state::MOVE_TO_LOITER: {
				_mission_item.altitude = _rtl_position.alt + _land_approach.height_m;
				_mission_item.lat = _land_approach.lat;
				_mission_item.lon = _land_approach.lon;
				_mission_item.nav_cmd = NAV_CMD_LOITER_TO_ALT;
				_mission_item.loiter_radius = _param_rtl_loiter_rad.get();

				_navigator->get_mission_result()->finished = false;
				_navigator->set_mission_result_updated();
				reset_mission_item_reached();

				// convert mission item to current setpoint
				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_apply_limitation(_mission_item);
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				pos_sp_triplet->next.valid = true;
				pos_sp_triplet->next.lat = _rtl_position.lat;
				pos_sp_triplet->next.lon = _rtl_position.lon;
				pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_LAND;

				pos_sp_triplet->previous.valid = false;
				pos_sp_triplet->current.yaw_valid = true;

				_land_state = vtol_land_state::LOITER_DOWN;
				break;
			}

		case vtol_land_state::LOITER_DOWN: {
				_mission_item.lat = _rtl_position.lat;
				_mission_item.lon = _rtl_position.lon;
				_mission_item.altitude = _rtl_position.alt + _land_approach.height_m;

				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
				_mission_item.vtol_back_transition = true;

				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				// set previous item location to loiter location such that vehicle tracks line between loiter
				// location and land location after exiting the loiter circle
				pos_sp_triplet->previous.lat = _land_approach.lat;
				pos_sp_triplet->previous.lon = _land_approach.lon;
				pos_sp_triplet->previous.alt = _mission_item.altitude;
				pos_sp_triplet->previous.valid = true;

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				reset_mission_item_reached();

				_land_state = vtol_land_state::TRANSITION_TO_MC;

				break;
			}

		case vtol_land_state::TRANSITION_TO_MC: {
				set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
				_mission_item.vtol_back_transition = true;

				issue_command(_mission_item);

				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				// issue_command(_mission_item);
				reset_mission_item_reached();

				_land_state = vtol_land_state::LAND;

				break;
			}

		case vtol_land_state::LAND: {
				_mission_item.nav_cmd = NAV_CMD_LAND;
				struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

				//publish_navigator_mission_item(); // for logging
				_navigator->set_position_setpoint_triplet_updated();

				// issue_command(_mission_item);
				reset_mission_item_reached();

				_land_state = vtol_land_state::IDLE;

				break;
			}

		default: {

				break;
			}
		}
	}
}

void VtolLand::setRtlApproach(RtlPosition rtl_position, loiter_point_s loiter_pos)
{
	if (!isActive()) {
		_land_approach = loiter_pos;
		_rtl_position = rtl_position;
	}
}

void
VtolLand::set_loiter_position()
{
	_mission_item.lat  = _land_approach.lat;
	_mission_item.lon = _land_approach.lon;
	_mission_item.altitude = math::max(_rtl_position.alt + _param_return_alt_rel_m.get(),
					   _global_pos_sub.get().alt);
	_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
	_mission_item.force_heading = true;
	_mission_item.autocontinue = false;
	_mission_item.time_inside = _min_loiter_time_before_land;

	_mission_item.altitude_is_relative = false;

	_mission_item.loiter_radius = _land_approach.loiter_radius_m;
	_mission_item.origin = ORIGIN_ONBOARD;

	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

	_navigator->set_can_loiter_at_sp(true);

	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

rtl_time_estimate_s VtolLand::calc_rtl_time_estimate()
{
	rtl_time_estimate_s time_estimate;
	time_estimate.valid = false;
	time_estimate.timestamp = hrt_absolute_time();

	return time_estimate;
}
