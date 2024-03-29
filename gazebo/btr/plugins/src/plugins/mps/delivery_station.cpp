/***************************************************************************
 *  delivery_station.cpp - controls a delivery station mps
 *
 *  Generated: Wed Apr 22 14:32:39 2015
 *  Copyright  2015  Randolph Maaßen
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "delivery_station.h"

#include "durations.h"

#include <spdlog/spdlog.h>

using namespace gazebo;

DeliveryStation::DeliveryStation(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
: Mps(_parent, _sdf)
{
	station_ = Station::STATION_DELIVERY;
	start_server();
}

void
DeliveryStation::process_command_in()
{
	Mps::process_command_in();
	uint16_t value = uint16_t(action_id_in_.GetValue());
	if (value == 0) {
		return;
	}
	if (calculate_station_type_from_command(value) != station_) {
		return;
	}
	Operation oper = Operation(value - station_);
	if (oper == Operation::OPERATION_DELIVER) {
		deliver();
	} else {
		//SPDLOG_LOGGER_WARN(logger, "Unexpected operation {} on station {}", oper, station_);
		return;
	}
}

/** Send delivery information to the refbox and move the puck.
 * If we have a puck in the input and we received a prepare message, move the
 * puck to the selected gate and send a DELIVER command to the refbox, then
 * reset the puck and the prepared status.
 * Otherwise, do nothing.
 */
void
DeliveryStation::deliver()
{
	uint16_t slot_ = uint16_t(payload1_in_.GetValue());
	if (slot_ != 1 && slot_ != 2 && slot_ != 3) {
		SPDLOG_LOGGER_WARN(logger, "Unexpected slot__ {}", slot_);
		return;
	}
	SPDLOG_LOGGER_INFO(logger, "{} prepared to deliver on slot {}", name_, slot_);
	if (!wp_in_input_) {
		SPDLOG_LOGGER_INFO(logger, "No workpiece in machine's input");
		return;
	}
	status_busy_in_.SetValue(true);
	action_id_in_.SetValue((uint16_t)0);
	payload1_in_.SetValue((uint16_t)0);
	std::this_thread::sleep_for(deliver_duration);
	// TODO use the right gate
	wp_in_input_->SetWorldPose(get_puck_world_pose(0.3, -0.2));
	SPDLOG_LOGGER_DEBUG(logger, "Sending delivery information for puck {}", wp_in_input_->GetName());
	gazsim_msgs::WorkpieceCommand cmd_msg;
	cmd_msg.set_command(gazsim_msgs::Command::DELIVER);
	cmd_msg.set_puck_name(wp_in_input_->GetName());
	if (name_[0] == 'C') {
		cmd_msg.set_team_color(gazsim_msgs::Team::CYAN);
	} else if (name_[0] == 'M') {
		cmd_msg.set_team_color(gazsim_msgs::Team::MAGENTA);
	}
	puck_cmd_pub_->Publish(cmd_msg);
	wp_in_input_.reset();
	status_busy_in_.SetValue(false);
}
