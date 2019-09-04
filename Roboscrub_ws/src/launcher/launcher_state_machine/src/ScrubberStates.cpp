//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "launcher_state_machine/ScrubberStates.h"

namespace rock::scrubber::launcher {
	ScrubberStates::ScrubberStates()
			: charging_(false), error_(0), config_(0), state_(States::IDLE), last_state_(States::IDLE) {

		name_[States::IDLE]      = "IDLE";
		name_[States::AUTOMATIC] = "Automatic";
		name_[States::CHARGING]  = "Charging";
		name_[States::UPDATING]  = "Updating";
		name_[States::MANUAL]    = "Manual";
		name_[States::MAPPING]   = "Mapping";
		name_[States::TRACKING]  = "Tracking";
		name_[States::PAUSE]     = "Pause";
		name_[States::PREPARING] = "Preparing";
	}

	void ScrubberStates::shiftState(const uint8_t& newState) {
		uint8_t curState = state_;
		last_state_ = curState;
		state_      = newState;
		ROS_INFO("%s --> %s", stateName(last_state_).c_str(), stateName(state_).c_str());
	}

	std::string ScrubberStates::stateName(const uint8_t& state) {
		if (name_.find(state) != name_.end()) {
			return name_[state];
		}

		return "?";
	}

}