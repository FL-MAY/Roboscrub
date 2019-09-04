//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#pragma once

#include <ros/ros.h>
#include <launcher_msgs/LauncherAction.h>
#include <atomic>

namespace rock::scrubber::launcher {
	typedef launcher_msgs::MissionResult States;

	class ScrubberStates {
	public:
		ScrubberStates();

		void shiftState(const uint8_t&);

		void setError(uint8_t error_code) { error_ = error_code; }

		void setConfig(uint8_t config_code) { config_ = config_code; }

		uint8_t getError() const { return error_; }

		uint8_t getConfig() const { return config_; }

		uint8_t getState() const { return state_; }

		uint8_t getLastState() const { return last_state_; }

		std::string stateName(const uint8_t& state);

	private:
		std::map<uint8_t, std::string> name_;

		std::atomic<uint8_t> state_;
		std::atomic<uint8_t> last_state_;
		std::atomic<uint8_t> error_;
		std::atomic<uint8_t> config_;
		std::atomic<bool>    charging_;


	};
}

