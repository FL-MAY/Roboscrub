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

#include <thread>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <db_msgs/GetScrubberConfig.h>
#include <launcher_msgs/MissionResult.h>
#include <scrubber_msgs/SetCleanConfig.h>
#include <scrubber_msgs/SetPathConfig.h>

namespace rock::scrubber::scrubber {
	typedef launcher_msgs::MissionResult States;

	class Scrubber {
	public:
		Scrubber();

		~Scrubber() = default;

	private:
		void emergencyStop(const std_msgs::Bool::ConstPtr& msg);

		void baseVel(const geometry_msgs::Twist::ConstPtr& msg);

		void currentState(const std_msgs::UInt8::ConstPtr& msg);

		bool setPathConfig(scrubber_msgs::SetPathConfigRequest& req, scrubber_msgs::SetCleanConfigResponse&);

		bool updateZoneConfig();

		void pubVelocityLimit();

		void exeThread();

		uint8_t cur_state_;
		uint32_t map_id_;
		double  max_x_config_;
		double  max_x_base_;

		ros::Publisher                           vel_limit_;
		ros::Publisher                           pose_pub_;
		ros::Subscriber                          state_sub_;
		ros::Subscriber                          emg_sub_;
		ros::Subscriber                          base_vel_sub_;
		ros::ServiceServer                       path_config_srv_;
		ros::ServiceClient                       pause_;
		ros::ServiceClient                       get_scrubber_config_;
		ros::ServiceClient                       set_clean_config_;
		std::thread                              scrubber_thread_;
		geometry_msgs::PoseWithCovarianceStamped cur_pose_;
	};
}

