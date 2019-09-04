//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************


#include "scrubber/Scrubber.h"

namespace rock::scrubber::scrubber {

	Scrubber::Scrubber() {
		ros::NodeHandle n;
		ros::NodeHandle private_nh("~");
		emg_sub_             = n.subscribe("base/emergency_stop", 5, &Scrubber::emergencyStop, this);
		base_vel_sub_        = n.subscribe("base/max_volocity", 5, &Scrubber::baseVel, this);
		state_sub_           = n.subscribe("launcher/state", 1, &Scrubber::currentState, this);
		pause_               = n.serviceClient<std_srvs::Empty>("launcher/pause");
		get_scrubber_config_ = n.serviceClient<db_msgs::GetScrubberConfig>("scrubber_database/getScrubberConfig");
		set_clean_config_    = n.serviceClient<scrubber_msgs::SetCleanConfig>("base/setCleanConfig");
		path_config_srv_     = n.advertiseService("setPathConfig", &Scrubber::setPathConfig, this);
		vel_limit_           = private_nh.advertise<geometry_msgs::Twist>("vel_limit", 5);
		pose_pub_            = private_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 5);
		scrubber_thread_.swap(*new std::thread(&Scrubber::exeThread, this));
	}

	void Scrubber::baseVel(const geometry_msgs::Twist::ConstPtr& msg) {
		max_x_base_ = msg->linear.x;

	}

	void Scrubber::emergencyStop(const std_msgs::Bool::ConstPtr& msg) {
		if (msg->data) { //Emergency Stop Event
			std_srvs::Empty empty;
			pause_.call(empty);
		}
	}

	void Scrubber::pubVelocityLimit() {
		geometry_msgs::Twist limit;
		limit.linear.x  = max_x_config_ > max_x_base_ ? max_x_base_ : max_x_config_;
		vel_limit_.publish(limit);
	}

	void Scrubber::exeThread() {
		tf2_ros::Buffer                 tfBuffer;
		tf2_ros::TransformListener      tfListener(tfBuffer);
		geometry_msgs::TransformStamped transformStamped;
		ros::Rate                       r(5);
		while (ros::ok()) {
			//Update current pose
			if (cur_state_ == States::AUTOMATIC || cur_state_ == States::MAPPING ||
			    cur_state_ == States::TRACKING) {
				try {
					transformStamped = tfBuffer.lookupTransform("map", "base_link",
					                                            ros::Time(0), ros::Duration(0.2));
					cur_pose_.header                = transformStamped.header;
					cur_pose_.pose.pose.position.x  = transformStamped.transform.translation.x;
					cur_pose_.pose.pose.position.y  = transformStamped.transform.translation.y;
					cur_pose_.pose.pose.orientation = transformStamped.transform.rotation;

					pose_pub_.publish(cur_pose_);
				} catch (tf2::TransformException& ex) {
					ROS_WARN("%s", ex.what());
				}
			}

			if(cur_state_ == States::AUTOMATIC){

				pubVelocityLimit();
			}
			r.sleep();
		}
	}

	void Scrubber::currentState(const std_msgs::UInt8::ConstPtr& msg) {
		cur_state_ = msg->data;
	}

	bool Scrubber::setPathConfig(scrubber_msgs::SetPathConfigRequest& req, scrubber_msgs::SetCleanConfigResponse&) {
		db_msgs::GetScrubberConfig config;
		scrubber_msgs::SetCleanConfig clean_config;
		config.request.config_id = req.config_id;
		if (get_scrubber_config_.call(config)) {
			clean_config.request.vacuum = config.response.vacuum;
			clean_config.request.squeegee = config.response.squeegee;
			clean_config.request.flow = config.response.flow;
			clean_config.request.brush = config.response.brush;
			return set_clean_config_.call(clean_config);
		}

		return false;
	}
}