//*********************************************************
//
// Copyright (c) 2019 Roborock. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

#include "launcher_state_machine/StateMachine.h"

namespace rock::scrubber::launcher {
	StateMachine::StateMachine() : mission_(nullptr), np_(nullptr),
	                               scrubber_state_(new ScrubberStates()),
	                               node_state_(States::IDLE),
	                               as_(new LauncherActionServer(ros::NodeHandle(), "LauncherServer",
	                                                            boost::bind(&StateMachine::executeCallback, this, _1),
	                                                            false)) {
		//Services for pause and resume
		ros::NodeHandle private_nh("~");
		pause_srv_   = private_nh.advertiseService("pause", &StateMachine::pauseService, this);
		resume_srv_  = private_nh.advertiseService("resume", &StateMachine::resumeService, this);
		start_srv_   = private_nh.advertiseService("start", &StateMachine::startService, this);
		prepare_srv_ = private_nh.advertiseService("prepare", &StateMachine::prepareService, this);
		state_pub_   = private_nh.advertise<std_msgs::UInt8>("scrubber_state", 1);
		//Subscribe pause topic
		ros::NodeHandle n;
		pause_sub_ = n.subscribe("operator/pause", 5, &StateMachine::pauseCallback, this);
		pose_sub_  = n.subscribe("amcl_pose", 5, &StateMachine::poseCallback, this);
		//Start server & standby thread
		as_->start();
		standby_.swap(*new std::thread(&StateMachine::standbyThread, this));
	}

	void StateMachine::executeCallback(const launcher_msgs::LauncherGoalConstPtr& goal) {
		if (!goal) {
			ROS_ERROR("Receive an empty Goal!");
			return;
		}

		ROS_INFO("Received new goal: %d", goal->mission.type);

		pre_goal_ = cur_goal_;
		cur_goal_ = *goal;
		//Generate goal depends on mission type
		mission_  = nullptr;
		scrubber_state_->shiftState(States::PREPARING);
		as_->publishFeedback(getFeedback());
		if (goal->mission.type == Actions::AUTOMATIC && node_state_ == States::AUTOMATIC) {
			mission_ = std::make_unique<Automatic>(scrubber_state_, goal->mission.id);
		} else if (goal->mission.type == Actions::MAPPING && node_state_ == States::MAPPING) {
			mission_ = std::make_unique<Mapping>(scrubber_state_);
		} else {
			ROS_ERROR("Unknown mission type :%d", goal->mission.type);
			as_->setAborted(getResult(), "Aborted because mission type is not valid");
			return;
		}

		if (mission_) {
			mission_->start();
		}

		ros::NodeHandle n;
		ros::Rate       r(20);

		while (n.ok()) {
			if (as_->isPreemptRequested()) {//Interrupt event
				mission_->cancel();
				ROS_INFO("Goal cancelled");
				while (!mission_->isDone()) {
					//Wait for the thread finish its job
					r.sleep();
				}
				mission_.reset(nullptr);
				as_->setPreempted(getResult(), "Current goal is Preempted");

				//Check abnormal interrupt
				if (as_->isNewGoalAvailable()) {
					ROS_ERROR("New goal interrupted, this should restricted by app");
					as_->acceptNewGoal();
					as_->setAborted(getResult(), "Abort because illegal interrupt");
				}

				return;
			}

			if (mission_->isDone()) {
				if (mission_->isSuccess()) {
					as_->setSucceeded(getResult(), "Complete");
				} else {
					as_->setAborted(getResult(), "Failed");
				}

				mission_.reset(nullptr);
				return;
			}

			as_->publishFeedback(getFeedback());
			r.sleep();
		}
	}


	launcher_msgs::LauncherResult StateMachine::getResult() {
		launcher_msgs::LauncherResult result;
		result.result.state        = scrubber_state_->getState();
		result.result.id           = cur_goal_.mission.id;
		result.result.error_code   = scrubber_state_->getError();
		result.result.clean_config = scrubber_state_->getConfig();//TODO: matching config id
		return result;
	}

	launcher_msgs::LauncherFeedback StateMachine::getFeedback() {
		launcher_msgs::LauncherFeedback feedback;
		feedback.state_feedback.state        = scrubber_state_->getState();
		feedback.state_feedback.id           = cur_goal_.mission.id;
		feedback.state_feedback.error_code   = scrubber_state_->getError();
		feedback.state_feedback.clean_config = scrubber_state_->getConfig(); //TODO: matching config id
		return feedback;
	}


	bool StateMachine::pauseService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
		if (!mission_) return false;

		mission_->pause();
		return true;
	}

	bool StateMachine::resumeService(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
		if (!mission_) return false;

		mission_->resume();
		return true;
	}

	bool StateMachine::startService(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) {
		if (!mission_) {
			return false;
		}
		mission_->start();
	}

	void StateMachine::pauseCallback(const std_msgs::Bool::ConstPtr& msg) {
		ROS_INFO("Pause callback");
		if (msg->data == pause_) return;
		pause_ = msg->data;
		std_srvs::EmptyRequest  req;
		std_srvs::EmptyResponse resp;
		if (pause_) {
			pauseService(req, resp);
			return;
		}

		resumeService(req, resp);
	}

	bool
	StateMachine::prepareService(launcher_msgs::PrepareNodeRequest& req, launcher_msgs::PrepareNodeResponse& resp) {
		if (node_state_ == req.target_state) return true;
		auto success = false;
		if (node_state_ == States::IDLE) {//Nothing launched yet
			if (req.target_state == States::AUTOMATIC || req.target_state == States::MAPPING
			    || req.target_state == States::TRACKING || req.target_state == States::MANUAL) {
				success = prepareState(req.target_state);
			}
		} else {
			if (req.target_state == States::IDLE) { //Shut down launched node
				success = prepareState(req.target_state);
			} else { //Transit to other model
				if (mission_) {
					mission_->cancel();
				}
				prepareState(States::IDLE);
				success = prepareState(req.target_state);
			}
		}

		return success;
	}

	bool StateMachine::prepareState(uint8_t state) {
		if (np_ != nullptr) {
			if (state != States::IDLE) {
				ROS_ERROR("Only IDLE can shut down current running sub nodes");
				return false;
			}
			if (node_state_ == States::AUTOMATIC) {
				FILE* kp = popen("rosnode kill /amcl /rviz /move_base_flex /mbf_state_machine /speed_ratio", "r");
				pclose(kp);
				pclose(np_);
				node_state_ = States::IDLE;
				scrubber_state_->shiftState(States::IDLE);
				np_ = nullptr;
				return true;
			} else if (node_state_ == States::MAPPING) {
				FILE* kp = popen("rosnode kill /cartographer_node /cartographer_occupancy_grid_node /rviz ", "r");
				pclose(kp);
				pclose(np_);
				node_state_ = States::IDLE;
				scrubber_state_->shiftState(States::IDLE);
				np_ = nullptr;
				return true;
			} else {
				ROS_ERROR("Unknown node state %d", node_state_);
				return false;
			}
		}

		if (state == States::AUTOMATIC) {
			tf2::Quaternion qt(cur_pose_.pose.pose.orientation.x, cur_pose_.pose.pose.orientation.y,
			                   cur_pose_.pose.pose.orientation.z, cur_pose_.pose.pose.orientation.w);
			double roll, pitch,last_yaw;
			tf2::Matrix3x3(qt).getRPY(roll, pitch, last_yaw);
			double          last_x    = cur_pose_.pose.pose.position.x;
			double          last_y    = cur_pose_.pose.pose.position.y;
			std::string     navigator = "roslaunch navigator navigator.launch ";
			navigator += "initial_pose_x:=" + std::to_string(last_x) + " initial_pose_y:="
			             + std::to_string(last_y) + " initial_pose_a:="+std::to_string(last_yaw);
			np_                       = popen(navigator.c_str(), "r");
			node_state_               = States::AUTOMATIC;
			scrubber_state_->shiftState(States::AUTOMATIC);
		} else if (state == States::MAPPING) {
			np_         = popen("roslaunch mapper mapper.launch", "r");
			node_state_ = States::MAPPING;
			scrubber_state_->shiftState(States::MAPPING);
			cur_pose_ = {};
			cur_pose_.pose.pose.orientation.w = 1;
		} else {
			ROS_ERROR("Unknown state %s", scrubber_state_->stateName(state).c_str());
			return false;
		}

		return true;
	}

	void StateMachine::standbyThread() {
		ros::Rate       r(20);
		std_msgs::UInt8 state_msg;
		while (ros::ok()) {
			//Publish state
			state_msg.data = scrubber_state_->getState();
			state_pub_.publish(state_msg);
			//TODO: Check error

			//TODO: Charging
			r.sleep();
		}
	}

	void StateMachine::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
		ROS_INFO("Got amcl pose");
		cur_pose_ = *msg;
	}

	bool StateMachine::relocation(amcl::RectPara area) {
		if (!ros::service::waitForService("range_localization", ros::Duration(2))) {
			return false;
		}
		relocated_ = ros::service::call("range_localization", area);
		ROS_INFO("%d ; %.2f %.2f %.2f %.2f", relocated_, area.request.rect_max_x,
		         area.request.rect_min_x, area.request.rect_max_y, area.request.rect_min_y);
		return relocated_;
	}
}//namespace launcher_state_machine