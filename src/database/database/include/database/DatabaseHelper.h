//
// Created by longyue on 19-7-31.
//

#pragma once

#include <ros/ros.h>
#include <sqlite3.h>
#include <vector>
#include <std_srvs/Empty.h>
#include <std_msgs/UInt32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <db_msgs/GetPlan.h>
#include <db_msgs/GetTask.h>
#include <db_msgs/Rename.h>
#include <db_msgs/AddPlan.h>
#include <db_msgs/Delete.h>
#include <db_msgs/UpdatePlan.h>
#include <db_msgs/GetScrubberConfig.h>
#include <db_msgs/AddMap.h>
#include <db_msgs/GetMap.h>
#include <db_msgs/UpdateMap.h>
#include <db_msgs/GetZoneInMap.h>
#include <db_msgs/GetZone.h>
#include <db_msgs/AddZone.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rock::scrubber::database {
	struct Task {
		nav_msgs::Path path;
		uint32_t       type;
		uint32_t       config;
	};

	class DatabaseHelper {
	public:
		DatabaseHelper();

		~DatabaseHelper();


	private:
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

		bool openDatabase(const char* path);

		bool closeDatabase();

		bool getPlan(db_msgs::GetPlanRequest& req, db_msgs::GetPlanResponse& resp);

		bool getTask(db_msgs::GetTaskRequest& req, db_msgs::GetTaskResponse& resp);

		bool addPlan(db_msgs::AddPlanRequest& req, db_msgs::AddPlanResponse& resp);

		bool clearOldPlan(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

		bool updatePlan(db_msgs::UpdatePlanRequest& req, db_msgs::UpdatePlanResponse& resp);

		bool getScrubberConfig(db_msgs::GetScrubberConfigRequest& req, db_msgs::GetScrubberConfigResponse& resp);

		bool addMap(db_msgs::AddMapRequest& req, db_msgs::AddMapResponse&);

		bool getMap(db_msgs::GetMapRequest& req, db_msgs::GetMapResponse& resp);

		bool renameData(db_msgs::RenameRequest& req, db_msgs::RenameResponse&);

		bool deleteData(db_msgs::DeleteRequest& req, db_msgs::DeleteResponse&);

		bool updateMap(db_msgs::UpdateMapRequest& req, db_msgs::UpdateMapResponse&);

		bool clearOldMap(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

		bool getZoneInMap(db_msgs::GetZoneInMapRequest& req, db_msgs::GetZoneInMapResponse& resp);

		bool getZone(db_msgs::GetZoneRequest& req, db_msgs::GetZoneResponse& resp);

		bool addZone(db_msgs::AddZoneRequest& req, db_msgs::AddZoneResponse&);

		int32_t sqlExec(const char* sql_state, int (* callback_fuc)(void*, int, char**, char**));

		int32_t sqlExec(const char* sql_state) {
			return sqlExec(sql_state, defaultCallBack);
		}

		static int32_t listCallBack(void* handler, int element_num, char** elements, char** col_name);

		static int32_t pathCallBack(void* handler, int element_num, char** elements, char** col_name);

		static int32_t taskCallBack(void* handler, int element_num, char** elements, char** col_name);

		static int32_t defaultCallBack(void* handler, int element_num, char** elements, char** col_name);

		ros::ServiceServer rename_srv_;
		ros::ServiceServer del_srv_;
		ros::ServiceServer get_plan_srv_;
		ros::ServiceServer get_task_srv_;
		ros::ServiceServer add_plan_srv_;
		ros::ServiceServer update_plan_srv_;
		ros::ServiceServer clear_plan_srv_;
		ros::ServiceServer get_scub_config_srv_;
		ros::ServiceServer add_map_srv_;
		ros::ServiceServer get_map_srv_;
		ros::ServiceServer update_map_srv_;
		ros::ServiceServer clear_map_srv_;
		ros::ServiceServer get_map_zones_srv_;
		ros::ServiceServer get_zone_srv_;
		ros::ServiceServer add_zone_srv_;
		ros::Subscriber    map_sub_;
		ros::Publisher     map_pub_;
		ros::Publisher     map_id_pub_;

		char* err_msg_;
		uint8_t               squeegee_, brush_, flow_, vacuum_;
		bool                  is_open_;
		std::vector<uint32_t> tasks_;
		std::vector<uint32_t> zones_;
		Task                  task_content_;
		sqlite3     * sql_db_;
		sqlite3_stmt* stmt_;
		nav_msgs::OccupancyGrid map_;
	};
}


