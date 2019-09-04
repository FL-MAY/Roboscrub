//
// Created by longyue on 19-7-31.
//

#include "database/DatabaseHelper.h"

namespace rock::scrubber::database {
	DatabaseHelper::DatabaseHelper() : err_msg_(nullptr), is_open_(false) {
		//make this path param
		ros::NodeHandle nh("~");
		ros::NodeHandle n;
		std::string     db_path;
		nh.param("db_path", db_path,
		         std::string("/home/longyue/git_repo/RoboScrub_Nav/src/database/database/db/test1.db"));

		if (!openDatabase(db_path.c_str())) {
			ROS_ERROR("Can not open database");
		}

		tasks_               = {};
		task_content_        = {};
		map_sub_             = n.subscribe("map", 2, &DatabaseHelper::mapCallback, this);
		map_pub_             = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
		map_id_pub_          = nh.advertise<std_msgs::UInt32>("map_id", 1, true);
		rename_srv_          = nh.advertiseService("rename", &DatabaseHelper::renameData, this);
		del_srv_             = nh.advertiseService("delete", &DatabaseHelper::deleteData, this);
		get_plan_srv_        = nh.advertiseService("getPlan", &DatabaseHelper::getPlan, this);
		get_task_srv_        = nh.advertiseService("getTask", &DatabaseHelper::getTask, this);
		add_plan_srv_        = nh.advertiseService("addPlan", &DatabaseHelper::addPlan, this);
		update_plan_srv_     = nh.advertiseService("updatePlan", &DatabaseHelper::updatePlan, this);
		clear_plan_srv_      = nh.advertiseService("clearOldPlan", &DatabaseHelper::clearOldPlan, this);
		get_scub_config_srv_ = nh.advertiseService("getScrubberConfig", &DatabaseHelper::getScrubberConfig, this);
		add_map_srv_         = nh.advertiseService("addMap", &DatabaseHelper::addMap, this);
		get_map_srv_         = nh.advertiseService("getMap", &DatabaseHelper::getMap, this);
		update_map_srv_      = nh.advertiseService("updateMap", &DatabaseHelper::updateMap, this);
		clear_map_srv_       = nh.advertiseService("clearOldMaps", &DatabaseHelper::clearOldMap, this);
		get_map_zones_srv_   = nh.advertiseService("getZonesInMap", &DatabaseHelper::getZoneInMap, this);
		get_zone_srv_        = nh.advertiseService("getZone", &DatabaseHelper::getZone, this);
		add_zone_srv_        = nh.advertiseService("addZone", &DatabaseHelper::addZone, this);
	}

	DatabaseHelper::~DatabaseHelper() {
		sqlite3_finalize(stmt_);
		if (!closeDatabase()) {
			ROS_ERROR("Database not close properly");
		}
		delete err_msg_;
	}

	bool DatabaseHelper::openDatabase(const char* path) {
		if (SQLITE_OK != sqlite3_open(path, &sql_db_)) {
			ROS_ERROR("Open database error: %s", sqlite3_errmsg(sql_db_));
			sqlite3_close(sql_db_);
			return false;
		}

		is_open_ = true;
		return true;
	}

	bool DatabaseHelper::closeDatabase() {
		if (!is_open_) return false;

		if (SQLITE_OK != sqlite3_close(sql_db_)) {
			ROS_ERROR("Error in close database: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		return true;
	}

	int32_t DatabaseHelper::sqlExec(const char* sql_state, int (* callback_fuc)(void*, int, char**, char**)) {
		return sqlite3_exec(sql_db_, sql_state, callback_fuc, (void*) this, &err_msg_);
	}

	int32_t DatabaseHelper::listCallBack(void* handler, int element_num, char** elements, char** col_name) {
		if (!handler) {
			return -1;
		}

		if (element_num != 1) { // Should just got task_id  or zone_id only
			ROS_ERROR("Plan callback got unexpected data");
			return -1;
		}

		auto dp = (DatabaseHelper*) handler;
		if (strcmp(*col_name, "task_id") == 0) {
			dp->tasks_.push_back((uint32_t) strtol(*elements, nullptr, 0));
		} else if (strcmp(*col_name, "zone_id") == 0) {
			dp->zones_.push_back((uint32_t) strtol(*elements, nullptr, 0));
		} else {
			ROS_ERROR("Unknown col_name %s", *col_name);
		}

		return 0;
	}

	int32_t DatabaseHelper::pathCallBack(void* handler, int element_num, char** elements, char** col_name) {
		if (!handler) {
			return -1;
		}

		if (element_num != 3) { // Should just got x,y only
			ROS_ERROR("Path callback got unexpected data");
			return -1;
		}

		auto                       dp = (DatabaseHelper*) handler;
		geometry_msgs::PoseStamped pose;
		pose.header.stamp    = ros::Time::now();
		pose.header.frame_id = "map";

		for (int i = 0; i < element_num; i++) {
			if (strcmp(col_name[i], "x") == 0) {
				pose.pose.position.x = (uint32_t) strtol(elements[i], nullptr, 0);
			} else if (strcmp(col_name[i], "y") == 0) {
				pose.pose.position.y = (uint32_t) strtol(elements[i], nullptr, 0);
			} else if (strcmp(col_name[i], "theta") == 0) {
				tf2::Quaternion qt;
				qt.setRPY(0, 0, strtod(elements[i], nullptr));
				qt.normalize();
				pose.pose.orientation.x = qt.getX();
				pose.pose.orientation.y = qt.getY();
				pose.pose.orientation.z = qt.getZ();
				pose.pose.orientation.w = qt.getW();
			} else {
				ROS_ERROR("Unexpected col name %s in path callback", col_name[i]);
			}
		}

		dp->task_content_.path.poses.push_back(pose);
		return 0;
	}

	int32_t DatabaseHelper::taskCallBack(void* handler, int element_num, char** elements, char** col_name) {
		if (!handler) {
			return -1;
		}

		if (element_num != 2) { // Should just got config_id and type
			ROS_ERROR("Task callback got unexpected data");
			return -1;
		}

		auto dp = (DatabaseHelper*) handler;

		for (int i = 0; i < element_num; i++) {
			if (strcmp(col_name[i], "config_id") == 0) {
				dp->task_content_.config = (uint32_t) strtol(elements[i], nullptr, 0);
			} else if (strcmp(col_name[i], "type") == 0) {
				dp->task_content_.type = (uint32_t) strtol(elements[i], nullptr, 0);
			} else {
				ROS_ERROR("Unexpected col name %s in task callback", col_name[i]);
			}
		}

		return 0;
	}

	int32_t DatabaseHelper::defaultCallBack(void* handler, int element_num, char** elements, char** col_name) {
		for (int i = 0; i < element_num; i++) {
			ROS_INFO("%s = %s", col_name[i], elements[i] ? elements[i] : "NULL");
		}

		return 0;
	}

	bool DatabaseHelper::getPlan(db_msgs::GetPlanRequest& req, db_msgs::GetPlanResponse& resp) {
		ROS_INFO("Plan Requested");
		std::string sql_state;
		if (req.by_plan_name) {
			sql_state = "SELECT task_id FROM Task JOIN Plan"
			            " ON Task.plan_id = Plan.plan_id "
			            "WHERE name = '";
			sql_state += req.plan_name;
			sql_state += "' ORDER BY idx ASC;";
		} else {
			sql_state = "SELECT task_id FROM Task JOIN Plan"
			            " ON Task.plan_id = Plan.plan_id "
			            "WHERE Plan.plan_id = ";
			sql_state += std::to_string(req.plan_id) + " ORDER BY idx ASC;";
		}

		if (SQLITE_OK == sqlExec(sql_state.c_str(), listCallBack)) {
			resp.tasks = tasks_;
			tasks_.clear();// Clear task vector
			return true;
		}

		ROS_ERROR("%s", err_msg_);
		tasks_.clear();// Clear task vector
		return false;
	}

	bool DatabaseHelper::getTask(db_msgs::GetTaskRequest& req, db_msgs::GetTaskResponse& resp) {
		ROS_INFO("Task Requested");
		std::string path_state = "SELECT x, y, theta FROM PathData as P JOIN FollowPath as FP "
		                         "ON P.path_id = FP.path_id WHERE task_id = ";
		path_state += std::to_string(req.task_id);
		path_state += " ORDER BY idx asc; ";

		std::string task_state = "SELECT config_id, type FROM Task Where task_id = ";
		task_state += std::to_string(req.task_id);
		task_state += ";";

		task_content_.path.header.frame_id = "map";
		task_content_.path.header.stamp    = ros::Time::now();

		if (SQLITE_OK == sqlExec(path_state.c_str(), pathCallBack)
		    && SQLITE_OK == sqlExec(task_state.c_str(), taskCallBack)) {
			resp.path      = task_content_.path;
			resp.config_id = task_content_.config;
			resp.type      = task_content_.type;
			task_content_ = {}; //clear task
			return true;
		}

		ROS_ERROR("%s", err_msg_);
		task_content_ = {}; //clear task
		return false;
	}

	bool DatabaseHelper::addPlan(db_msgs::AddPlanRequest& req, db_msgs::AddPlanResponse& resp) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		//Adding a plan
		std::string state = "INSERT INTO PLAN(name) VALUES('" + req.plan_name + "');";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			sqlExec("ROLLBACK");
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		//Binding tasks
		for (int i = 0; i < req.ordered_tasks.size(); i++) {
			state = "INSERT INTO Task(plan_id, config_id, type, idx) VALUES(";
			state += "(SELECT plan_id from Plan WHERE name = '" + req.plan_name + "'),";
			state += std::to_string(req.ordered_tasks[i].config_id) + ",";
			state += std::to_string(req.ordered_tasks[i].type) + ",";
			state += std::to_string(i) + ");";
			if (SQLITE_OK != sqlExec(state.c_str())) { //Insert task;
				ROS_ERROR("%s", err_msg_);
				sqlExec("ROLLBACK");
				return false;
			}

			state = "INSERT INTO FollowPath VALUES((SELECT max(task_id) from Task), (SELECT path_id from Path ";
			state += "WHERE name = '" + req.ordered_tasks[i].path_name + "'));";
			if (SQLITE_OK != sqlExec(state.c_str())) { // Binding task and path;
				ROS_ERROR("%s", err_msg_);
				sqlExec("ROLLBACK");
				return false;
			}
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::updatePlan(db_msgs::UpdatePlanRequest& req, db_msgs::UpdatePlanResponse& resp) {
		//Adding plan using a template name
		std::string              new_name = req.plan_name + "_new_";
		db_msgs::AddPlanRequest  add_plan_req;
		db_msgs::AddPlanResponse add_plan_resp;
		add_plan_req.plan_name     = new_name;
		add_plan_req.ordered_tasks = req.ordered_tasks;
		if (!addPlan(add_plan_req, add_plan_resp)) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		//Rename old one
		db_msgs::RenameRequest  re_req;
		db_msgs::RenameResponse re_resp;
		re_req.type                        = db_msgs::RenameRequest::PLAN;
		re_req.old_name                    = req.plan_name;
		re_req.new_name                    = req.plan_name + "_" + std::to_string(ros::Time::now().sec);

		if (!renameData(re_req, re_resp)) {
			ROS_ERROR("Unable to save old plan name %s, plan name now is %s", req.plan_name.c_str(), new_name.c_str());
			ROS_ERROR("%s", err_msg_);
			return false;
		}
		//Set old flag
		std::string             flag_state = "UPDATE Plan SET old = 1 WHERE name = '" + re_req.new_name + "';";
		if (SQLITE_OK != sqlExec(flag_state.c_str())) {
			ROS_ERROR("Unable to set old flag");
			ROS_ERROR("%s", err_msg_);
			return false;
		}
		//Rename new plan to original name
		re_req.old_name = new_name;
		re_req.new_name = req.plan_name;
		if (!renameData(re_req, re_resp)) {
			ROS_ERROR("Unable to use plan name %s, plan name now is %s", req.plan_name.c_str(), new_name.c_str());
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		return true;
	}

	bool DatabaseHelper::clearOldPlan(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		//Deleting path binding
		std::string plan_id = "(SELECT plan_id FROM Plan WHERE old != 0)";
		std::string state   = "DELETE FROM FollowPath WHERE task_id in "
		                      "(SELECT task_id from Task WHERE plan_id in " + plan_id + ");";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		//Deleting tasks
		state = "DELETE FROM Task WHERE plan_id in " + plan_id + ";";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		//Deleting plan itself
		state = "DELETE FROM Plan WHERE old != 0;";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::getScrubberConfig(db_msgs::GetScrubberConfigRequest& req,
	                                       db_msgs::GetScrubberConfigResponse& resp) {
		std::string state = "SELECT squeegee FROM ScrubberConfig where config_id = "
		                    + std::to_string(req.config_id) + ";";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get squeegee prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.squeegee = sqlite3_column_int(stmt_, 0);

		state = "SELECT value FROM ConfigValue where value_id = (SELECT brush from ScrubberConfig "
		        "WHERE config_id = " + std::to_string(req.config_id) + ");";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get brush prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.brush = sqlite3_column_int(stmt_, 0);

		state = "SELECT value FROM ConfigValue where value_id = (SELECT flow from ScrubberConfig "
		        "WHERE config_id = " + std::to_string(req.config_id) + ");";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get flow prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.flow = sqlite3_column_int(stmt_, 0);

		state = "SELECT value FROM ConfigValue where value_id = (SELECT vacuum from ScrubberConfig "
		        "WHERE config_id = " + std::to_string(req.config_id) + ");";
		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get vacuum prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.vacuum = sqlite3_column_int(stmt_, 0);

		sqlite3_reset(stmt_);
		return true;
	}

	void DatabaseHelper::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
		map_.data   = msg->data;
		map_.header = msg->header;
		map_.info   = msg->info;
	}

	bool DatabaseHelper::addMap(db_msgs::AddMapRequest& req, db_msgs::AddMapResponse&) {
		if (map_.data.empty()) {
			ROS_ERROR("Currently no map");
			return false;
		}

		stmt_ = nullptr;
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		std::string state = "INSERT INTO Map(name, create_time) VALUES('" + req.name + "',"
		                    + std::to_string(ros::Time::now().toSec()) + ");";

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("ERROR IN PREPARE: %s", sqlite3_errmsg(sql_db_));
			sqlExec("ROLLBACK");
			return false;
		}

		sqlite3_step(stmt_);

		double      left   = map_.info.origin.position.y + map_.info.resolution * map_.info.width / 2;
		double      top    = map_.info.origin.position.x + map_.info.resolution * map_.info.height / 2;
		std::string map_id = "(SELECT map_id FROM Map where name = '" + req.name + "')";
		state = "INSERT INTO MapData (map_id, modify_time, resolution, width, height, left, top, data) VALUES ("
		        + map_id +
		        "," + std::to_string(ros::Time::now().toSec()) + "," + std::to_string(map_.info.resolution) + ","
		        + std::to_string(map_.info.width) + "," + std::to_string(map_.info.height) + ","
		        + std::to_string(left) + "," + std::to_string(top) + ", ?);";

		int32_t size = sizeof(map_.data[0]) * map_.data.size();
		ROS_INFO("Map size %d byte", size);

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("ERROR IN PREPARE: %s", sqlite3_errmsg(sql_db_));
			sqlExec("ROLLBACK");
			return false;
		}


		if (SQLITE_OK != sqlite3_bind_blob(stmt_, 1, map_.data.data(), size, SQLITE_STATIC)) {
			ROS_ERROR("ERROR IN BIND: %s", sqlite3_errmsg(sql_db_));
			sqlExec("ROLLBACK");
			return false;
		}

		/*//For saving pbstream map
		FILE* pf = nullptr;//fopen("/home/longyue/map.pbstream","rb");
		if(pf){
			fseek(pf, 0, SEEK_END);
			int32_t pbs_size = ftell(pf);
			rewind(pf);
			auto buffer = (char*) malloc(sizeof(char)*pbs_size);
			fread(buffer,1, pbs_size,pf);
			int32_t f_size = sizeof(char)*pbs_size;
			ROS_INFO("pbstrem size %d",f_size);
			if (SQLITE_OK != sqlite3_bind_blob(stmt_, 2, buffer, f_size, SQLITE_STATIC)) {
				ROS_ERROR("ERROR IN BIND: %s", sqlite3_errmsg(sql_db_));
				sqlExec("ROLLBACK");
				fclose(pf);
				free(buffer);
				return false;
			}
			fclose(pf);
			free(buffer);
		}
		*/
		sqlite3_step(stmt_);

		sqlite3_reset(stmt_);
		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::getMap(db_msgs::GetMapRequest& req, db_msgs::GetMapResponse& resp) {
		std::string map_id = req.by_map_name ? "SELECT map_id FROM Map WHERE name = '" + req.map_name + "'"
		                                     : std::to_string(req.map_id);
		std::string state  = "SELECT resolution, width, height, left, top, data FROM MapData WHERE map_id = (" + map_id
		                     + ");";
		if (req.by_map_name) {
			if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, map_id.c_str(), map_id.length(), &stmt_, nullptr)) {
				ROS_ERROR("Error in get map_id prepare: %s", sqlite3_errmsg(sql_db_));
				return false;
			}
			sqlite3_step(stmt_);
			resp.map_id = sqlite3_column_int(stmt_, 0);

		} else {
			resp.map_id = req.map_id;
		}

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get map prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);

		resp.map.header.stamp              = ros::Time::now();
		resp.map.header.frame_id           = "map";
		resp.map.info.resolution           = sqlite3_column_double(stmt_, 0);
		resp.map.info.width                = sqlite3_column_int(stmt_, 1);
		resp.map.info.height               = sqlite3_column_int(stmt_, 2);
		resp.map.info.origin.position.y    = sqlite3_column_double(stmt_, 3)
		                                     - resp.map.info.resolution * resp.map.info.width / 2;
		resp.map.info.origin.position.x    = sqlite3_column_double(stmt_, 4)
		                                     - resp.map.info.resolution * resp.map.info.height / 2;
		resp.map.info.origin.orientation.w = 1;

		uint32_t size     = resp.map.info.width * resp.map.info.height;
		auto     raw_data = (const int8_t*) sqlite3_column_blob(stmt_, 5);

		resp.map.data = {raw_data, raw_data + size};

		std_msgs::UInt32 cur_map_id;
		cur_map_id.data = resp.map_id;
		map_pub_.publish(resp.map);
		map_id_pub_.publish(cur_map_id);

		sqlite3_reset(stmt_);
		return true;
	}

	bool DatabaseHelper::renameData(db_msgs::RenameRequest& req, db_msgs::RenameResponse&) {
		std::string state = "UPDATE " + req.type + " SET name = '" + req.new_name + "' WHERE name = '" + req.old_name
		                    + "';";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("Error in rename %s: %s", req.type.c_str(), sqlite3_errmsg(sql_db_));
			return false;
		}

		return true;
	}

	bool DatabaseHelper::deleteData(db_msgs::DeleteRequest& req, db_msgs::DeleteResponse&) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		db_msgs::Rename rename;
		rename.request.type     = req.type;
		rename.request.old_name = req.name;
		rename.request.new_name = req.name + "_" + std::to_string(ros::Time::now().sec);
		if (!renameData(rename.request, rename.response)) {
			sqlExec("ROLLBACK");
			return false;
		}

		std::string state = "UPDATE " + req.type + " SET old = 1 WHERE name = '" + rename.request.new_name + "';";
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("Error in set %s old %s", req.type.c_str(), err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::updateMap(db_msgs::UpdateMapRequest& req, db_msgs::UpdateMapResponse& resp) {
		//Adding current map as temp
		std::string     temp_name = req.name + "_temp_";
		db_msgs::AddMap temp_map;
		temp_map.request.name = temp_name;
		if (!addMap(temp_map.request, temp_map.response)) {
			ROS_INFO("Update map Failed in creating temp map");
			return false;
		}

		//Keep create time
		db_msgs::Delete del;
		std::string     state = "UPDATE Map SET create_time = (SELECT create_time from Map WHERE name = '"
		                        + req.name + "') WHERE name = '" + temp_name + "';";
		del.request.type = db_msgs::Delete::Request::MAP;
		if (SQLITE_OK != sqlExec(state.c_str())) {
			ROS_ERROR("%s", err_msg_);
			del.request.name = temp_name;
			deleteData(del.request, del.response);
			return false;
		}

		//Soft 'delete' old one
		del.request.name = req.name;
		if (!deleteData(del.request, del.response)) {
			ROS_ERROR("Update map Fail in delete old map");
			del.request.name = temp_name;
			deleteData(del.request, del.response);
			return false;
		}

		//Get rid of _temp_
		db_msgs::Rename rename;
		rename.request.type     = db_msgs::Rename::Request::MAP;
		rename.request.old_name = temp_name;
		rename.request.new_name = req.name;
		if (!renameData(rename.request, rename.response)) {
			ROS_ERROR("Update map Fail in rename temp");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::clearOldMap(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		if (SQLITE_OK != sqlExec("DELETE FROM MapData WHERE map_id in (SELECT map_id FROM Map where old != 0);")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("DELETE FROM Map WHERE old != 0;")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}

	bool DatabaseHelper::getZoneInMap(db_msgs::GetZoneInMapRequest& req, db_msgs::GetZoneInMapResponse& resp) {
		std::string map_id = req.by_map_name ? "SELECT map_id FROM Map WHERE name = '" + req.name + "'" :
		                     std::to_string(req.map_id);
		std::string state  = "SELECT zone_id from Zone WHERE map_id = (" + map_id + ");";
		if (SQLITE_OK != sqlExec(state.c_str(), listCallBack)) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		resp.zones = zones_;
		zones_.clear();
		return true;
	}

	bool DatabaseHelper::getZone(db_msgs::GetZoneRequest& req, db_msgs::GetZoneResponse& resp) {
		std::string zone_id = req.by_zone_name ? "SELECT zone_id from Zone WHERE name = '" + req.zone_name + "'" :
		                      std::to_string(req.zone_id);

		std::string state = "SELECT map_id, path_id, type, config_id FROM Zone JOIN ZoneConfig "
		                    "ON Zone.zone_id = ZoneConfig.zone_id WHERE Zone.zone_id = (" + zone_id + ");";

		if (SQLITE_OK != sqlite3_prepare_v2(sql_db_, state.c_str(), state.length(), &stmt_, nullptr)) {
			ROS_ERROR("Error in get map_id prepare: %s", sqlite3_errmsg(sql_db_));
			return false;
		}

		sqlite3_step(stmt_);
		resp.map_id    = sqlite3_column_int(stmt_, 0);
		resp.path_id   = sqlite3_column_int(stmt_, 1);
		resp.type      = sqlite3_column_int(stmt_, 2);
		resp.config_id = sqlite3_column_int(stmt_, 3);
		return true;
	}

	bool DatabaseHelper::addZone(db_msgs::AddZoneRequest& req, db_msgs::AddZoneResponse&) {
		if (SQLITE_OK != sqlExec("BEGIN TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		std::string zone_id = "(SELECT zone_id FROM Zone WHERE name = '"+req.name+"')";
		std::string state = "INSERT INTO Zone (name,map_id, path_id, type) VALUES('" + req.name + "',"
		                    + std::to_string(req.map_id) + "," + std::to_string(req.path_id) + ","
		                    + std::to_string(req.type) + ");";
		if(SQLITE_OK != sqlExec(state.c_str())){
			ROS_ERROR("%s", err_msg_);
			return false;
		}

		state = "INSERT INTO ZoneConfig VALUES (" + zone_id+","+std::to_string(req.config_id)+");";

		if(SQLITE_OK != sqlExec(state.c_str())){
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		if (SQLITE_OK != sqlExec("END TRANSACTION")) {
			ROS_ERROR("%s", err_msg_);
			sqlExec("ROLLBACK");
			return false;
		}

		return true;
	}
}