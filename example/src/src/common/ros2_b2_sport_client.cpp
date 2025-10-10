/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "ros2_b2_sport_client.h"

void SportClient::Damp(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP;
  req_puber_->publish(req);
}

void SportClient::BalanceStand(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
  req_puber_->publish(req);
}

void SportClient::StopMove(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
  req_puber_->publish(req);
}

void SportClient::StandUp(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP;
  req_puber_->publish(req);
}

void SportClient::StandDown(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN;
  req_puber_->publish(req);
}

void SportClient::RecoveryStand(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND;
  req_puber_->publish(req);
}


void SportClient::Move(unitree_api::msg::Request &req, float vx, float vy,
                       float vyaw) {
  nlohmann::json js;
  js["x"] = vx;
  js["y"] = vy;
  js["z"] = vyaw;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
  req_puber_->publish(req);
}

void SportClient::SwitchGait(unitree_api::msg::Request &req, int d) {
  nlohmann::json js;
  js["data"] = d;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHGAIT;
  req_puber_->publish(req);

}
void SportClient::BodyHeight(unitree_api::msg::Request &req, float height) {
  nlohmann::json js;
  js["data"] = height;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_BODYHEIGHT;
  req_puber_->publish(req);
}

void SportClient::SpeedLevel(unitree_api::msg::Request &req, int level) {
  nlohmann::json js;
  js["data"] = level;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
  req_puber_->publish(req);
}

void SportClient::TrajectoryFollow(unitree_api::msg::Request &req, std::vector<PathPoint> &path) {
  nlohmann::json js_path;
  for (int i = 0; i < 30; i++) {
        nlohmann::json js_point;
        js_point["t_from_start"] = path[i].timeFromStart;
        js_point["x"] = path[i].x;
        js_point["y"] = path[i].y;
        js_point["yaw"] = path[i].yaw;
        js_point["vx"] = path[i].vx;
        js_point["vy"] = path[i].vy;
        js_point["vyaw"] = path[i].vyaw;
        js_path.push_back(js_point);
    }
  req.parameter = js_path.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW;
  req_puber_->publish(req);
}

void SportClient::ContinuousGait(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTINUOUSGAIT ;
  req_puber_->publish(req);
}

void SportClient::MoveToPos(unitree_api::msg::Request &req, float x, float y, float yaw) {
  nlohmann::json js;
  js["x"] = x;
  js["y"] = y;
  js["yaw"] = yaw;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVETOPOS;
  req_puber_->publish(req);
}

void SportClient::SwitchMoveMode(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHMOVEMODE;
  req_puber_->publish(req);
}

void SportClient::HandStand(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_HANDSTAND;
  req_puber_->publish(req);
}

void SportClient::VisionWalk(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_VISIONWALK;
  req_puber_->publish(req);
}

void SportClient::AutoRecoverySet(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_AUTORECOVERY_SET;
  req_puber_->publish(req);
}

void SportClient::FreeWalk(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FREEWALK;
  req_puber_->publish(req);
}

void SportClient::ClassicWalk(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_CLASSICWALK;
  req_puber_->publish(req);
}

void SportClient::FastWalk(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FASTWALK;
  req_puber_->publish(req);
}

 void SportClient::Euler(unitree_api::msg::Request &req, float roll, float pitch, float yaw) {
  nlohmann::json js;
  js["x"] = roll;
  js["y"] = pitch;
  js["z"] = yaw;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FREEEULER;
  req_puber_->publish(req);
}
