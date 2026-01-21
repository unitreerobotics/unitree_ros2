/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "ros2_sport_client.h"

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

void SportClient::Euler(unitree_api::msg::Request &req, float roll, float pitch,
                        float yaw) {
  nlohmann::json js;
  js["x"] = roll;
  js["y"] = pitch;
  js["z"] = yaw;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER;
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

void SportClient::Sit(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SIT;
  req_puber_->publish(req);
}

void SportClient::RiseSit(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_RISESIT;
  req_puber_->publish(req);
}

void SportClient::SpeedLevel(unitree_api::msg::Request &req, int level) {
  nlohmann::json js;
  js["data"] = level;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
  req_puber_->publish(req);
}

void SportClient::Hello(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_HELLO;
  req_puber_->publish(req);
}

void SportClient::Stretch(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STRETCH;
  req_puber_->publish(req);
}

void SportClient::SwitchJoystick(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK;
  req_puber_->publish(req);
}

void SportClient::Content(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTENT;
  req_puber_->publish(req);
}

void SportClient::Pose(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_POSE;
  req_puber_->publish(req);
}

void SportClient::Scrape(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SCRAPE;
  req_puber_->publish(req);
}

void SportClient::FrontFlip(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTFLIP;
  req_puber_->publish(req);
}

void SportClient::FrontJump(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTJUMP;
  req_puber_->publish(req);
}

void SportClient::FrontPounce(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTPOUNCE;
  req_puber_->publish(req);
}

void SportClient::Dance1(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE1;
  req_puber_->publish(req);
}

void SportClient::Dance2(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE2;
  req_puber_->publish(req);
}

void SportClient::Heart(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_HEART;
  req_puber_->publish(req);
}

void SportClient::StaticWalk(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STATICWALK;
  req_puber_->publish(req);
}

void SportClient::TrotRun(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_TROTRUN;
  req_puber_->publish(req);
}

void SportClient::EconomicGait(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_ECONOMICGAIT;
  req_puber_->publish(req);
}

void SportClient::LeftFlip(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_LEFTFLIP;
  req_puber_->publish(req);
}

void SportClient::BackFlip(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_BACKFLIP;
  req_puber_->publish(req);
}

void SportClient::HandStand(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_HANDSTAND;
  req_puber_->publish(req);
}

void SportClient::FreeWalk(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FREEWALK;
  req_puber_->publish(req);
}

void SportClient::FreeBound(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FREEBOUND;
  req_puber_->publish(req);
}

void SportClient::FreeJump(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FREEJUMP;
  req_puber_->publish(req);
}

void SportClient::FreeAvoid(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FREEAVOID;
  req_puber_->publish(req);
}

void SportClient::ClassicWalk(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_CLASSICWALK;
  req_puber_->publish(req);
}

void SportClient::WalkUpright(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_WALKUPRIGHT;
  req_puber_->publish(req);
}

void SportClient::CrossStep(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_CROSSSTEP;
  req_puber_->publish(req);
}

void SportClient::AutoRecoverySet(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_AUTORECOVERY_SET;
  req_puber_->publish(req);
}

void SportClient::AutoRecoveryGet(unitree_api::msg::Request &req, bool &flag) {

  req.header.identity.api_id = ROBOT_SPORT_API_ID_AUTORECOVERY_GET;
  auto js = Call<unitree_api::msg::Request, unitree_api::msg::Response>(req);
  flag = js["data"].get<bool>();
}

void SportClient::SwitchAvoidMode(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHAVOIDMODE;
  req_puber_->publish(req);
}
