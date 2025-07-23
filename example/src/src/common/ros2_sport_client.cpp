#include "ros2_sport_client.h"

void SportClient::Damp(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP;
}

void SportClient::BalanceStand(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
}

void SportClient::StopMove(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
}

void SportClient::StandUp(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP;
}

void SportClient::StandDown(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN;
}

void SportClient::RecoveryStand(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND;
}

void SportClient::Euler(unitree_api::msg::Request &req, float roll, float pitch,
                        float yaw) {
  nlohmann::json js;
  js["x"] = roll;
  js["y"] = pitch;
  js["z"] = yaw;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER;
}

void SportClient::Move(unitree_api::msg::Request &req, float vx, float vy,
                       float vyaw) {
  nlohmann::json js;
  js["x"] = vx;
  js["y"] = vy;
  js["z"] = vyaw;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
}

void SportClient::Sit(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SIT;
}

void SportClient::RiseSit(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_RISESIT;
}


void SportClient::SpeedLevel(unitree_api::msg::Request &req, int level) {
  nlohmann::json js;
  js["data"] = level;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
}

void SportClient::Hello(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_HELLO;
}

void SportClient::Stretch(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_STRETCH;
}


void SportClient::SwitchJoystick(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK;
}

void SportClient::Content(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTENT;
}

void SportClient::Pose(unitree_api::msg::Request &req, bool flag) {
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = ROBOT_SPORT_API_ID_POSE;
}

void SportClient::Scrape(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_SCRAPE;
}

void SportClient::FrontFlip(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTFLIP;
}

void SportClient::FrontJump(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTJUMP;
}

void SportClient::FrontPounce(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTPOUNCE;
}

void SportClient::Dance1(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE1;
}

void SportClient::Dance2(unitree_api::msg::Request &req) {
  req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE2;
}