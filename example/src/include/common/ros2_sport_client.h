/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef _ROS2_SPORT_CLIENT_
#define _ROS2_SPORT_CLIENT_
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
const int32_t ROBOT_SPORT_API_ID_DAMP = 1001;
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;
const int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
const int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;
const int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;
const int32_t ROBOT_SPORT_API_ID_EULER = 1007;
const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;
const int32_t ROBOT_SPORT_API_ID_SIT = 1009;
const int32_t ROBOT_SPORT_API_ID_RISESIT = 1010;
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;
const int32_t ROBOT_SPORT_API_ID_HELLO = 1016;
const int32_t ROBOT_SPORT_API_ID_STRETCH = 1017;
const int32_t ROBOT_SPORT_API_ID_CONTENT = 1020;
const int32_t ROBOT_SPORT_API_ID_DANCE1 = 1022;
const int32_t ROBOT_SPORT_API_ID_DANCE2 = 1023;
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027;
const int32_t ROBOT_SPORT_API_ID_POSE = 1028;
const int32_t ROBOT_SPORT_API_ID_SCRAPE = 1029;
const int32_t ROBOT_SPORT_API_ID_FRONTFLIP = 1030;
const int32_t ROBOT_SPORT_API_ID_FRONTJUMP = 1031;
const int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE = 1032;
const int32_t ROBOT_SPORT_API_ID_HEART = 1036;
const int32_t ROBOT_SPORT_API_ID_STATICWALK = 1061;
const int32_t ROBOT_SPORT_API_ID_TROTRUN = 1062;
const int32_t ROBOT_SPORT_API_ID_ECONOMICGAIT = 1063;
const int32_t ROBOT_SPORT_API_ID_LEFTFLIP = 2041;
const int32_t ROBOT_SPORT_API_ID_BACKFLIP = 2043;
const int32_t ROBOT_SPORT_API_ID_HANDSTAND = 2044;
const int32_t ROBOT_SPORT_API_ID_FREEWALK = 2045;
const int32_t ROBOT_SPORT_API_ID_FREEBOUND = 2046;
const int32_t ROBOT_SPORT_API_ID_FREEJUMP = 2047;
const int32_t ROBOT_SPORT_API_ID_FREEAVOID = 2048;
const int32_t ROBOT_SPORT_API_ID_CLASSICWALK = 2049;
const int32_t ROBOT_SPORT_API_ID_WALKUPRIGHT = 2050;
const int32_t ROBOT_SPORT_API_ID_CROSSSTEP = 2051;
const int32_t ROBOT_SPORT_API_ID_AUTORECOVERY_SET = 2054;
const int32_t ROBOT_SPORT_API_ID_AUTORECOVERY_GET = 2055;
const int32_t ROBOT_SPORT_API_ID_SWITCHAVOIDMODE = 2058;

#pragma pack(1)
struct PathPoint {
  float timeFromStart;
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vyaw;
};
#pragma pack()

class SportClient {
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr req_suber_;
  rclcpp::Node *node_;

 public:
  explicit SportClient(rclcpp::Node *node) : node_(node) {
    req_puber_ = node_->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);
  }

  template <typename Request, typename Response>
  nlohmann::json Call(const Request &req) {
    std::promise<typename Response::SharedPtr> response_promise;
    auto response_future = response_promise.get_future();
    auto api_id = req.header.identity.api_id;
    auto req_suber_ = node_->create_subscription<Response>(
        "/api/sport/response", 1,
        [&response_promise, api_id](const typename Response::SharedPtr data) {
          if (data->header.identity.api_id == api_id) {
            response_promise.set_value(data);
          }
        });

    req_puber_->publish(req);

    auto response = *response_future.get();
    nlohmann::json js = nlohmann::json::parse(response.data.data());
    req_suber_.reset();
    return js;
  }

  /*
   * @brief Damp
   * @api: 1001
   */
  void Damp(unitree_api::msg::Request &req);

  /*
   * @brief BalanceStand
   * @api: 1002
   */
  void BalanceStand(unitree_api::msg::Request &req);

  /*
   * @brief StopMove
   * @api: 1003
   */
  void StopMove(unitree_api::msg::Request &req);

  /*
   * @brief StandUp
   * @api: 1004
   */
  void StandUp(unitree_api::msg::Request &req);

  /*
   * @brief StandDown
   * @api: 1005
   */
  void StandDown(unitree_api::msg::Request &req);

  /*
   * @brief RecoveryStand
   * @api: 1006
   */
  void RecoveryStand(unitree_api::msg::Request &req);

  /*
   * @brief Euler
   * @api: 1007
   */
  void Euler(unitree_api::msg::Request &req, float roll, float pitch,
             float yaw);

  /*
   * @brief Move
   * @api: 1008
   */
  void Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw);

  /*
   * @brief Sit
   * @api: 1009
   */
  void Sit(unitree_api::msg::Request &req);

  /*
   * @brief RiseSit
   * @api: 1010
   */
  void RiseSit(unitree_api::msg::Request &req);

  /*
   * @brief SpeedLevel
   * @api: 1015
   */
  void SpeedLevel(unitree_api::msg::Request &req, int level);

  /*
   * @brief Hello
   * @api: 1016
   */
  void Hello(unitree_api::msg::Request &req);

  /*
   * @brief Stretch
   * @api: 1017
   */
  void Stretch(unitree_api::msg::Request &req);

  /*
   * @brief SwitchJoystick
   * @api: 1027
   */
  void SwitchJoystick(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief Content
   * @api: 1020
   */
  void Content(unitree_api::msg::Request &req);

  /*
   * @brief Pose
   * @api: 1028
   */
  void Pose(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief Scrape
   * @api: 1029
   */
  void Scrape(unitree_api::msg::Request &req);

  /*
   * @brief FrontFlip
   * @api: 1030
   */
  void FrontFlip(unitree_api::msg::Request &req);

  /*
   * @brief FrontJump
   * @api: 1031
   */
  void FrontJump(unitree_api::msg::Request &req);

  /*
   * @brief FrontPounce
   * @api: 1032
   */
  void FrontPounce(unitree_api::msg::Request &req);

  /*
   * @brief Dance1
   * @api: 1022
   */
  void Dance1(unitree_api::msg::Request &req);

  /*
   * @brief Dance2
   * @api: 1023
   */
  void Dance2(unitree_api::msg::Request &req);

  /*
   * @brief Heart
   * @api: 1036
   */
  void Heart(unitree_api::msg::Request &req);

  /*
   * @brief StaticWalk
   * @api: 1061
   */
  void StaticWalk(unitree_api::msg::Request &req);

  /*
   * @brief TrotRun
   * @api: 1062
   */
  void TrotRun(unitree_api::msg::Request &req);

  /*
   * @brief EconomicGait
   * @api: 1063
   */
  void EconomicGait(unitree_api::msg::Request &req);

  /*
   * @brief LeftFlip
   * @api: 2041
   */
  void LeftFlip(unitree_api::msg::Request &req);

  /*
   * @brief BackFlip
   * @api: 2043
   */
  void BackFlip(unitree_api::msg::Request &req);

  /*
   * @brief Handstand
   * @api: 2044
   */
  void HandStand(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief FreeWalk
   * @api: 2045
   */
  void FreeWalk(unitree_api::msg::Request &req);

  /*
   * @brief FreeBound
   * @api: 2046
   */
  void FreeBound(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief FreeJump
   * @api: 2047
   */
  void FreeJump(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief FreeAvoid
   * @api: 2048
   */
  void FreeAvoid(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief ClassicWalk
   * @api: 2049
   */
  void ClassicWalk(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief WalkUpright
   * @api: 2050
   */
  void WalkUpright(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief CrossStep
   * @api: 2051
   */
  void CrossStep(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief AutoRecoverySet
   * @api: 2054
   * @param flag: true to enable, false to disable
   */
  void AutoRecoverySet(unitree_api::msg::Request &req, bool flag);
  /*
   * @brief AutoRecoveryGet
   * @api: 2055
   * NOTICE!!!: This function cannot be used in ros2 callback.
   *
   */
  void AutoRecoveryGet(unitree_api::msg::Request &req, bool &flag);
  /*
   * @brief SwitchAvoidMode
   * @api: 2058
   * @param flag: true to enable, false to disable
   */
  void SwitchAvoidMode(unitree_api::msg::Request &req);
};

#endif