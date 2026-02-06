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
const int32_t ROBOT_SPORT_API_ID_DAMP              = 1001;
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND      = 1002;
const int32_t ROBOT_SPORT_API_ID_STOPMOVE          = 1003;
const int32_t ROBOT_SPORT_API_ID_STANDUP           = 1004;
const int32_t ROBOT_SPORT_API_ID_STANDDOWN         = 1005;
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND     = 1006;
const int32_t ROBOT_SPORT_API_ID_MOVE              = 1008;
const int32_t ROBOT_SPORT_API_ID_SWITCHGAIT        = 1011;
const int32_t ROBOT_SPORT_API_ID_BODYHEIGHT        = 1013;
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL        = 1015;
const int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW  = 1018;
const int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT    = 1019;
const int32_t ROBOT_SPORT_API_ID_MOVETOPOS            = 1036;
const int32_t ROBOT_SPORT_API_ID_SWITCHMOVEMODE       = 1038;
const int32_t ROBOT_SPORT_API_ID_VISIONWALK        = 1101;
const int32_t ROBOT_SPORT_API_ID_HANDSTAND       = 1039;
const int32_t ROBOT_SPORT_API_ID_AUTORECOVERY_SET = 1040;
const int32_t ROBOT_SPORT_API_ID_FREEWALK  = 1045;
const int32_t ROBOT_SPORT_API_ID_CLASSICWALK = 1049;
const int32_t ROBOT_SPORT_API_ID_FASTWALK  = 1050;
const int32_t ROBOT_SPORT_API_ID_FREEEULER = 1051;

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
   * @brief Move
   * @api: 1008
   */
  void Move(unitree_api::msg::Request &req, float vx, float vy, float vyaw);

  /*
   * @brief SwitchGait
   * @api: 1011
   */
  void SwitchGait(unitree_api::msg::Request &req, int d);

  /*
   * @brief BodyHeight
   * @api: 1013
   */
  void BodyHeight(unitree_api::msg::Request &req, float height);

  /*
   * @brief SpeedLevel
   * @api: 1015
   */
  void SpeedLevel(unitree_api::msg::Request &req, int level);

  /*
   * @brief TrajectoryFollow
   * @api: 1018
   */
  void TrajectoryFollow(unitree_api::msg::Request &req, std::vector<PathPoint> &path);

  /*
   * @brief ContinuousGait
   * @api: 1019
   */
  void ContinuousGait(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief MoveToPos
   * @api: 1036
   */
  void MoveToPos(unitree_api::msg::Request &req, float x, float y, float yaw);

  /*
   * @brief SwitchMoveMode
   * @api: 1038
   */
  void SwitchMoveMode(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief Handstand
   * @api: 1039
   */
  void HandStand(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief VisionWalk
   * @api: 1101
   */
  void VisionWalk(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief AutoRecoverySet
   * @api: 1040
   */
  void AutoRecoverySet(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief FreeWalk
   * @api: 1045
   */
  void FreeWalk(unitree_api::msg::Request &req);

  /*
   * @brief ClassicWalk
   * @api: 1049
   */
  void ClassicWalk(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief FastWalk
   * @api: 1050
   */
  void FastWalk(unitree_api::msg::Request &req, bool flag);

  /*
   * @brief Euler
   * @api: 1051
   */
  void Euler(unitree_api::msg::Request &req, float roll, float pitch, float yaw);
};

#endif