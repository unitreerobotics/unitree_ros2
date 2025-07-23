#ifndef _ROS2_SPORT_CLIENT_
#define _ROS2_SPORT_CLIENT_
#include <iostream>

#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"

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
 public:
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
};

#endif