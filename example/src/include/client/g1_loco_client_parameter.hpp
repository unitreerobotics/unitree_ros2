#pragma once

#include "unitree_api/srv/generic.hpp"

constexpr int32_t ROBOT_API_ID_LOCO_GET_FSM_ID = 7001;
constexpr int32_t ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002;
constexpr int32_t ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 7003;
constexpr int32_t ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 7004;
constexpr int32_t ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 7005;
constexpr int32_t ROBOT_API_ID_LOCO_GET_PHASE = 7006; // deprecated

constexpr int32_t ROBOT_API_ID_LOCO_SET_FSM_ID = 7101;
constexpr int32_t ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 7102;
constexpr int32_t ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 7103;
constexpr int32_t ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 7104;
constexpr int32_t ROBOT_API_ID_LOCO_SET_VELOCITY = 7105;
constexpr int32_t ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106;

namespace unitree
{
namespace ros2
{
namespace g1
{

class LocoClientParameter
{
public:
    void GetFsmIdReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetFsmIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int &fsm_id);

    void GetFsmModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetFsmModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int &fsm_mode);

    void GetBalanceModeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetBalanceModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, int32_t &balance_mode);

    void GetSwingHeightReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetSwingHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, float &swing_height);

    void GetStandHeightReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetStandHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, float &stand_height);

    void GetPhaseReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetPhaseRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, std::vector<float> &phase);

    void SetFsmIdReq(int32_t fsmId, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SetFsmIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SetBalanceModeReq(int32_t balanceMode, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SetBalanceModeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SetSwingHeightReq(float swingHeight, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SetSwingHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SetStandHeightReq(float standHeight, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SetStandHeightRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SetVelocityReq(float vx, float vy, float omega, float duration, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SetVelocityRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void SetTaskIdReq(int32_t taskId, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SetTaskIdRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);
};

}
}
}
