#pragma once

#include <rclcpp/rclcpp.hpp>
#include "g1_loco_client_parameter.hpp"

namespace unitree
{
namespace ros2
{
namespace g1
{

class LocoClient: public rclcpp::Node
{
public:
    LocoClient();

    // Low Level API CALL
    int32_t GetFsmId(int32_t &fsm_id);

    int32_t GetFsmMode(int32_t& fsm_mode);

    int32_t GetBalanceMode(int32_t& balance_mode);

    int32_t GetSwingHeight(float& swing_height);

    int32_t GetStandHeight(float& stand_height);

    int32_t GetPhase(std::vector<float>& phase);

    int32_t SetFsmId(int32_t fsmId);

    int32_t SetBalanceMode(int32_t balanceMode);

    int32_t SetSwingHeight(float swingHeight);

    int32_t SetStandHeight(float standHeight);

    int32_t SetVelocity(float vx, float vy, float omega, float duration = 1.f);

    int32_t SetTaskId(int32_t taskId);

    // High Level API CALL
    int32_t Damp();

    int32_t Start();

    int32_t Squat();

    int32_t Sit();

    int32_t StandUp();

    int32_t ZeroTorque();

    int32_t StopMove();

    int32_t HighStand();

    int32_t LowStand();

    int32_t Move(float vx, float vy, float vyaw, bool continous_move);

    int32_t Move(float vx, float vy, float vyaw);

    int32_t SwitchMoveMode(bool flag);

    int32_t BalanceStand();

    int32_t ContinuousGait(bool flag);

    int32_t WaveHand(bool turn_flag = false);

    int32_t ShakeHand(int stage = -1);

private:
    bool wait_service();

    LocoClientParameter mLocoParam;
    rclcpp::Client<unitree_api::srv::Generic>::SharedPtr mClient;

    bool mContinousMove = false;
    bool mFirstShakeHandStage = false;
};

}
}
}
