#include <chrono>
#include <array>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "common/motor_crc_hg.h"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

using namespace std::chrono_literals;

static constexpr const char *TOPIC_LOWCMD = "rt/lowcmd";
static constexpr const char *TOPIC_LOWSTATE = "rt/lowstate";

static constexpr int H2_NUM_MOTOR = 31;

static const std::array<float, H2_NUM_MOTOR> Kp{
    200, 200, 200, 200, 200, 200,  // legs
    200, 200, 200, 200, 200, 200,  // legs
    300, 300, 300,                 // waist
    100, 100, 100, 100, 50, 50, 50,// arms
    100, 100, 100, 100, 50, 50, 50,// arms
    50, 10                         // head
};

static const std::array<float, H2_NUM_MOTOR> Kd{
    3, 3, 3, 3, 3, 3,        // legs
    3, 3, 3, 3, 3, 3,        // legs
    5, 5, 5,                 // waist
    2, 2, 2, 2, 2, 2, 2,     // arms
    2, 2, 2, 2, 2, 2, 2,     // arms
    2, 0.1                   // head
};

static const std::array<int, H2_NUM_MOTOR> joint_idx_in_idl{
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13, 14,
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28,
    29, 30
};

enum class Mode
{
    PR = 0,  // Series Control for Ptich/Roll Joints
    AB = 1   // Parallel Control for A/B Joints
};

enum H2JointIndex
{
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    LeftAnkleRoll = 4,
    LeftAnkleRollRaw = 4,
    LeftAnklePitch = 5,
    LeftAnklePitchRaw = 5,
    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,
    RightAnkleRoll = 10,
    RightAnkleRollRaw = 10,
    RightAnklePitch = 11,
    RightAnklePitchRaw = 11,
    WaistYaw = 12,
    WaistRoll = 13,       
    WaistA = 13,    
    WaistPitch = 14,   
    WaistB = 14,
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristpitch = 20,
    LeftWristyaw = 21,
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristpitch = 27,
    RightWristyaw = 28,
    HEAD_PITCH = 29,
    HEAD_YAW = 30,
};

static double clamp(double value, double low, double high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

class Custom : public rclcpp::Node
{
public:
    Custom() : Node("h2_ankle_swing_example")
    {
        Init();
        Start();
    }

private:
    void Init()
    {
        lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
            TOPIC_LOWSTATE, 10, [this](const unitree_hg::msg::LowState::SharedPtr msg)
            { LowStateMessageHandler(msg); });

        lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>(TOPIC_LOWCMD, 10);
    }

    void Start()
    {
        lowcmd_timer_ = this->create_wall_timer(2ms, [this]
                                               { LowCmdWrite(); });
    }

    void LowStateMessageHandler(const unitree_hg::msg::LowState::SharedPtr msg)
    {
        got_state_ = true;
        mode_machine_ = static_cast<int>(msg->mode_machine);
        for (int i = 0; i < H2_NUM_MOTOR; ++i)
        {
            motor_[i] = msg->motor_state.at(joint_idx_in_idl.at(i));
        }
    }

    void LowCmdWrite()
    {
        if (!got_state_)
        {
            return;
        }

        time_ += control_dt_;

    low_cmd_.mode_pr = static_cast<uint8_t>(mode_);
        low_cmd_.mode_machine = mode_machine_;

        // Initialize all motor slots, then fill only mapped joints.
        for (int slot = 0; slot < H2_NUM_MOTOR; ++slot)
        {
            low_cmd_.motor_cmd[slot].mode = 0;
            low_cmd_.motor_cmd[slot].tau = 0.0F;
            low_cmd_.motor_cmd[slot].q = 0.0F;
            low_cmd_.motor_cmd[slot].dq = 0.0F;
            low_cmd_.motor_cmd[slot].kp = 0.0F;
            low_cmd_.motor_cmd[slot].kd = 0.0F;
        }

        for (int i = 0; i < H2_NUM_MOTOR; ++i)
        {
            const int slot = joint_idx_in_idl.at(i);
            low_cmd_.motor_cmd[slot].mode = 1;  // 1:Enable, 0:Disable
            low_cmd_.motor_cmd[slot].tau = 0.0F;
            low_cmd_.motor_cmd[slot].q = 0.0F;
            low_cmd_.motor_cmd[slot].dq = 0.0F;
            low_cmd_.motor_cmd[slot].kp = Kp[i];
            low_cmd_.motor_cmd[slot].kd = Kd[i];
        }

        if (time_ < duration_)
        {
            // [Stage 1]: set robot to zero posture
            for (int i = 0; i < H2_NUM_MOTOR; ++i)
            {
                const double ratio = clamp(time_ / duration_, 0.0, 1.0);
                const int slot = joint_idx_in_idl.at(i);
                low_cmd_.motor_cmd[slot].q =
                    static_cast<float>((1.0 - ratio) * motor_[i].q);
            }
        }
        else if (time_ < duration_ * 2.0)
        {
            // [Stage 2]: swing ankle using PR mode
            mode_ = Mode::PR;
            const double max_P = M_PI * 30.0 / 180.0;
            const double max_R = M_PI * 30.0 / 180.0;
            const double t = time_ - duration_;
            const double L_P_des = max_P * std::sin(2.0 * M_PI * t);
            const double L_R_des = max_R * std::sin(2.0 * M_PI * t);
            const double R_P_des = max_P * std::sin(2.0 * M_PI * t);
            const double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

            // PR: use Pitch/Roll indices (compact indexing).
            low_cmd_.motor_cmd[joint_idx_in_idl.at(LeftAnklePitch)].q = static_cast<float>(L_P_des);
            low_cmd_.motor_cmd[joint_idx_in_idl.at(LeftAnkleRoll)].q = static_cast<float>(L_R_des);
            low_cmd_.motor_cmd[joint_idx_in_idl.at(RightAnklePitch)].q = static_cast<float>(R_P_des);
            low_cmd_.motor_cmd[joint_idx_in_idl.at(RightAnkleRoll)].q = static_cast<float>(R_R_des);
        }
        else
        {
            // [Stage 3]: swing ankle using AB mode
            mode_ = Mode::AB;
            const double max_A = M_PI * 30.0 / 180.0;
            const double max_B = M_PI * 10.0 / 180.0;
            const double t = time_ - duration_ * 2.0;
            const double L_A_des = +max_A * std::sin(M_PI * t);
            const double L_B_des = +max_B * std::sin(M_PI * t + M_PI);
            const double R_A_des = -max_A * std::sin(M_PI * t);
            const double R_B_des = -max_B * std::sin(M_PI * t + M_PI);

            low_cmd_.motor_cmd[joint_idx_in_idl.at(LeftAnklePitchRaw)].q = static_cast<float>(L_A_des);
            low_cmd_.motor_cmd[joint_idx_in_idl.at(LeftAnkleRollRaw)].q = static_cast<float>(L_B_des);
            low_cmd_.motor_cmd[joint_idx_in_idl.at(RightAnklePitchRaw)].q = static_cast<float>(R_A_des);
            low_cmd_.motor_cmd[joint_idx_in_idl.at(RightAnkleRollRaw)].q = static_cast<float>(R_B_des);
        }

        get_crc(low_cmd_);
        lowcmd_publisher_->publish(low_cmd_);
    }

    rclcpp::TimerBase::SharedPtr lowcmd_timer_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_;

    unitree_hg::msg::LowCmd low_cmd_;
    unitree_hg::msg::MotorState motor_[H2_NUM_MOTOR];

    const double control_dt_ = 0.002;  // 2ms
    double time_{0.0};
    const double duration_{3.0};
    Mode mode_{Mode::PR};
    int mode_machine_{0};
    bool got_state_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Custom>());
    rclcpp::shutdown();
    return 0;
}

