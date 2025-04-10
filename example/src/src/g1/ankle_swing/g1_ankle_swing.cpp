/**
 * This example demonstrates how to use ROS2 to control ankle commands of unitree g1 robot
 **/
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "client/motion_switch_client.hpp" 
#include "utils/gamepad.hpp"
#include "motor_crc_hg.h"
#include "utils/utils.hpp"

using namespace unitree::common;
using std::placeholders::_1;

static const std::string HG_CMD_TOPIC = "lowcmd";
static const std::string HG_IMU_TORSO = "secondary_imu";
static const std::string HG_STATE_TOPIC = "lowstate";

template <typename T>
class DataBuffer
{
public:
    void SetData(const T &newData)
    {
        std::lock_guard<std::mutex> lock(mutex);
        data = std::make_shared<T>(newData);
    }

    std::shared_ptr<const T> GetData()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return data ? data : nullptr;
    }

    void Clear()
    {
        std::lock_guard<std::mutex> lock(mutex);
        data = nullptr;
    }

private:
    std::shared_ptr<T> data;
    std::mutex mutex;
};

const int G1_NUM_MOTOR = 29;
struct ImuState
{
    std::array<float, 3> rpy = {};
    std::array<float, 3> omega = {};
};
struct MotorCommand
{
    std::array<float, G1_NUM_MOTOR> q_target = {};
    std::array<float, G1_NUM_MOTOR> dq_target = {};
    std::array<float, G1_NUM_MOTOR> kp = {};
    std::array<float, G1_NUM_MOTOR> kd = {};
    std::array<float, G1_NUM_MOTOR> tau_ff = {};
};
struct MotorState
{
    std::array<float, G1_NUM_MOTOR> q = {};
    std::array<float, G1_NUM_MOTOR> dq = {};
};

// Stiffness for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,    // legs
    60, 60, 60, 100, 40, 40,    // legs
    60, 40, 40,                 // waist
    40, 40, 40, 40, 40, 40, 40, // arms
    40, 40, 40, 40, 40, 40, 40  // arms
};

// Damping for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,    // legs
    1, 1, 1, 2, 1, 1,    // legs
    1, 1, 1,             // waist
    1, 1, 1, 1, 1, 1, 1, // arms
    1, 1, 1, 1, 1, 1, 1  // arms
};

enum class Mode
{
    PR = 0, // Series Control for Ptich/Roll Joints
    AB = 1  // Parallel Control for A/B Joints
};

enum G1JointIndex
{
    LeftHipPitch = 0,
    LeftHipRoll = 1,
    LeftHipYaw = 2,
    LeftKnee = 3,
    LeftAnklePitch = 4,
    LeftAnkleB = 4,
    LeftAnkleRoll = 5,
    LeftAnkleA = 5,
    RightHipPitch = 6,
    RightHipRoll = 7,
    RightHipYaw = 8,
    RightKnee = 9,
    RightAnklePitch = 10,
    RightAnkleB = 10,
    RightAnkleRoll = 11,
    RightAnkleA = 11,
    WaistYaw = 12,
    WaistRoll = 13,  // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistA = 13,     // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14, // NOTE INVALID for g1 23dof/29dof with waist locked
    WaistB = 14,     // NOTE INVALID for g1 23dof/29dof with waist locked
    LeftShoulderPitch = 15,
    LeftShoulderRoll = 16,
    LeftShoulderYaw = 17,
    LeftElbow = 18,
    LeftWristRoll = 19,
    LeftWristPitch = 20, // NOTE INVALID for g1 23dof
    LeftWristYaw = 21,   // NOTE INVALID for g1 23dof
    RightShoulderPitch = 22,
    RightShoulderRoll = 23,
    RightShoulderYaw = 24,
    RightElbow = 25,
    RightWristRoll = 26,
    RightWristPitch = 27, // NOTE INVALID for g1 23dof
    RightWristYaw = 28    // NOTE INVALID for g1 23dof
};

class g1_ankle_swing_sender : public rclcpp::Node
{
public:
    g1_ankle_swing_sender() : Node("g1_ankle_swing_sender"), mTime(0.0), mControlDt(0.002),
                              mDuration(3.0), mCounter(0), mModePr(Mode::PR), mModeMachine(0)
    {
        //  bind to g1_ankle_swing_sender::LowStateHandler for subscribe "lowstate" topic
        mLowstateSubscriber = this->create_subscription<unitree_hg::msg::LowState>(
            HG_STATE_TOPIC, 10, std::bind(&g1_ankle_swing_sender::LowStateHandler, this, _1));
        //  bind to g1_ankle_swing_sender::ImuStateHandler for subscribe "secondary_imu" topic
        mImustateSubscriber = this->create_subscription<unitree_hg::msg::IMUState>(
            HG_IMU_TORSO, 10, std::bind(&g1_ankle_swing_sender::ImuStateHandler, this, _1));

        // the mLowcmdPublisher is set to publish "/lowcmd" topic
        mLowcmdPublisher = this->create_publisher<unitree_hg::msg::LowCmd>(HG_CMD_TOPIC, 10);

        mTimer1 = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&g1_ankle_swing_sender::Control, this));
        mTimer2 = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&g1_ankle_swing_sender::LowCommandWriter, this));
    }

private:
    void Control()
    {
        MotorCommand motorCommandTmp;
        const std::shared_ptr<const MotorState> ms = mMotorStateBuffer.GetData();

        for (int i = 0; i < G1_NUM_MOTOR; ++i)
        {
            motorCommandTmp.tau_ff.at(i) = 0.0;
            motorCommandTmp.q_target.at(i) = 0.0;
            motorCommandTmp.dq_target.at(i) = 0.0;
            motorCommandTmp.kp.at(i) = Kp[i];
            motorCommandTmp.kd.at(i) = Kd[i];
        }

        if (ms)
        {
            mTime += mControlDt;
            if (mTime < mDuration)
            {
                // [Stage 1]: set robot to zero posture
                for (int i = 0; i < G1_NUM_MOTOR; ++i)
                {
                    double ratio = unitree::common::clamp(mTime / mDuration, 0.0, 1.0);
                    motorCommandTmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
                }
            }
            else if (mTime < mDuration * 2)
            {
                // [Stage 2]: swing ankle using PR mode
                mModePr = Mode::PR;
                double max_P = M_PI * 30.0 / 180.0;
                double max_R = M_PI * 10.0 / 180.0;
                double t = mTime - mDuration;
                double L_P_des = max_P * std::sin(2.0 * M_PI * t);
                double L_R_des = max_R * std::sin(2.0 * M_PI * t);
                double R_P_des = max_P * std::sin(2.0 * M_PI * t);
                double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

                motorCommandTmp.q_target.at(LeftAnklePitch) = L_P_des;
                motorCommandTmp.q_target.at(LeftAnkleRoll) = L_R_des;
                motorCommandTmp.q_target.at(RightAnklePitch) = R_P_des;
                motorCommandTmp.q_target.at(RightAnkleRoll) = R_R_des;
            }
            else
            {
                // [Stage 3]: swing ankle using AB mode
                mModePr = Mode::AB;
                double max_A = M_PI * 30.0 / 180.0;
                double max_B = M_PI * 10.0 / 180.0;
                double t = mTime - mDuration * 2;
                double L_A_des = +max_A * std::sin(M_PI * t);
                double L_B_des = +max_B * std::sin(M_PI * t + M_PI);
                double R_A_des = -max_A * std::sin(M_PI * t);
                double R_B_des = -max_B * std::sin(M_PI * t + M_PI);

                motorCommandTmp.q_target.at(LeftAnkleA) = L_A_des;
                motorCommandTmp.q_target.at(LeftAnkleB) = L_B_des;
                motorCommandTmp.q_target.at(RightAnkleA) = R_A_des;
                motorCommandTmp.q_target.at(RightAnkleB) = R_B_des;
            }

            mMotorCommandBuffer.SetData(motorCommandTmp);
        }
    }

    void LowCommandWriter()
    {
        unitree_hg::msg::LowCmd lowCommand;
        lowCommand.mode_pr = static_cast<uint8_t>(mModePr);
        lowCommand.mode_machine = mModeMachine;

        const std::shared_ptr<const MotorCommand> mc = mMotorCommandBuffer.GetData();
        if (mc)
        {
            for (size_t i = 0; i < G1_NUM_MOTOR; i++)
            {
                lowCommand.motor_cmd.at(i).mode = 1; // 1:Enable, 0:Disable
                lowCommand.motor_cmd.at(i).tau = mc->tau_ff.at(i);
                lowCommand.motor_cmd.at(i).q = mc->q_target.at(i);
                lowCommand.motor_cmd.at(i).dq = mc->dq_target.at(i);
                lowCommand.motor_cmd.at(i).kp = mc->kp.at(i);
                lowCommand.motor_cmd.at(i).kd = mc->kd.at(i);
            }

            get_crc(lowCommand);
            mLowcmdPublisher->publish(lowCommand);
        }
    }

    void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message)
    {
        // get motor state
        MotorState msTmp;
        for (int i = 0; i < G1_NUM_MOTOR; ++i)
        {
            msTmp.q.at(i) = message->motor_state[i].q;
            msTmp.dq.at(i) = message->motor_state[i].dq;
            if (message->motor_state[i].motorstate && i <= RightAnkleRoll)
            {
                RCLCPP_INFO(this->get_logger(), "[ERROR] motor %d with code %d", i, message->motor_state[i].motorstate);
            }
        }
        mMotorStateBuffer.SetData(msTmp);

        // get imu state
        ImuState imuTmp;
        imuTmp.omega = message->imu_state.gyroscope;
        imuTmp.rpy = message->imu_state.rpy;
        mImuStateBuffer.SetData(imuTmp);

        // update gamepad
        Gamepad gamepad;
        REMOTE_DATA_RX rx;
        memcpy(rx.buff, &message->wireless_remote[0], 40);
        gamepad.update(rx.RF_RX);

        // update mode machine
        if (mModeMachine != message->mode_machine)
        {
            if (mModeMachine == 0)
            {
                RCLCPP_INFO(this->get_logger(), "G1 type: %d", unsigned(message->mode_machine));
            }
            mModeMachine = message->mode_machine;
        }

        // report robot status every second
        if (++mCounter % 500 == 0)
        {
            mCounter = 0;
            // IMU
            auto &rpy = message->imu_state.rpy;
            RCLCPP_INFO(this->get_logger(), "IMU.pelvis.rpy: %.2f %.2f %.2f\n", rpy[0], rpy[1], rpy[2]);

            // RC
            RCLCPP_INFO(this->get_logger(), "gamepad.A.pressed: %d\n", static_cast<int>(gamepad.A.pressed));
            RCLCPP_INFO(this->get_logger(), "gamepad.B.pressed: %d\n", static_cast<int>(gamepad.B.pressed));
            RCLCPP_INFO(this->get_logger(), "gamepad.X.pressed: %d\n", static_cast<int>(gamepad.X.pressed));
            RCLCPP_INFO(this->get_logger(), "gamepad.Y.pressed: %d\n", static_cast<int>(gamepad.Y.pressed));

            // Motor
            auto &ms = message->motor_state;
            RCLCPP_INFO(this->get_logger(), "All %d Motors:", G1_NUM_MOTOR);
            std::ostringstream oss;
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << static_cast<int32_t>(ms[i].mode) << " ";
            }
            RCLCPP_INFO(this->get_logger(), "mode: %s", oss.str().c_str());
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << std::fixed << std::setprecision(2) << ms[i].q << " ";
            }
            RCLCPP_INFO(this->get_logger(), "pos: %s", oss.str().c_str());
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << std::fixed << std::setprecision(2) << ms[i].dq << " ";
            }
            RCLCPP_INFO(this->get_logger(), "vel: %s", oss.str().c_str());
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << std::fixed << std::setprecision(2) << ms[i].tau_est << " ";
            }
            RCLCPP_INFO(this->get_logger(), "tau_est: %s", oss.str().c_str());
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << ms[i].temperature[0] << "," << ms[i].temperature[1] << " ";
            }
            RCLCPP_INFO(this->get_logger(), "temperature: %s", oss.str().c_str());
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << std::fixed << std::setprecision(2) << ms[i].vol << " ";
            }
            RCLCPP_INFO(this->get_logger(), "vol: %s", oss.str().c_str());
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << ms[i].sensor[0] << "," << ms[i].sensor[1] << " ";
            }
            RCLCPP_INFO(this->get_logger(), "sensor: %s", oss.str().c_str());
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << ms[i].motorstate << " ";
            }
            RCLCPP_INFO(this->get_logger(), "motorstate: %s", oss.str().c_str());
            oss.str("");
            for (int i = 0; i < G1_NUM_MOTOR; ++i)
            {
                oss << ms[i].reserve[0] << "," << ms[i].reserve[1] << "," << ms[i].reserve[2] << ms[i].reserve[3] << " ";
            }
            RCLCPP_INFO(this->get_logger(), "reserve: %s", oss.str().c_str());
        }
    }

    void ImuStateHandler(unitree_hg::msg::IMUState::SharedPtr message)
    {
        auto &rpy = message->rpy;
        if (mCounter % 500 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "IMU.torso.rpy: %.2f %.2f %.2f", rpy[0], rpy[1], rpy[2]);
        }
    }

    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr mLowcmdPublisher;         // ROS2 Publisher
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr mLowstateSubscriber; // ROS2 Subscriber
    rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr mImustateSubscriber; // ROS2 Subscriber
    rclcpp::TimerBase::SharedPtr mTimer1;
    rclcpp::TimerBase::SharedPtr mTimer2;

    double mTime;
    double mControlDt; // [2ms]
    double mDuration;  // [3 s]
    int32_t mCounter;
    Mode mModePr;
    std::atomic<uint8_t> mModeMachine;

    DataBuffer<MotorState> mMotorStateBuffer;
    DataBuffer<MotorCommand> mMotorCommandBuffer;
    DataBuffer<ImuState> mImuStateBuffer;
};

// try to shutdown motion control-related service
void ShutdownMotionCtrl()
{
    auto client = std::make_shared<unitree::ros2::MotionSwitchClient>();
    std::string form, name;
    while (client->CheckMode(form, name), !name.empty())
    {
        if (client->ReleaseMode() != 0)
        {
            std::cout << "Failed to switch to Release Mode" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    std::cout << "Shutdown motion control success" << std::endl;
}

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv); // Initialize rclcpp
        ShutdownMotionCtrl();

        auto node = std::make_shared<g1_ankle_swing_sender>(); // Create a ROS2 node and make share with g1_ankle_swing_sender class
        rclcpp::spin(node);                                    // Run ROS2 node
        rclcpp::shutdown();                                    // Exit
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        std::cerr << "RCLError caught: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}