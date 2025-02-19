/**
 * This example demonstrates how to use ROS2 to control arm commands of unitree g1 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"

using std::placeholders::_1;

constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;

enum JointIndex
{
    // Left leg
    kLeftHipPitch,
    kLeftHipRoll,
    kLeftHipYaw,
    kLeftKnee,
    kLeftAnkle,
    kLeftAnkleRoll,

    // Right leg
    kRightHipPitch,
    kRightHipRoll,
    kRightHipYaw,
    kRightKnee,
    kRightAnkle,
    kRightAnkleRoll,

    kWaistYaw,
    kWaistRoll,
    kWaistPitch,

    // Left arm
    kLeftShoulderPitch,
    kLeftShoulderRoll,
    kLeftShoulderYaw,
    kLeftElbow,
    kLeftWistRoll,
    kLeftWistPitch,
    kLeftWistYaw,
    // Right arm
    kRightShoulderPitch,
    kRightShoulderRoll,
    kRightShoulderYaw,
    kRightElbow,
    kRightWistRoll,
    kRightWistPitch,
    kRightWistYaw,

    kNotUsedJoint,
    kNotUsedJoint1,
    kNotUsedJoint2,
    kNotUsedJoint3,
    kNotUsedJoint4,
    kNotUsedJoint5
};

struct LowCmdParam
{
    float period = 5.f;
    float init_time = 2.0f;
    float control_dt = 0.02f;
    float weight = 0.f;
    float weight_rate = 0.2f;
    float kp = 60.f;
    float kd = 1.5f;
    float dq = 0.f;
    float tau_ff = 0.f;
    float max_joint_velocity = 0.5f;
    float delta_weight = weight_rate * control_dt;
    float max_joint_delta = max_joint_velocity * control_dt;
    int32_t sleep_time = static_cast<int>(control_dt / 0.001f);
    int32_t num_time_steps = static_cast<int>(period / control_dt);
    std::array<float, 17> init_pos = {0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0};
    std::array<float, 17> target_pos = {0.f, kPi_2, 0.f, kPi_2, 0.f, 0.f, 0.f,
                                        0.f, -kPi_2, 0.f, kPi_2, 0.f, 0.f, 0.f,
                                        0, 0, 0};
    std::array<JointIndex, 17> arm_joints = {
        JointIndex::kLeftShoulderPitch, JointIndex::kLeftShoulderRoll,
        JointIndex::kLeftShoulderYaw, JointIndex::kLeftElbow,
        JointIndex::kLeftWistRoll, JointIndex::kLeftWistPitch,
        JointIndex::kLeftWistYaw,
        JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
        JointIndex::kRightShoulderYaw, JointIndex::kRightElbow,
        JointIndex::kRightWistRoll, JointIndex::kRightWistPitch,
        JointIndex::kRightWistYaw,
        JointIndex::kWaistYaw,
        JointIndex::kWaistRoll,
        JointIndex::kWaistPitch};
    std::array<float, 17> current_jpos_des{};
};

// Create a g1_arm7_control_sender class for low state receive
class g1_arm7_control_sender : public rclcpp::Node
{
public:
    g1_arm7_control_sender() : Node("g1_arm7_control_sender")
    {
        auto topic_name = "lowstate";

        //  bind to g1_arm7_control_sender::LowStateHandler for subscribe "lowstate" topic
        mLowstateSubscriber = this->create_subscription<unitree_hg::msg::LowState>(
            topic_name, 10, std::bind(&g1_arm7_control_sender::LowStateHandler, this, _1));

        // the mLowcmdPublisher is set to subscribe "/arm_sdk" topic
        mLowcmdPublisher = this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", 10);
    }

private:
    void Control()
    {
        InitArms();
        LiftArmsUp();
        PutArmsDown();
        StopControl();
        mTimer->cancel();
    }

    void InitArms()
    {
        // get current joint position
        std::array<float, 17> current_jpos{};
        std::ostringstream oss;
        for (size_t i = 0; i < mLowCmdParam.arm_joints.size(); ++i)
        {
            current_jpos.at(i) = mStateMsg->motor_state.at(mLowCmdParam.arm_joints.at(i)).q;
            oss << current_jpos.at(i) << " ";
        }
        RCLCPP_INFO(this->get_logger(), "Current joint position: %s", oss.str().c_str());

        // set init position
        RCLCPP_INFO(this->get_logger(), "Initailizing arms ...");

        int init_time_steps = static_cast<int>(mLowCmdParam.init_time / mLowCmdParam.control_dt);
        for (int i = 0; i < init_time_steps; ++i)
        {
            // increase weight
            mLowCmdParam.weight = 1.0;
            mCmd.motor_cmd.at(JointIndex::kNotUsedJoint).set__q(mLowCmdParam.weight);
            float phase = 1.0 * i / init_time_steps;
            RCLCPP_INFO(this->get_logger(), "Phase: %f", phase);

            // set control joints
            for (size_t j = 0; j < mLowCmdParam.init_pos.size(); ++j)
            {
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__q(mLowCmdParam.init_pos.at(j) * phase + current_jpos.at(j) * (1 - phase));
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__dq(mLowCmdParam.dq);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__kp(mLowCmdParam.kp);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__kd(mLowCmdParam.kd);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__tau(mLowCmdParam.tau_ff);
            }
            // Publish lowcmd message
            mLowcmdPublisher->publish(mCmd);

            // sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(mLowCmdParam.sleep_time));
        }
        RCLCPP_INFO(this->get_logger(), "Init Arms Done!");
    }

    void LiftArmsUp()
    {
        RCLCPP_INFO(this->get_logger(), "Start lift arms up!");
        for (int i = 0; i < mLowCmdParam.num_time_steps; ++i)
        {
            // update jpos des
            for (size_t j = 0; j < mLowCmdParam.init_pos.size(); ++j)
            {
                mLowCmdParam.current_jpos_des.at(j) +=
                    clamp(mLowCmdParam.target_pos.at(j) - mLowCmdParam.current_jpos_des.at(j),
                          -mLowCmdParam.max_joint_delta, mLowCmdParam.max_joint_delta);
            }

            // set control joints
            for (size_t j = 0; j < mLowCmdParam.init_pos.size(); ++j)
            {
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__q(mLowCmdParam.current_jpos_des.at(j));
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__dq(mLowCmdParam.dq);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__kp(mLowCmdParam.kp);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__kd(mLowCmdParam.kd);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__tau(mLowCmdParam.tau_ff);
            }

            // send ros msg
            mLowcmdPublisher->publish(mCmd);

            // sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(mLowCmdParam.sleep_time));
        }
    }

    void PutArmsDown()
    {
        RCLCPP_INFO(this->get_logger(), "Start lift arms down!");
        for (int32_t i = 0; i < mLowCmdParam.num_time_steps; ++i)
        {
            // update jpos des
            for (size_t j = 0; j < mLowCmdParam.init_pos.size(); ++j)
            {
                mLowCmdParam.current_jpos_des.at(j) +=
                    clamp(mLowCmdParam.init_pos.at(j) - mLowCmdParam.current_jpos_des.at(j), -mLowCmdParam.max_joint_delta,
                          mLowCmdParam.max_joint_delta);
            }

            // set control joints
            for (size_t j = 0; j < mLowCmdParam.init_pos.size(); ++j)
            {
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__q(mLowCmdParam.current_jpos_des.at(j));
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__dq(mLowCmdParam.dq);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__kp(mLowCmdParam.kp);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__kd(mLowCmdParam.kd);
                mCmd.motor_cmd.at(mLowCmdParam.arm_joints.at(j)).set__tau(mLowCmdParam.tau_ff);
            }

            // send ros msg
            mLowcmdPublisher->publish(mCmd);

            // sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(mLowCmdParam.sleep_time));
        }
    }

    void StopControl()
    {
        RCLCPP_INFO(this->get_logger(), "Stoping arm ctrl ...");
        float stop_time = 2.0f;
        int stop_time_steps = static_cast<int>(stop_time / mLowCmdParam.control_dt);

        for (int i = 0; i < stop_time_steps; ++i)
        {
            // increase weight
            mLowCmdParam.weight -= mLowCmdParam.delta_weight;
            mLowCmdParam.weight = clamp(mLowCmdParam.weight, 0.f, 1.f);

            // set weight
            mCmd.motor_cmd.at(JointIndex::kNotUsedJoint).set__q(mLowCmdParam.weight);

            // send ros msg
            mLowcmdPublisher->publish(mCmd);

            // sleep
            std::this_thread::sleep_for(std::chrono::milliseconds(mLowCmdParam.sleep_time));
        }
    }

    void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message)
    {
        if (mStateMsg == nullptr)
        {
            std::lock_guard<std::mutex> mtx(mMtx);
            if (mStateMsg == nullptr)
            {
                mStateMsg = std::make_shared<unitree_hg::msg::LowState>();

                mStateMsg->set__version(message->version);
                mStateMsg->set__mode_pr(message->mode_pr);
                mStateMsg->set__mode_machine(message->mode_machine);
                mStateMsg->set__tick(message->tick);
                mStateMsg->set__imu_state(message->imu_state);
                mStateMsg->set__motor_state(message->motor_state);
                mStateMsg->set__wireless_remote(message->wireless_remote);
                mStateMsg->set__reserve(message->reserve);
                mStateMsg->set__crc(message->crc);

                mTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&g1_arm7_control_sender::Control, this));
            }
        }
    }

    double clamp(float value, float low, float high)
    {
        if (value < low)
            return low;
        if (value > high)
            return high;
        return value;
    }

    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr mLowcmdPublisher;         // ROS2 Publisher
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr mLowstateSubscriber; // ROS2 Subscriber
    rclcpp::TimerBase::SharedPtr mTimer;

    std::mutex mMtx;
    unitree_hg::msg::LowState::SharedPtr mStateMsg;
    unitree_hg::msg::LowCmd mCmd;
    LowCmdParam mLowCmdParam;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                               // Initialize rclcpp
    auto node = std::make_shared<g1_arm7_control_sender>(); // Create a ROS2 node and make share with g1_arm7_control_sender class
    rclcpp::spin(node);                                     // Run ROS2 node
    rclcpp::shutdown();                                     // Exit
    return 0;
}