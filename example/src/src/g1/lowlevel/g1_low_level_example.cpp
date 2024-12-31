/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of unitree g1 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include "common/motor_crc_hg.h"

#define INFO_IMU 0        // Set 1 to info IMU states
#define INFO_MOTOR 0      // Set 1 to info motor states
#define HIGH_FREQ 1 // Set 1 to subscribe to low states with high frequencies (500Hz)

enum PRorAB { PR = 0, AB = 1 };

using std::placeholders::_1;

const int G1_NUM_MOTOR = 29;

enum G1JointIndex {
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
  WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,  // NOTE INVALID for g1 23dof
  RightWristYaw = 28     // NOTE INVALID for g1 23dof
};


// Create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node
{
public:
    low_level_cmd_sender() : Node("low_level_cmd_sender")
    {
        auto topic_name = "lf/lowstate";
        if (HIGH_FREQ)
        {
            topic_name = "lowstate";
        }

        // The suber  callback function is bind to low_level_cmd_sender::topic_callback
        lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
            topic_name, 10, std::bind(&low_level_cmd_sender::LowStateHandler, this, _1));

        // the lowcmd_publisher_ is set to subscribe "/lowcmd" topic
        lowcmd_publisher_ = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);

        // The timer is set to 500 Hz, and bind to low_level_cmd_sender::Control function
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_dt), std::bind(&low_level_cmd_sender::Control, this));
        
        // Running time count
        time_ = 0;

        duration_ = 3; // 3 s
    }

private:
    void Control()
    {
        // Test code here
        time_ += control_dt_;
        low_command.mode_pr = mode_;
        low_command.mode_machine = mode_machine;
        for (int i = 0; i < G1_NUM_MOTOR; ++i) 
        {
            low_command.motor_cmd[i].mode = 1;  // 1:Enable, 0:Disable
            low_command.motor_cmd[i].tau = 0.0;
            low_command.motor_cmd[i].q= 0.0;
            low_command.motor_cmd[i].dq = 0.0;
            low_command.motor_cmd[i].kp = (i < 13) ? 100.0 : 50.0;
            low_command.motor_cmd[i].kd = 1.0;
        }

        if (time_ < duration_) 
        {
            // [Stage 1]: set robot to zero posture
            for (int i = 0; i < G1_NUM_MOTOR; ++i) {
                double ratio = clamp(time_ / duration_, 0.0, 1.0);
                low_command.motor_cmd[i].q =
                    (1. - ratio) * motor[i].q;
            }
        }
        else 
        {
            // [Stage 2]: swing ankle's PR
            mode_ = PRorAB::PR;  // Enable PR mode
            // generate sin/cos trajectory
            double max_P = 0.25;  // [rad]
            double max_R = 0.25;  // [rad]
            double t = time_ - duration_;
            double L_P_des = max_P * std::cos(2.0 * M_PI * t);
            double L_R_des = max_R * std::sin(2.0 * M_PI * t);
            double R_P_des = max_P * std::cos(2.0 * M_PI * t);
            double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

            // update ankle joint position targets
            float Kp_Pitch = 80;
            float Kd_Pitch = 1;
            float Kp_Roll = 80;
            float Kd_Roll = 1;

            low_command.motor_cmd[G1JointIndex::LeftAnklePitch].q = L_P_des; 
            low_command.motor_cmd[G1JointIndex::LeftAnklePitch].dq = 0;
            low_command.motor_cmd[G1JointIndex::LeftAnklePitch].kp = Kp_Pitch;
            low_command.motor_cmd[G1JointIndex::LeftAnklePitch].kd = Kd_Pitch;
            low_command.motor_cmd[G1JointIndex::LeftAnklePitch].tau = 0;
            low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].q = L_R_des;  
            low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].dq = 0;
            low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].kp = Kp_Roll;
            low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].kd = Kd_Roll;
            low_command.motor_cmd[G1JointIndex::LeftAnkleRoll].tau = 0;
            low_command.motor_cmd[G1JointIndex::RightAnklePitch].q = R_P_des;  
            low_command.motor_cmd[G1JointIndex::RightAnklePitch].dq = 0;
            low_command.motor_cmd[G1JointIndex::RightAnklePitch].kp = Kp_Pitch;
            low_command.motor_cmd[G1JointIndex::RightAnklePitch].kd = Kd_Pitch;
            low_command.motor_cmd[G1JointIndex::RightAnklePitch].tau = 0;
            low_command.motor_cmd[G1JointIndex::RightAnkleRoll].q = R_R_des;  
            low_command.motor_cmd[G1JointIndex::RightAnkleRoll].dq = 0;
            low_command.motor_cmd[G1JointIndex::RightAnkleRoll].kp = Kp_Roll;
            low_command.motor_cmd[G1JointIndex::RightAnkleRoll].kd = Kd_Roll;
            low_command.motor_cmd[G1JointIndex::RightAnkleRoll].tau = 0;

            double max_wrist_roll_angle = 0.5;  // [rad]
            double WristRoll_des = max_wrist_roll_angle * std::sin(2.0 * M_PI * t);
            low_command.motor_cmd[G1JointIndex::LeftWristRoll].q = WristRoll_des;  
            low_command.motor_cmd[G1JointIndex::LeftWristRoll].dq = 0;
            low_command.motor_cmd[G1JointIndex::LeftWristRoll].kp = 50;
            low_command.motor_cmd[G1JointIndex::LeftWristRoll].kd = 1;
            low_command.motor_cmd[G1JointIndex::LeftWristRoll].tau = 0;

            low_command.motor_cmd[G1JointIndex::RightWristRoll].q = WristRoll_des; 
            low_command.motor_cmd[G1JointIndex::RightWristRoll].dq = 0;
            low_command.motor_cmd[G1JointIndex::RightWristRoll].kp = 50;
            low_command.motor_cmd[G1JointIndex::RightWristRoll].kd = 1;
            low_command.motor_cmd[G1JointIndex::RightWristRoll].tau = 0;

        }
        get_crc(low_command); 
        lowcmd_publisher_->publish(low_command); // Publish lowcmd message
    }

    void LowStateHandler(unitree_hg::msg::LowState::SharedPtr message)
    {
        mode_machine = (int)message->mode_machine;
        imu = message->imu_state;
        for (int i = 0; i < G1_NUM_MOTOR; i++)
        {
            motor[i] = message->motor_state[i];
        }

        if (INFO_IMU)
        {
            // Info IMU states
            // RPY euler angle(ZYX order respected to body frame)
            // Quaternion
            // Gyroscope (raw data)
            // Accelerometer (raw data)
            RCLCPP_INFO(this->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0], imu.rpy[1], imu.rpy[2]);
            RCLCPP_INFO(this->get_logger(), "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                        imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]);
            RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f", imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);
            RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                        imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2]);
        }
        if (INFO_MOTOR)
        {
            // Info motor states
            // q: angluar (rad)
            // dq: angluar velocity (rad/s)
            // ddq: angluar acceleration (rad/(s^2))
            // tau_est: Estimated external torque
            for (int i = 0; i < G1_NUM_MOTOR; i++)
            {
                motor[i] = message->motor_state[i];
                RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                            i, motor[i].q, motor[i].dq, motor[i].ddq, motor[i].tau_est);
            }
        }
    }

    double clamp(double value, double low, double high) 
    {
        if (value < low) return low;
        if (value > high) return high;
        return value;
    }

    rclcpp::TimerBase::SharedPtr timer_;                                             // ROS2 timer
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;         // ROS2 Publisher
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscriber_; // ROS2 Subscriber
    unitree_hg::msg::LowCmd low_command;                                             // Unitree hg lowcmd message
    unitree_hg::msg::IMUState imu;                                                   // Unitree hg IMU message
    unitree_hg::msg::MotorState motor[G1_NUM_MOTOR];                               // Unitree hg motor state message
    double control_dt_ = 0.002;                                                      // 2ms
    int timer_dt = control_dt_*1000;
    double time_;                                                                    // Running time count
    double duration_;
    PRorAB mode_ = PRorAB::PR;
    int mode_machine;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                             // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_;                  // Create a timer callback object to send cmd in time intervals
    auto node = std::make_shared<low_level_cmd_sender>(); // Create a ROS2 node and make share with low_level_cmd_sender class
    rclcpp::spin(node);                                   // Run ROS2 node
    rclcpp::shutdown();                                   // Exit
    return 0;
}
