/**
 * This example demonstrates how to use ROS2 to receive low states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/motor_state.hpp"

#define INFO_IMU 1        // Set 1 to info IMU states
#define INFO_MOTOR 1      // Set 1 to info motor states

#define HIGH_FREQ 1 // Set 1 to subscribe to low states with high frequencies (500Hz)

using std::placeholders::_1;

class low_state_suber : public rclcpp::Node
{
public:
  low_state_suber() : Node("low_state_suber")
  {
    // suber is set to subscribe "/lowcmd" or  "lf/lowstate" (low frequencies) topic
    auto topic_name = "lf/lowstate";
    if (HIGH_FREQ)
    {
      topic_name = "lowstate";
    }
    // The suber  callback function is bind to low_state_suber::topic_callback
    suber = this->create_subscription<unitree_hg::msg::LowState>(
        topic_name, 10, std::bind(&low_state_suber::topic_callback, this, _1));
  }

private:
  void topic_callback(unitree_hg::msg::LowState::SharedPtr data)
  {

    if (INFO_IMU)
    {
      // Info IMU states
      // RPY euler angle(ZYX order respected to body frame)
      // Quaternion
      // Gyroscope (raw data)
      // Accelerometer (raw data)
      imu = data->imu_state;

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

      for (int i = 0; i < 12; i++)
      {
        motor[i] = data->motor_state[i];
        RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                    i, motor[i].q, motor[i].dq, motor[i].ddq, motor[i].tau_est);
      }
    }
  }

  // Create the suber  to receive low state of robot
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr suber;

  unitree_hg::msg::IMUState imu;         // Unitree hg IMU message
  unitree_hg::msg::MotorState motor[35]; // Unitree hg motor state message
  float battery_voltage;                 // Battery voltage
  float battery_current;                 // Battery current
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                          // Initialize rclcpp
  rclcpp::spin(std::make_shared<low_state_suber>()); // Run ROS2 node which is make share with low_state_suber class
  rclcpp::shutdown();
  return 0;
}