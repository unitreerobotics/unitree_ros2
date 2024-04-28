/**
 * This example demonstrates how to use ROS2 to receive motion states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#define INFO_FOOT_STATE 1 // Set 1 to info foot states (foot position and velocity in body frame)
#define HIGH_FREQ 0       // Set 1 to subscribe to motion states with high frequencies (500Hz)

using std::placeholders::_1;

class motion_state_suber : public rclcpp::Node
{
public:
  motion_state_suber() : Node("motion_state_suber")
  {
    // the cmd_puber is set to subscribe "sportmodestate" or  "lf/sportmodestate" (low frequencies) topic
    auto topic_name = "lf/sportmodestate";
    if (HIGH_FREQ)
    {
      topic_name = "sportmodestate";
    }

    // The suber  callback function is bind to motion_state_suber::topic_callback
    suber = this->create_subscription<unitree_go::msg::SportModeState>(
        topic_name, 10, std::bind(&motion_state_suber::topic_callback, this, _1));
  }

private:
  void topic_callback(unitree_go::msg::SportModeState::SharedPtr data)
  {
    // Info motion states
    // Gait type and foot raise height
    // Robot position (Odometry frame)
    // Robot velocity (Odometry frame)
    RCLCPP_INFO(this->get_logger(), "Gait state -- gait type: %d; raise height: %f", data->gait_type, data->foot_raise_height);
    RCLCPP_INFO(this->get_logger(), "Position -- x: %f; y: %f; z: %f; body height: %f",
                data->position[0], data->position[1], data->position[2], data->body_height);
    RCLCPP_INFO(this->get_logger(), "Velocity -- vx: %f; vy: %f; vz: %f; yaw: %f",
                data->velocity[0], data->velocity[1], data->velocity[2], data->yaw_speed);

    if (INFO_FOOT_STATE)
    {
      // Info foot states (foot position and velocity in body frame)
      for (int i = 0; i < 12; i++)
      {
        foot_pos[i] = data->foot_position_body[i];
        foot_vel[i] = data->foot_speed_body[i];
      }

      RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  0, foot_pos[0], foot_pos[1], foot_pos[2], foot_vel[0], foot_vel[1], foot_vel[2]);
      RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  1, foot_pos[3], foot_pos[4], foot_pos[5], foot_vel[3], foot_vel[4], foot_vel[5]);
      RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  2, foot_pos[6], foot_pos[7], foot_pos[8], foot_vel[6], foot_vel[7], foot_vel[8]);
      RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  3, foot_pos[9], foot_pos[10], foot_pos[11], foot_vel[9], foot_vel[10], foot_vel[11]);
    }
  }
  // Create the suber to receive motion states of robot
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber;
  float foot_pos[12];
  float foot_vel[12];
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv); // Initialize rclcpp
  rclcpp::spin(std::make_shared<motion_state_suber>()); // Run ROS2 node which is make share with motion_state_suber class
  rclcpp::shutdown();
  return 0;
}