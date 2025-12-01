/**
 * This example demonstrates how to use ROS2 to receive motion states of unitree
 *go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

constexpr bool INFO_FOOT_STATE = true;
// Set 1 to info foot states (foot position and velocity in body frame)
constexpr bool HIGH_FREQ = false;
// Set 1 to subscribe to low states with high frequencies (500Hz)

class MotionStateSuber : public rclcpp::Node {
 public:
  MotionStateSuber() : Node("motion_state_suber") {
    // the cmd_puber is set to subscribe "sportmodestate" or "lf/sportmodestate"
    // (low frequencies) topic
    const auto *topic_name = "lf/sportmodestate";
    if (HIGH_FREQ) {
      topic_name = "sportmodestate";
    }

    // The suber  callback function is bind to
    // motion_state_suber::topic_callback
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        topic_name, 10,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          topic_callback(data);
        });
  }

 private:
  void topic_callback(const unitree_go::msg::SportModeState::SharedPtr &data) {
    // Info motion states
    // Gait type and foot raise height
    // Robot position (Odometry frame)
    // Robot velocity (Odometry frame)
    RCLCPP_INFO(this->get_logger(),
                "Gait state -- gait type: %d; raise height: %f",
                data->gait_type, data->foot_raise_height);
    RCLCPP_INFO(this->get_logger(),
                "Position -- x: %f; y: %f; z: %f; body height: %f",
                data->position[0], data->position[1], data->position[2],
                data->body_height);
    RCLCPP_INFO(this->get_logger(),
                "Velocity -- vx: %f; vy: %f; vz: %f; yaw: %f",
                data->velocity[0], data->velocity[1], data->velocity[2],
                data->yaw_speed);

    if (INFO_FOOT_STATE) {
      // Info foot states (foot position and velocity in body frame)
      for (int i = 0; i < 12; i++) {
        foot_pos_[i] = data->foot_position_body[i];
        foot_vel_[i] = data->foot_speed_body[i];
      }

      RCLCPP_INFO(this->get_logger(),
                  "Foot position and velcity relative to body -- num: %d; x: "
                  "%f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  0, foot_pos_[0], foot_pos_[1], foot_pos_[2], foot_vel_[0],
                  foot_vel_[1], foot_vel_[2]);
      RCLCPP_INFO(this->get_logger(),
                  "Foot position and velcity relative to body -- num: %d; x: "
                  "%f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  1, foot_pos_[3], foot_pos_[4], foot_pos_[5], foot_vel_[3],
                  foot_vel_[4], foot_vel_[5]);
      RCLCPP_INFO(this->get_logger(),
                  "Foot position and velcity relative to body -- num: %d; x: "
                  "%f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  2, foot_pos_[6], foot_pos_[7], foot_pos_[8], foot_vel_[6],
                  foot_vel_[7], foot_vel_[8]);
      RCLCPP_INFO(this->get_logger(),
                  "Foot position and velcity relative to body -- num: %d; x: "
                  "%f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                  3, foot_pos_[9], foot_pos_[10], foot_pos_[11], foot_vel_[9],
                  foot_vel_[10], foot_vel_[11]);
    }
  }
  // Create the suber to receive motion states of robot
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  float foot_pos_[12]{};
  float foot_vel_[12]{};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::spin(
      std::make_shared<MotionStateSuber>());  // Run ROS2 node which is make
                                              // share with motion_state_suber
                                              // class
  rclcpp::shutdown();
  return 0;
}