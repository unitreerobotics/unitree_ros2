
#include <unistd.h>

#include <cmath>

#include "common/ros2_sport_client.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

// Create a soprt_request class for soprt commond request
class SoprtRequest : public rclcpp::Node {
 public:
  SoprtRequest() : Node("req_sender") {
    // the state_suber is set to subscribe "sportmodestate" topic
    state_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "sportmodestate", 10,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          state_callback(data);
        });
    // the req_puber is set to subscribe "/api/sport/request" topic with dt
    req_puber_ = this->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
        [this] { timer_callback(); });

    // Runing time count
  };

 private:
  void timer_callback() {
    t += dt_;
    if (t > 0) {
      double const time_seg = 0.2;
      double time_temp = t - time_seg;

      std::vector<PathPoint> path;

      for (int i = 0; i < 30; i++) {
        PathPoint path_point_tmp;
        time_temp += time_seg;
        // Tacking a sin path in x direction
        // The path is respect to the initial coordinate system
        float const px_local = 0.5 * sin(0.5 * time_temp);
        float const py_local = 0;
        float const yaw_local = 0.;
        float const vx_local = 0.5 * cos(0.5 * time_temp);
        float const vy_local = 0;
        float const vyaw_local = 0.;

        // Convert trajectory commands to the initial coordinate system
        path_point_tmp.timeFromStart = i * time_seg;
        path_point_tmp.x = px_local * cos(yaw0_) - py_local * sin(yaw0_) + px0_;
        path_point_tmp.y = px_local * sin(yaw0_) + py_local * cos(yaw0_) + py0_;
        path_point_tmp.yaw = yaw_local + yaw0_;
        path_point_tmp.vx = vx_local * cos(yaw0_) - vy_local * sin(yaw0_);
        path_point_tmp.vy = vx_local * sin(yaw0_) + vy_local * cos(yaw0_);
        path_point_tmp.vyaw = vyaw_local;
        path.push_back(path_point_tmp);
      }
      // Get request messages corresponding to high-level motion commands
      sport_req.TrajectoryFollow(req_, path);
      // Publish request messages
      req_puber_->publish(req_);
    }
  };

  void state_callback(const unitree_go::msg::SportModeState::SharedPtr& data) {
    // Get current position of robot when t<0
    // This position is used as the initial coordinate system

    if (t < 0) {
      // Get initial position
      px0_ = data->position[0];
      py0_ = data->position[1];
      yaw0_ = data->imu_state.rpy[2];
      std::cout << px0_ << ", " << py0_ << ", " << yaw0_ << std::endl;
    }
  }

  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber_;

  rclcpp::TimerBase::SharedPtr timer_;  // ROS2 timer
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;

  unitree_api::msg::Request req_;  // Unitree Go2 ROS2 request message
  SportClient sport_req;

  double t{-1};        // runing time count
  double dt_ = 0.002;  // control time step

  double px0_ = 0;   // initial x position
  double py0_ = 0;   // initial y position
  double yaw0_ = 0;  // initial yaw angle
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::TimerBase::SharedPtr const
      timer_;  // Create a timer callback object to
               // send sport request in time intervals

  rclcpp::spin(std::make_shared<SoprtRequest>());  // Run ROS2 node

  rclcpp::shutdown();
  return 0;
}
