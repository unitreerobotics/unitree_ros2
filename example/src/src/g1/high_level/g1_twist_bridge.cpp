/**
 * Twist-to-Velocity Bridge for G1 Robot
 * Subscribes to /cmd_vel (geometry_msgs/Twist) and converts to G1 velocity
 * commands
 */

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "g1/g1_loco_client.hpp"

class G1TwistBridge : public rclcpp::Node {
 public:
  G1TwistBridge()
      : Node("g1_twist_bridge"),
        loco_client_(this),
        last_cmd_time_(this->now()),
        vx_(0.0f),
        vy_(0.0f),
        omega_(0.0f) {
    // Declare and get velocity duration parameter
    this->declare_parameter<double>("velocity_duration", 0.2);
    velocity_duration_ = static_cast<float>(
        this->get_parameter("velocity_duration").as_double());

    this->declare_parameter<int>("velocity_update_period_ms", 100);
    auto requested_period = static_cast<int>(
        this->get_parameter("velocity_update_period_ms").as_int());
    velocity_period_ms_ = std::max(10, requested_period);

    // Subscribe to cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        std::bind(&G1TwistBridge::cmd_vel_callback, this,
                  std::placeholders::_1));

    // Timer to check for command timeout (stop if no commands for 1 second)
    timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&G1TwistBridge::timeout_timer_callback, this));

    // Use a reentrant callback group so SetVelocity waits don't block
    // responses.
    velocity_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Timer to send velocity commands at a steady cadence
    velocity_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(velocity_period_ms_),
        std::bind(&G1TwistBridge::velocity_timer_callback, this),
        velocity_callback_group_);

    RCLCPP_INFO(this->get_logger(),
                "G1 Twist Bridge started. Listening on /cmd_vel");
    RCLCPP_INFO(this->get_logger(), "Velocity duration: %.2fs",
                velocity_duration_);
    RCLCPP_INFO(this->get_logger(), "Velocity update period: %dms",
                velocity_period_ms_);
    RCLCPP_INFO(this->get_logger(),
                "Publish geometry_msgs/Twist to /cmd_vel to control robot");
  }

 private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Convert Twist to G1 velocity command
    float vx = static_cast<float>(msg->linear.x);      // forward/backward
    float vy = static_cast<float>(msg->linear.y);      // left/right
    float omega = static_cast<float>(msg->angular.z);  // rotation

    // Clamp velocities to safe ranges
    vx = std::clamp(vx, -0.5f, 0.5f);
    vy = std::clamp(vy, -0.3f, 0.3f);
    omega = std::clamp(omega, -1.0f, 1.0f);

    // Update velocity data (thread-safe with atomics)
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      vx_ = vx;
      vy_ = vy;
      omega_ = omega;
    }

    RCLCPP_INFO(this->get_logger(),
                "Velocity command received: vx=%.2f, vy=%.2f, omega=%.2f", vx,
                vy, omega);

    last_cmd_time_ = this->now();
  }

  void timeout_timer_callback() {
    // Check if no commands received for timeout duration
    auto time_since_last_cmd = (this->now() - last_cmd_time_).seconds();

    if (time_since_last_cmd > 1.0) {  // 1 second timeout
      // Set velocities to zero (worker thread will send the stop command)
      {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        vx_ = 0.0f;
        vy_ = 0.0f;
        omega_ = 0.0f;
      }

      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "No cmd_vel received for %.1f seconds. Stopping robot.",
          time_since_last_cmd);
    }
  }

  void velocity_timer_callback() {
    float vx;
    float vy;
    float omega;

    // Get current velocity (thread-safe)
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      vx = vx_;
      vy = vy_;
      omega = omega_;
    }

    auto t0 = std::chrono::steady_clock::now();

    RCLCPP_INFO(
        this->get_logger(),
        "Call SetVelocity start: vx=%.2f, vy=%.2f, omega=%.2f, duration=%.2f",
        vx, vy, omega, velocity_duration_);

    int32_t ret = loco_client_.SetVelocity(vx, vy, omega, velocity_duration_);

    RCLCPP_INFO(this->get_logger(), "Call SetVelocity end");

    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - t0)
                           .count();

    RCLCPP_INFO(this->get_logger(),
                "SetVelocity result: code=%d, duration time=%ldms", ret,
                duration_ms);

    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "SetVelocity FAILED with error code: %d",
                  ret);
    }
  }

  unitree::robot::g1::LocoClient loco_client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr velocity_timer_;
  rclcpp::CallbackGroup::SharedPtr velocity_callback_group_;
  rclcpp::Time last_cmd_time_;
  float velocity_duration_;
  int velocity_period_ms_;

  // Thread-safe velocity storage
  std::mutex velocity_mutex_;
  float vx_;
  float vy_;
  float omega_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<G1TwistBridge>();

  // Multi-threaded executor keeps timers responsive even if a SetVelocity call
  // blocks.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
