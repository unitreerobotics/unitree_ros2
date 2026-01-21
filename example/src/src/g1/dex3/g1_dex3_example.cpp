#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"
using namespace std::chrono_literals;

enum State { INIT, ROTATE, GRIP, STOP, PRINT };

// set URDF Limits
const std::array<float, 7> max_limits_left = {1.05, 1.05, 1.75, 0, 0, 0, 0};
const std::array<float, 7> min_limits_left = {-1.05, -0.724, 0,    -1.57,
                                              -1.75, -1.57,  -1.75};
const std::array<float, 7> max_limits_right = {1.05, 0.742, 0,   1.57,
                                               1.75, 1.57,  1.75};
const std::array<float, 7> min_limits_right = {-1.05, -1.05, -1.75, 0, 0, 0, 0};

#define MOTOR_MAX 7
#define SENSOR_MAX 9

struct RisMode {
  uint8_t id : 4;
  uint8_t status : 3;
  uint8_t timeout : 1;
};

class Dex3HandNode : public rclcpp::Node {  // NOLINT
 public:
  Dex3HandNode(std::string hand_side, std::string network_interface)
      : Node("dex3_hand_node"),
        hand_side_(std::move(hand_side)),
        network_interface_(std::move(network_interface)) {
    // Set up DDS topics based on hand side
    if (hand_side_ == "L") {
      dds_namespace_ = "/dex3/left/cmd";
      sub_namespace_ = "/lf/dex3/left/state";
    } else {
      dds_namespace_ = "/dex3/right/cmd";
      sub_namespace_ = "/lf/dex3/right/state";
    }
    // Initialize publishers and subscribers
    handcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::HandCmd>(dds_namespace_, 10);
    handstate_subscriber_ =
        this->create_subscription<unitree_hg::msg::HandState>(
            sub_namespace_, rclcpp::QoS(1),
            [this](const std::shared_ptr<const unitree_hg::msg::HandState>
                       message) { stateHandler(message); });

    state_.motor_state.resize(MOTOR_MAX);
    state_.press_sensor_state.resize(SENSOR_MAX);
    msg_.motor_cmd.resize(MOTOR_MAX);

    // Print initialization message
    RCLCPP_INFO(this->get_logger(), "Dex3 %s Hand Node initialized",
                hand_side_.c_str());
    PrintHelp();

    // Create timer for state machine
    thread_control_ = std::thread([this] { Loop(); });
    thread_input_ = std::thread([this] {
      while (true) {
        HandleUserInput();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }

  ~Dex3HandNode() override { StopMotors(); }

 private:
  void Loop() {
    while (true) {
      usleep(100);
      State state = {};
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = current_state_.load();
      }

      if (state != last_state_) {
        std::cout << "\n--- Current State: " << stateToString(state)
                  << " ---\n";
        std::cout << "Commands:\n";
        std::cout << "  r - Rotate\n";
        std::cout << "  g - Grip\n";
        std::cout << "  p - Print_state\n";
        std::cout << "  q - Quit\n";
        std::cout << "  s - Stop\n";
        last_state_ = state;
      }
      switch (current_state_) {
        case INIT:
          RCLCPP_INFO_ONCE(this->get_logger(), "Initializing...");
          current_state_ = ROTATE;
          break;
        case ROTATE:
          RotateMotors();
          break;
        case GRIP:
          GripHand();
          break;
        case STOP:
          StopMotors();
          break;
        case PRINT:
          PrintState();
          break;
      }
    }
  }

  void HandleUserInput() {
    char ch = getNonBlockingInput();
    if (ch != 0) {
      switch (ch) {
        case 'q':
          RCLCPP_INFO(this->get_logger(), "Exiting...");
          rclcpp::shutdown();
          break;
        case 'r':
          current_state_ = ROTATE;
          break;
        case 'g':
          current_state_ = GRIP;
          break;
        case 'p':
          current_state_ = PRINT;
          break;
        case 's':
          current_state_ = STOP;
          break;
        case 'h':
          PrintHelp();
          break;
      }
    }
  }

  static char getNonBlockingInput() {
    struct termios oldt {};
    struct termios newt {};
    char ch = 0;
    int oldf = 0;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  // NOLINT

    ch = getchar();  // NOLINT

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);  // NOLINT
    return ch;
  }

  void stateHandler(
      const std::shared_ptr<const unitree_hg::msg::HandState> message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = *message;
  }

  void RotateMotors() {
    static int count = 1;
    static int dir = 1;

    const std::array<float, 7>& max_limits =
        (hand_side_ == "L") ? max_limits_left : max_limits_right;
    const std::array<float, 7>& min_limits =
        (hand_side_ == "L") ? min_limits_left : min_limits_right;

    for (int i = 0; i < MOTOR_MAX; i++) {
      RisMode ris_mode{};
      ris_mode.id = i;
      ris_mode.status = 0x01;
      ris_mode.timeout = 0x00;
      uint8_t mode = 0;
      mode |= (ris_mode.id & 0x0F);
      mode |= (ris_mode.status & 0x07) << 4;
      mode |= (ris_mode.timeout & 0x01) << 7;

      msg_.motor_cmd[i].mode = (mode);
      msg_.motor_cmd[i].tau = 0;
      msg_.motor_cmd[i].kp = (0.5);
      msg_.motor_cmd[i].kd = (0.1);

      float range = max_limits[i] - min_limits[i];
      float mid = (max_limits[i] + min_limits[i]) / 2.0F;
      float amplitude = range / 2.0F;
      auto q = static_cast<float>(
          mid + amplitude * sin(static_cast<float>(count) / 20000.0F * M_PI));

      msg_.motor_cmd[i].q = q;
    }

    handcmd_publisher_->publish(msg_);
    count += dir;

    if (count >= 10000) {
      dir = -1;
    }
    if (count <= -10000) {
      dir = 1;
    }
  }

  void GripHand() {
    const std::array<float, 7>& max_limits =
        (hand_side_ == "L") ? max_limits_left : max_limits_right;
    const std::array<float, 7>& min_limits =
        (hand_side_ == "L") ? min_limits_left : min_limits_right;

    for (int i = 0; i < MOTOR_MAX; i++) {
      RisMode ris_mode{};
      ris_mode.id = i;
      ris_mode.status = 0x01;
      ris_mode.timeout = 0x00;
      uint8_t mode = 0;
      mode |= (ris_mode.id & 0x0F);
      mode |= (ris_mode.status & 0x07) << 4;
      mode |= (ris_mode.timeout & 0x01) << 7;

      msg_.motor_cmd[i].mode = (mode);
      msg_.motor_cmd[i].tau = 0;
      msg_.motor_cmd[i].kp = 1.5;
      msg_.motor_cmd[i].kd = 0.1;

      float mid = (max_limits[i] + min_limits[i]) / 2.0F;
      msg_.motor_cmd[i].q = mid;
      msg_.motor_cmd[i].dq = 0;
    }

    handcmd_publisher_->publish(msg_);
    usleep(1000000);
  }
  void StopMotors() {
    for (int i = 0; i < MOTOR_MAX; i++) {
      RisMode ris_mode{};
      ris_mode.id = i;
      ris_mode.status = 0x01;
      ris_mode.timeout = 0x01;

      uint8_t mode = 0;
      mode |= (ris_mode.id & 0x0F);
      mode |= (ris_mode.status & 0x07) << 4;
      mode |= (ris_mode.timeout & 0x01) << 7;

      msg_.motor_cmd[i].mode = (mode);
      msg_.motor_cmd[i].tau = 0;
      msg_.motor_cmd[i].dq = 0;
      msg_.motor_cmd[i].kp = 0;
      msg_.motor_cmd[i].kd = 0;
      msg_.motor_cmd[i].q = 0;
    }
    handcmd_publisher_->publish(msg_);
  }

  void PrintState() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    Eigen::Matrix<float, 7, 1> q;
    const std::array<float, 7>& max_limits =
        (hand_side_ == "L") ? max_limits_left : max_limits_right;
    const std::array<float, 7>& min_limits =
        (hand_side_ == "L") ? min_limits_left : min_limits_right;

    for (int i = 0; i < 7; i++) {
      q(i) = state_.motor_state[i].q;
      q(i) = (q(i) - min_limits[i]) / (max_limits[i] - min_limits[i]);
      q(i) = std::clamp(q(i), 0.0F, 1.0F);
    }

    std::cout << "\033[2J\033[H";  // Clear screen
    std::cout << "-- " << hand_side_ << " Hand State --\n";
    std::cout << "Current State: " << stateToString(current_state_) << "\n";
    std::cout << "Position: " << q.transpose() << "\n";
    std::cout << "PressSensorState(example[0][0]): "
              << state_.press_sensor_state[0].pressure[0] << std::endl;
    PrintHelp();
  }

  static void PrintHelp() {
    std::cout << "Commands:\n";
    std::cout << "  r - Rotate\n";
    std::cout << "  g - Grip\n";
    std::cout << "  p - Print state\n";
    std::cout << "  s - Stop\n";
    std::cout << "  h - Help\n";
    std::cout << "  q - Quit\n";
  }

  static const char* stateToString(State state) {
    switch (state) {
      case INIT:
        return "INIT";
      case ROTATE:
        return "ROTATE";
      case GRIP:
        return "GRIP";
      case STOP:
        return "STOP";
      case PRINT:
        return "PRINT";
      default:
        return "UNKNOWN";
    }
  }

  // Member variables
  std::string hand_side_;
  std::string network_interface_;
  std::string dds_namespace_;
  std::string sub_namespace_;

  rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr handcmd_publisher_;
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr
      handstate_subscriber_;

  unitree_hg::msg::HandCmd msg_;
  unitree_hg::msg::HandState state_;

  std::atomic<State> current_state_{INIT};
  std::atomic<State> last_state_{INIT};
  std::mutex state_mutex_;
  std::thread thread_control_;
  std::thread thread_input_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <L/R> <network_interface>"
              << std::endl;
    return 1;
  }

  std::string hand_side = argv[1];
  if (hand_side != "L" && hand_side != "R") {
    std::cerr << "Invalid hand side. Please specify 'L' or 'R'." << std::endl;
    return 1;
  }

  std::string network_interface = argv[2];

  auto node = std::make_shared<Dex3HandNode>(hand_side, network_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}