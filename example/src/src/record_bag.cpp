/**
 * This example demonstrates how to use ROS2 to record motion states of unitree
 *go2 robot
 **/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#if __has_include("rosbag2_storage/storage_options.hpp")
#include "rosbag2_storage/storage_options.hpp"
#endif
#include "unitree_go/msg/sport_mode_state.hpp"

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

#if __has_include("rosbag2_storage/storage_options.hpp")
    using StorageOptions = rosbag2_storage::StorageOptions;
#else
    using StorageOptions = rosbag2_cpp::StorageOptions;
#endif
    const StorageOptions storage_options(
        {declare_parameter<std::string>("output", "motion_state_bag"), "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
        {rmw_get_serialization_format(), rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    // The suber  callback function is bind to
    // motion_state_suber::topic_callback
    suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        topic_name, rclcpp::SensorDataQoS(),
        [this](unitree_go::msg::SportModeState::SharedPtr data) {
          topic_callback(data);
        });
    topic_name_ = suber_->get_topic_name();
    rosbag2_storage::TopicMetadata metadata;
    metadata.name = topic_name_;
    metadata.type = "unitree_go/msg/SportModeState";
    metadata.serialization_format = rmw_get_serialization_format();
    writer_->create_topic(metadata);
  }

 private:
  void topic_callback(unitree_go::msg::SportModeState::SharedPtr data) {
    // Info motion states
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

    // Record motion states and serialize
    auto serializer = rclcpp::Serialization<unitree_go::msg::SportModeState>();
    auto serialized_message = rclcpp::SerializedMessage();
    serializer.serialize_message(data.get(), &serialized_message);

    auto bag_message =
        std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        new rcutils_uint8_array_t, [this](rcutils_uint8_array_t *msg) {
          auto fini_return = rcutils_uint8_array_fini(msg);
          delete msg;
          if (fini_return != RCUTILS_RET_OK) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to destroy serialized message %s",
                         rcutils_get_error_string().str);
          }
        });
    *bag_message->serialized_data =
        serialized_message.release_rcl_serialized_message();

    // Create a topic for message bag recoding
    bag_message->topic_name = topic_name_;
#ifdef UNITREE_ROSBAG2_SEPARATE_TIMESTAMPS
    bag_message->recv_timestamp = now().nanoseconds();
    bag_message->send_timestamp = bag_message->recv_timestamp;
#else
    bag_message->time_stamp = now().nanoseconds();
#endif

    writer_->write(bag_message);
  }

  std::string topic_name_;
  // Create the suber to receive motion states of robot
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
  // Create the writer_ to record motion states of robot
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
  unitree_go::msg::SportModeState
      state_data_;  // Unitree sportmode state message
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
