#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/time_tools.hpp"
#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
namespace unitree::ros2::g1 {

constexpr int32_t ROBOT_API_ID_AUDIO_TTS = 1001;
constexpr int32_t ROBOT_API_ID_AUDIO_ASR = 1002;
constexpr int32_t ROBOT_API_ID_AUDIO_START_PLAY = 1003;
constexpr int32_t ROBOT_API_ID_AUDIO_STOP_PLAY = 1004;
constexpr int32_t ROBOT_API_ID_AUDIO_GET_VOLUME = 1005;
constexpr int32_t ROBOT_API_ID_AUDIO_SET_VOLUME = 1006;
constexpr int32_t ROBOT_API_ID_AUDIO_SET_RGB_LED = 1010;

class AudioClient : public rclcpp::Node {
  uint32_t tts_index_ = 0;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;

 public:
  AudioClient()
      : rclcpp::Node("audio_client"),
        req_puber_(this->create_publisher<unitree_api::msg::Request>(
            "/api/voice/request", rclcpp::QoS(10))) {};
  template <typename Request, typename Response>
  int32_t Call(Request req, nlohmann::json &js, int timeout_seconds = 2) {
    std::promise<typename Response::SharedPtr> response_promise;
    auto response_future = response_promise.get_future();
    const auto api_id = req.header.identity.api_id;
    req.header.identity.id = unitree::common::GetSystemUptimeInNanoseconds();
    auto req_suber_ = this->create_subscription<Response>(
        "/api/voice/response", rclcpp::QoS(10),
        [&response_promise, api_id](const typename Response::SharedPtr data) {
          if (data->header.identity.api_id == api_id) {
            response_promise.set_value(data);
          }
        });

    req_puber_->publish(req);
    auto status =
        response_future.wait_for(std::chrono::seconds(timeout_seconds));

    unitree_api::msg::Response response;
    if (status == std::future_status::ready) {
      response = *response_future.get();
      if (response.header.status.code != 0) {
        std::cout << "error code: " << response.header.status.code << std::endl;
        return response.header.status.code;
      }
      try {
        js = nlohmann::json::parse(response.data.data());
      } catch (nlohmann::detail::exception &e) {
        std::cout << "parse error" << std::endl;
      }
      return 0;
      std::cout << "task finish." << std::endl;
    }
    if (status == std::future_status::timeout) {
      std::cout << "task timeout." << std::endl;
      return -1;
    }
    std::cout << "task error." << std::endl;
    return -2;
  }

  int32_t TtsMaker(const std::string &text, int32_t speaker_id) {
    nlohmann::json js;
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_TTS;
    js["index"] = tts_index_++;
    js["text"] = text;
    js["speaker_id"] = speaker_id;
    req.parameter = js.dump();
    auto res = Call<unitree_api::msg::Request, unitree_api::msg::Response>(
        req, js, 10);
    return res;
  }

  int32_t GetVolume(uint8_t &volume) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_GET_VOLUME;
    nlohmann::json js;
    auto res =
        Call<unitree_api::msg::Request, unitree_api::msg::Response>(req, js);
    if (res == 0) {
      js["volume"].get_to(volume);
    }
    return res;
  }

  int32_t SetVolume(uint8_t volume) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_SET_VOLUME;
    nlohmann::json js;
    js["volume"] = volume;
    req.parameter = js.dump();
    auto res =
        Call<unitree_api::msg::Request, unitree_api::msg::Response>(req, js);
    return res;
  }

  int32_t PlayStream(const std::string &app_name, const std::string &stream_id,
                     const std::vector<uint8_t> &pcm_data) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_START_PLAY;
    nlohmann::json js;
    js["app_name"] = app_name;
    js["stream_id"] = stream_id;
    req.parameter = js.dump();
    req.binary = pcm_data;
    auto res =
        Call<unitree_api::msg::Request, unitree_api::msg::Response>(req, js);
    return res;
  }

  int32_t PlayStop(const std::string &app_name) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_STOP_PLAY;
    nlohmann::json js;
    js["app_name"] = app_name;
    req.parameter = js.dump();
    auto res =
        Call<unitree_api::msg::Request, unitree_api::msg::Response>(req, js);
    return res;
  }

  int32_t LedControl(uint8_t r, uint8_t g, uint8_t b) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_SET_RGB_LED;
    nlohmann::json js;
    js["R"] = r;
    js["G"] = g;
    js["B"] = b;
    req.parameter = js.dump();
    auto res =
        Call<unitree_api::msg::Request, unitree_api::msg::Response>(req, js);
    return res;
  }
};

}  // namespace unitree::ros2::g1
