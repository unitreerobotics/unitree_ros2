#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "base_client.hpp"
#include "common/time_tools.hpp"
#include "common/ut_errror.hpp"
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
  BaseClient base_client_;

 public:
  AudioClient()
      : rclcpp::Node("audio_client"),
        base_client_(this, "/api/voice/request", "/api/voice/response") {};

  int32_t TtsMaker(const std::string &text, int32_t speaker_id) {
    nlohmann::json js;
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_TTS;
    js["index"] = tts_index_++;
    js["text"] = text;
    js["speaker_id"] = speaker_id;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t GetVolume(uint8_t &volume) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_GET_VOLUME;
    nlohmann::json js;
    auto res = base_client_.Call(req, js);
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
    return base_client_.Call(req);
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
    return base_client_.Call(req);
  }

  int32_t PlayStop(const std::string &app_name) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_STOP_PLAY;
    nlohmann::json js;
    js["app_name"] = app_name;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }

  int32_t LedControl(uint8_t r, uint8_t g, uint8_t b) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_API_ID_AUDIO_SET_RGB_LED;
    nlohmann::json js;
    js["R"] = r;
    js["G"] = g;
    js["B"] = b;
    req.parameter = js.dump();
    return base_client_.Call(req);
  }
};

}  // namespace unitree::ros2::g1
