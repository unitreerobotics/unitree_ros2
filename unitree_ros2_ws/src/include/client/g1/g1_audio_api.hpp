#pragma once

#include "unitree_api/srv/generic.hpp"

namespace unitree
{
namespace ros2
{
namespace g1
{

constexpr int32_t ROBOT_API_ID_AUDIO_TTS = 1001;
constexpr int32_t ROBOT_API_ID_AUDIO_ASR = 1002;
constexpr int32_t ROBOT_API_ID_AUDIO_START_PLAY = 1003;
constexpr int32_t ROBOT_API_ID_AUDIO_STOP_PLAY = 1004;
constexpr int32_t ROBOT_API_ID_AUDIO_GET_VOLUME = 1005;
constexpr int32_t ROBOT_API_ID_AUDIO_SET_VOLUME = 1006;
constexpr int32_t ROBOT_API_ID_AUDIO_SET_RGB_LED = 1010;

class AudioClientApi
{
public:
    void TtsMakerReq(const std::string &text, int32_t speaker_id, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t TtsMakerRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void GetVolumeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t GetVolumeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, uint8_t &volume);

    void SetVolumeReq(uint8_t volume, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t SetVolumeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void PlayStreamReq(const std::string &app_name, const std::string &stream_id, const std::vector<uint8_t> &pcm_data, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t PlayStreamRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void PlayStopReq(const std::string &app_name, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t PlayStopRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

    void LedControlReq(uint8_t R, uint8_t G, uint8_t B, const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

    int32_t LedControlRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

private:
  uint32_t mTtsIndex = 0;
};

}
}
}