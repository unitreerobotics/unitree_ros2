#pragma once

#include "client/base_client.hpp"
#include "client/g1/g1_audio_api.hpp"

namespace unitree
{
namespace ros2
{
namespace g1
{

class AudioClient: public BaseClient
{
public:
    AudioClient(const std::string &nodeName = "g1_voice_lient");

    int32_t TtsMaker(const std::string &text, int32_t speaker_id);

    int32_t GetVolume(uint8_t &volume);

    int32_t SetVolume(uint8_t volume);

    int32_t PlayStream(const std::string &app_name, const std::string &stream_id, const std::vector<uint8_t> &pcm_data);

    int32_t PlayStop(const std::string &app_name);

    int32_t LedControl(uint8_t R, uint8_t G, uint8_t B);

private:

    AudioClientApi mParam;
};

}
}
}
