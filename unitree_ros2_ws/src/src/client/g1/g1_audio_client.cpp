#include "client/g1/g1_audio_client.hpp"
#include "client/decl.hpp"

using namespace unitree::ros2::g1;

#define AUDIO_SERVICE_NAME "voice"
#define SEND_AUDIO_REQUEST(REQUEST_FUNC, ...) SEND_REQUEST(mClient, mParam, REQUEST_FUNC, __VA_ARGS__)
#define PARSE_AUDIO_RESPONSE(RESPONSE_FUNC, ...) PARSE_RESPONSE(mParam, RESPONSE_FUNC, __VA_ARGS__)

AudioClient::AudioClient(const std::string &nodeName) : Node(nodeName)
{
    mClient = this->create_client<unitree_api::srv::Generic>(AUDIO_SERVICE_NAME);
}

int32_t AudioClient::TtsMaker(const std::string &text, int32_t speaker_id)
{
    SEND_AUDIO_REQUEST(TtsMakerReq, text, speaker_id)
    PARSE_AUDIO_RESPONSE(TtsMakerRes)
}

int32_t AudioClient::GetVolume(uint8_t &volume)
{
    SEND_AUDIO_REQUEST(GetVolumeReq)
    PARSE_AUDIO_RESPONSE(GetVolumeRes, volume)
}

int32_t AudioClient::SetVolume(uint8_t volume)
{
    SEND_AUDIO_REQUEST(SetVolumeReq, volume)
    PARSE_AUDIO_RESPONSE(SetVolumeRes)
}

int32_t AudioClient::PlayStream(const std::string &app_name, const std::string &stream_id, const std::vector<uint8_t> &pcm_data)
{
    SEND_AUDIO_REQUEST(PlayStreamReq, app_name, stream_id, pcm_data)
    PARSE_AUDIO_RESPONSE(PlayStreamRes)
}

int32_t AudioClient::PlayStop(const std::string &app_name)
{
    SEND_AUDIO_REQUEST(PlayStopReq, app_name)
    PARSE_AUDIO_RESPONSE(PlayStopRes)
}

int32_t AudioClient::LedControl(uint8_t R, uint8_t G, uint8_t B)
{
    SEND_AUDIO_REQUEST(LedControlReq, R, G, B)
    PARSE_AUDIO_RESPONSE(LedControlRes)
}

bool unitree::ros2::g1::AudioClient::wait_service()
{
    WAIT_SERVICE(mClient, AUDIO_SERVICE_NAME)
}
