#include "client/g1/g1_audio_api.hpp"
#include "nlohmann/json.hpp"

using namespace unitree::ros2::g1;

void AudioClientApi::TtsMakerReq(const std::string &text, int32_t speaker_id, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["index"] = mTtsIndex++;
    js["text"] = text;
    js["speaker_id"] = speaker_id;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_AUDIO_TTS;
}

int32_t AudioClientApi::TtsMakerRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void AudioClientApi::GetVolumeReq(const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    req->api_id = ROBOT_API_ID_AUDIO_GET_VOLUME;
}

int32_t AudioClientApi::GetVolumeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res, uint8_t &volume)
{
    if (res->code == 0)
    {
        auto js = nlohmann::json::parse(res->data.c_str());
        volume = js.at("volume");
    }
    return res->code;
}

void AudioClientApi::SetVolumeReq(uint8_t volume, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["volume"] = volume;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_AUDIO_SET_VOLUME;
}

int32_t AudioClientApi::SetVolumeRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void AudioClientApi::PlayStreamReq(const std::string &app_name, const std::string &stream_id,
                                   const std::vector<uint8_t> &pcm_data, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["app_name"] = app_name;
    js["stream_id"] = stream_id;
    req->parameter = js.dump();
    req->binary = pcm_data;
    req->fastreply = true;
    req->api_id = ROBOT_API_ID_AUDIO_START_PLAY;
}

int32_t AudioClientApi::PlayStreamRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void AudioClientApi::PlayStopReq(const std::string &app_name, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["app_name"] = app_name;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_AUDIO_STOP_PLAY;
}

int32_t AudioClientApi::PlayStopRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}

void AudioClientApi::LedControlReq(uint8_t R, uint8_t G, uint8_t B, const std::shared_ptr<unitree_api::srv::Generic::Request> &req)
{
    nlohmann::json js;
    js["R"] = R;
    js["G"] = G;
    js["B"] = B;
    req->parameter = js.dump();
    req->api_id = ROBOT_API_ID_AUDIO_SET_RGB_LED;
}

int32_t AudioClientApi::LedControlRes(const std::shared_ptr<unitree_api::srv::Generic::Response> &res)
{
    return res->code;
}
