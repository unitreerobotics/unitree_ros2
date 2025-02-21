#include "client/config.hpp"

using namespace unitree::ros2;

constexpr uint32_t DEFAULT_API_TIME_OUT = 10;

void ClientConfig::SetApiTimeout(int32_t apiId, uint32_t timeout)
{
    CliConfig config;
    GetConfig(apiId, config);
    config.timeout = timeout;
    mConfigMap[apiId] = config;
}

uint32_t unitree::ros2::ClientConfig::GetApiTimeout(int32_t apiId)
{
    CliConfig config;
    if (GetConfig(apiId, config))
    {
        return config.timeout;
    }
    else
    {
        return DEFAULT_API_TIME_OUT;
    }
}

bool ClientConfig::GetConfig(int32_t apiId, CliConfig &config)
{
    if (mConfigMap.count(apiId) == 0)
    {
        return false;
    }
    config = mConfigMap[apiId];

    return true;
}