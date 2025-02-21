#pragma once
#include <unordered_map>

namespace unitree
{
namespace ros2
{

struct CliConfig
{
    uint32_t timeout;
};

class ClientConfig
{
public:
    void SetApiTimeout(int32_t apiId, uint32_t timeout);
    uint32_t GetApiTimeout(int32_t apiId);

private:
    bool GetConfig(int32_t apiId, CliConfig &config);

    std::unordered_map<int32_t, CliConfig> mConfigMap;
};
}
}
