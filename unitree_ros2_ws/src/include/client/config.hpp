#pragma once
#include <unordered_map>
#include <atomic>

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
    ClientConfig() : mServiceReady(false) {}
    virtual ~ClientConfig() {}

    void SetApiTimeout(int32_t apiId, uint32_t timeout);
    uint32_t GetApiTimeout(int32_t apiId);

    bool IsServiceReady()
    {
        return mServiceReady;
    }

protected:
    void SetServiceStatus(bool ready)
    {
        mServiceReady = ready;
    }

private:
    bool GetConfig(int32_t apiId, CliConfig &config);

    std::unordered_map<int32_t, CliConfig> mConfigMap;

    std::atomic_bool mServiceReady;
};
}
}
