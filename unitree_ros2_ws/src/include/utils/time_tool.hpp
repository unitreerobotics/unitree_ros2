#pragma once

#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>

namespace unitree
{
namespace common
{
    uint64_t GetCurrentTimeSeconds() 
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    }

    uint64_t GetCurrentTimeMilliseconds() 
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    }

    uint64_t GetCurrentTimeMicroseconds() 
    {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }

    std::string GetCurrentTimeStr(const std::string& format = "%Y-%m-%d %H:%M:%S") 
    {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm tm_time = *std::localtime(&now_time);

        std::ostringstream oss;
        oss << std::put_time(&tm_time, format.c_str());
        return oss.str();
    }

    uint64_t GetDurationSeconds(uint64_t start_time_seconds) 
    {
        return GetCurrentTimeSeconds() - start_time_seconds;
    }

    uint64_t GetDurationMilliseconds(uint64_t start_time_milliseconds) 
    {
        return GetCurrentTimeMilliseconds() - start_time_milliseconds;
    }

    uint64_t GetDurationMicroseconds(uint64_t start_time_microseconds) 
    {
        return GetCurrentTimeMicroseconds() - start_time_microseconds;
    }

}
}
