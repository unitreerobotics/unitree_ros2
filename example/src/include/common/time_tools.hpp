#pragma once

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

namespace unitree::common {
inline int64_t GetSystemUptimeInNanoseconds() {
  struct timespec ts {};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

inline uint64_t GetCurrentTimeSeconds() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
}

inline uint64_t GetCurrentTimeMilliseconds() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration)
      .count();
}

inline uint64_t GetCurrentTimeMicroseconds() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(duration)
      .count();
}

inline std::string GetCurrentTimeStr(
    const std::string& format = "%Y-%m-%d %H:%M:%S") {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm_time = *std::localtime(&now_time);

  std::ostringstream oss;
  oss << std::put_time(&tm_time, format.c_str());
  return oss.str();
}

inline uint64_t GetDurationSeconds(uint64_t start_time_seconds) {
  return GetCurrentTimeSeconds() - start_time_seconds;
}

inline uint64_t GetDurationMilliseconds(uint64_t start_time_milliseconds) {
  return GetCurrentTimeMilliseconds() - start_time_milliseconds;
}

inline uint64_t GetDurationMicroseconds(uint64_t start_time_microseconds) {
  return GetCurrentTimeMicroseconds() - start_time_microseconds;
}

}  // namespace unitree::common
