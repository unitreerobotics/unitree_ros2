#pragma once
#include <cstdint>
#define UT_DECL_ERR(name, code, desc) \
  const int32_t name = code;          \
  const constexpr char* name##_DESC = desc;

UT_DECL_ERR(UT_ROBOT_SUCCESS, 0, "success.")

UT_DECL_ERR(UT_ROBOT_TASK_TIMEOUT, -1, "task timeout.")

UT_DECL_ERR(UT_ROBOT_TASK_UNKNOWN_ERROR, -2, "task unknown error.")

// Returned by the client when a request was never handed to DDS. The value is
// the official SDK2 client send-request error (UT_ROBOT_ERR_CLIENT_SEND), which
// SDK2 assigns to the client side. A server status is passed through unchanged.
// UT_ROBOT_TASK_TIMEOUT keeps this package's value of -1; SDK2 uses 3104.
UT_DECL_ERR(UT_ROBOT_TASK_NOT_PUBLISHED, 3102, "task not published before its deadline.")

#define UT_PRINT_ERR(code, error)                         \
  if ((code) == (error)) {                                \
    RCLCPP_ERROR(this->get_logger(), "%s", error##_DESC); \
  }
