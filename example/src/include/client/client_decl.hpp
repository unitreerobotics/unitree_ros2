#pragma once

#define WAIT_ROS_SERVICE_INTERVAL (1)
#define WAIT_ROS_SERVICE_MAX_TRY_TIME (10)

#define SEND_REQUEST(CLIENT, PARAM, REQUEST_FUNC, ...)                     \
    if (!wait_service())                                                   \
    {                                                                      \
        return -1;                                                         \
    }                                                                      \
    auto request = std::make_shared<unitree_api::srv::Generic::Request>(); \
    PARAM.REQUEST_FUNC(__VA_ARGS__ __VA_OPT__(, ) request);                \
    auto future = CLIENT->async_send_request(request);

#define PARSE_RESPONSE(PARAM, RESPONSE_FUNC, ...)                                \
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future); \
    if (future.get())                                                            \
    {                                                                            \
        return PARAM.RESPONSE_FUNC(future.get() __VA_OPT__(, ) __VA_ARGS__);     \
    }                                                                            \
    else                                                                         \
    {                                                                            \
        RCLCPP_ERROR(this->get_logger(), "%s api call failed", __FUNCTION__);    \
        return -1;                                                               \
    }

#define WAIT_SERVICE(CLIENT, SERVICE_NAME)                                                                    \
    int32_t tryTimes = 0;                                                                                     \
    while (!CLIENT->wait_for_service(std::chrono::seconds(WAIT_ROS_SERVICE_INTERVAL)))                        \
    {                                                                                                         \
        if (!rclcpp::ok())                                                                                    \
        {                                                                                                     \
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service to be available..."); \
            return false;                                                                                     \
        }                                                                                                     \
        ++tryTimes;                                                                                           \
        if (tryTimes >= WAIT_ROS_SERVICE_MAX_TRY_TIME)                                                        \
        {                                                                                                     \
            RCLCPP_ERROR(this->get_logger(), "Failed to find service : %s", SERVICE_NAME);                    \
        }                                                                                                     \
        RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");                        \
    }                                                                                                         \
    return true;
