#pragma once

#define WAIT_ROS_SERVICE_INTERVAL (1)
#define WAIT_ROS_SERVICE_MAX_TRY_TIME (10)

#define ERR_ROS_SERVICE_NOT_READY (1001)
#define ERR_ROS_API_TIMEOUT (1002)
#define ERR_ROS_API_GET_FUTURE (1003)

#define SEND_REQUEST(CLIENT, PARAM, REQUEST_FUNC, ...)                     \
    if (!wait_service())                                                   \
    {                                                                      \
        return ERR_ROS_SERVICE_NOT_READY;                                  \
    }                                                                      \
    auto request = std::make_shared<unitree_api::srv::Generic::Request>(); \
    PARAM.REQUEST_FUNC(__VA_ARGS__ __VA_OPT__(, ) request);                \
    auto future = CLIENT->async_send_request(request);

#define PARSE_RESPONSE(PARAM, RESPONSE_FUNC, ...)                                                                             \
    uint32_t timeout = GetApiTimeout(request->api_id);                                                                        \
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(timeout)); \
    if (status == rclcpp::FutureReturnCode::SUCCESS)                                                                          \
    {                                                                                                                         \
        if (future.get())                                                                                                     \
        {                                                                                                                     \
            return PARAM.RESPONSE_FUNC(future.get() __VA_OPT__(, ) __VA_ARGS__);                                              \
        }                                                                                                                     \
        else                                                                                                                  \
        {                                                                                                                     \
            RCLCPP_ERROR(this->get_logger(), "%s api call get future failed", __FUNCTION__);                                  \
            return ERR_ROS_API_GET_FUTURE;                                                                                    \
        }                                                                                                                     \
    }                                                                                                                         \
    else                                                                                                                      \
    {                                                                                                                         \
        RCLCPP_ERROR(this->get_logger(), "%s api call timeout %d(s)", __FUNCTION__, timeout);                                 \
        return ERR_ROS_API_TIMEOUT;                                                                                           \
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
