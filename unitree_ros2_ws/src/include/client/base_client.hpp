#pragma once

#include <rclcpp/rclcpp.hpp>
#include "client/config.hpp"
#include "unitree_api/srv/generic.hpp"

#define WAIT_ROS_SERVICE_INTERVAL (1)
#define WAIT_ROS_SERVICE_MAX_TRY_TIME (5)

#define ERR_ROS_SERVICE_NOT_READY (1001)
#define ERR_ROS_API_TIMEOUT (1002)
#define ERR_ROS_API_GET_FUTURE (1003)
#define ERR_ROS_API_CALL_FAIL (1004)
#define ERR_ROS_API_INVALID_PARAM (1005)

#define SEND_REQUEST(PARAM, REQUEST_FUNC, ...)                                                \
    auto request = std::make_shared<unitree_api::srv::Generic::Request>();                    \
    std::shared_ptr<unitree_api::srv::Generic::Response> response;                            \
    PARAM.REQUEST_FUNC(__VA_ARGS__ __VA_OPT__(, ) request);                                   \
    int32_t ret = Call(request, response);                                                    \
    if (ret != 0)                                                                             \
    {                                                                                         \
        RCLCPP_ERROR(this->get_logger(), "%s api call failed . ret : %d", __FUNCTION__, ret); \
    }                                                                                         \
    if (response == nullptr)                                                                  \
    {                                                                                         \
        return ret;                                                                           \
    }

#define PARSE_RESPONSE(PARAM, RESPONSE_FUNC, ...) \
    return PARAM.RESPONSE_FUNC(response __VA_OPT__(, ) __VA_ARGS__);

namespace unitree
{
namespace ros2
{

class BaseClient : public rclcpp::Node,public ClientConfig
{
public:
    BaseClient(const std::string &nodeName, const std::string &serviceName) : Node(nodeName), mServiceName(serviceName)
    {
        mClient = this->create_client<unitree_api::srv::Generic>(mServiceName);
    }
    virtual ~BaseClient()
    {
    }

protected:
    int32_t Call(const std::shared_ptr<unitree_api::srv::Generic::Request> &request, std::shared_ptr<unitree_api::srv::Generic::Response> &response)
    {
        if (!wait_service())
        {
            RCLCPP_ERROR(this->get_logger(), "service %s is not ready", mServiceName);
            return ERR_ROS_SERVICE_NOT_READY;
        }
        auto future = mClient->async_send_request(request);
        uint32_t timeout = GetApiTimeout(request->api_id);
        auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(timeout));
        if (status == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (request->fastreply)
            {
                return 0;
            }
            if (future.valid())
            {
                response = future.get();
                return 0;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "api call get future failed");
                return ERR_ROS_API_GET_FUTURE;
            }
        }
        else if (status == rclcpp::FutureReturnCode::TIMEOUT)
        {
            RCLCPP_ERROR(this->get_logger(), "api call timeout %d(s)", timeout);
            SetServiceStatus(false);
            return ERR_ROS_API_TIMEOUT;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "api call failed . status : %d", status);
            return ERR_ROS_API_CALL_FAIL;
        }
    }

private:
    bool wait_service()
    {
        if (IsServiceReady())
        {
            return true;
        }
        int32_t tryTimes = 0;
        while (!mClient->wait_for_service(std::chrono::seconds(WAIT_ROS_SERVICE_INTERVAL)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service %s to be available...", mServiceName);
                return false;
            }
            ++tryTimes;
            if (tryTimes >= WAIT_ROS_SERVICE_MAX_TRY_TIME)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to find service : %s", mServiceName);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service %s to be available...", mServiceName);
        }
        SetServiceStatus(true);
        return true;
    }

    std::string mServiceName;
    rclcpp::Client<unitree_api::srv::Generic>::SharedPtr mClient;
};

}
}