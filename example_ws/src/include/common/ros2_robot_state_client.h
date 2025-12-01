/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 NOTICE: This client is only available on ROS2 Humble.
***********************************************************************/
#ifndef ROS2_ROBOT_STATE_CLIENT_H_
#define ROS2_ROBOT_STATE_CLIENT_H_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <future>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
 
// Constants for API IDs
constexpr int32_t ROBOT_STATE_API_ID_SERVICE_SWITCH = 1001;
constexpr int32_t ROBOT_STATE_API_ID_SET_REPORT_FREQ = 1002;
constexpr int32_t ROBOT_STATE_API_ID_SERVICE_LIST = 1003;

// Data structures
struct ServiceState {
    std::string name;
    int32_t status;
    int32_t protect;
};

struct ServiceSwitchRequest {
    std::string name;
    int32_t swit;
};

struct ServiceSwitchResponse {
    std::string name;
    int32_t status{};
};

struct SetReportFreqRequest {
    int32_t interval;
    int32_t duration;
};


// JSON serialization/deserialization
inline void from_json(const nlohmann::json& j, ServiceState& item) {
    j.at("name").get_to(item.name);
    j.at("status").get_to(item.status);
    j.at("protect").get_to(item.protect);
}

inline void to_json(nlohmann::json& j, const ServiceSwitchRequest& item) {
    j = nlohmann::json{{"name", item.name}, {"switch", item.swit}};
}

inline void from_json(const nlohmann::json& j, ServiceSwitchResponse& item) {
    j.at("name").get_to(item.name);
    j.at("status").get_to(item.status); 
}

inline void to_json(nlohmann::json& j, const SetReportFreqRequest& item) {
    j = nlohmann::json{{"interval", item.interval}, {"duration", item.duration}};
}
 
class RobotStateClient {
public:
    explicit RobotStateClient(rclcpp::Node* node) 
        : node_(node),
          req_puber_(node_->create_publisher<unitree_api::msg::Request>(
              "/api/robot_state/request", 10)) {
    }
  
    int32_t ServiceList(std::vector<ServiceState>& list) {
        unitree_api::msg::Request req;
        req.header.identity.api_id = ROBOT_STATE_API_ID_SERVICE_LIST;
        req.header.identity.id = GetSystemUptimeInNanoseconds();
        auto js = Call<unitree_api::msg::Request, unitree_api::msg::Response>(req);
        js.get_to(list);
        return 0;
    }
 
    int32_t ServiceSwitch(const std::string& name, int32_t swit, int32_t& status) {
        unitree_api::msg::Request req;
        req.header.identity.api_id = ROBOT_STATE_API_ID_SERVICE_SWITCH;
        req.header.identity.id = GetSystemUptimeInNanoseconds();
        nlohmann::json js;
        js["name"] = name;
        js["switch"] = swit;
        req.parameter = js.dump();
        
        auto js_res = Call<unitree_api::msg::Request, unitree_api::msg::Response>(req);
        ServiceSwitchResponse response;
        js_res.get_to(response);
        status = response.status;
        return 0;
    }
 
    int32_t SetReportFreq(int32_t interval, int32_t duration) {
        unitree_api::msg::Request req;
        req.header.identity.api_id = ROBOT_STATE_API_ID_SET_REPORT_FREQ;
        
        nlohmann::json js;
        js["interval"] = interval;
        js["duration"] = duration;
        req.parameter = js.dump();
        req.header.identity.id = GetSystemUptimeInNanoseconds();
        
        req_puber_->publish(req);
        return 0;
    }

private:
    template<typename Request, typename Response>
    nlohmann::json Call(const Request& req) {
        std::promise<typename Response::SharedPtr> response_promise;
        auto response_future = response_promise.get_future();
        const auto api_id = req.header.identity.api_id;
        
        auto req_suber_ = node_->create_subscription<Response>(
            "/api/robot_state/response", rclcpp::QoS(1),
            [&response_promise, api_id](const typename Response::SharedPtr data) {
                if (data->header.identity.api_id == api_id) {
                    response_promise.set_value(data);
                }
            });

        req_puber_->publish(req);

        auto response = *response_future.get();
        return nlohmann::json::parse(response.data.data());
    }

    static int64_t GetSystemUptimeInNanoseconds() {
        struct timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
    }

    rclcpp::Node* node_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
};

#endif // ROS2_ROBOT_STATE_CLIENT_H_