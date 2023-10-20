
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

using std::placeholders::_1;
// Create a soprt_request class for soprt commond request
class soprt_request : public rclcpp::Node
{
public:
    soprt_request() : Node("req_sender")
    {
        // the state_suber is set to subscribe "sportmodestate" topic
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&soprt_request::state_callback, this, _1));
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&soprt_request::timer_callback, this));

        t = -1; // Runing time count
    };

private:
    void timer_callback()
    {
        t += dt;
        if (t > 0)
        {
            double time_seg = 0.2;
            double time_temp = t - time_seg;

            std::vector<PathPoint> path;

            for (int i = 0; i < 30; i++)
            {
                PathPoint path_point_tmp;
                time_temp += time_seg;
                // Tacking a sin path in x direction
                // The path is respect to the initial coordinate system
                float px_local = 0.5 * sin(0.5 * time_temp);
                float py_local = 0;
                float yaw_local = 0.;
                float vx_local = 0.5 * cos(0.5 * time_temp);
                float vy_local = 0;
                float vyaw_local = 0.;

                // Convert trajectory commands to the initial coordinate system
                path_point_tmp.timeFromStart = i * time_seg;
                path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
                path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
                path_point_tmp.yaw = yaw_local + yaw0;
                path_point_tmp.vx = vx_local * cos(yaw0) - vy_local * sin(yaw0);
                path_point_tmp.vy = vx_local * sin(yaw0) + vy_local * cos(yaw0);
                path_point_tmp.vyaw = vyaw_local;
                path.push_back(path_point_tmp);
            }
            // Get request messages corresponding to high-level motion commands
            sport_req.TrajectoryFollow(req, path);
            // Publish request messages
            req_puber->publish(req);
        }
    };

    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // Get current position of robot when t<0
        // This position is used as the initial coordinate system

        if (t < 0)
        {
            // Get initial position
            px0 = data->position[0];
            py0 = data->position[1];
            yaw0 = data->imu_state.rpy[2];
            std::cout << px0 << ", " << py0 << ", " << yaw0 << std::endl;
        }
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;

    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    double t; // runing time count
    double dt = 0.002; //control time step

    double px0 = 0;  // initial x position
    double py0 = 0;  // initial y position
    double yaw0 = 0; // initial yaw angle
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals

    rclcpp::spin(std::make_shared<soprt_request>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
