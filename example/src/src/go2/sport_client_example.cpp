/**
 * This example demonstrates how to use ROS2 to call sport client api of unitree go2 robot
 **/
#include "client/go2/go2_sport_client.hpp"

using namespace unitree::ros2::go2;

int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        auto client = std::make_shared<SportClient>();
        client->SetApiTimeout(ROBOT_SPORT_API_ID_SIT, 5);
        // sit down
        client->Sit();
        std::this_thread::sleep_for(std::chrono::seconds(3));
        // recover
        client->RiseSit();
        rclcpp::shutdown();
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        std::cerr << "RCLError caught: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}