/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include "common/motor_crc_hg.h"

// Create a low_level_cmd_sender class for low state receive
class low_level_cmd_sender : public rclcpp::Node
{
public:
    low_level_cmd_sender() : Node("low_level_cmd_sender")
    {
        // the cmd_puber is set to subscribe "/lowcmd" topic
        cmd_puber = this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);

        // The timer is set to 200Hz, and bind to low_level_cmd_sender::timer_callback function
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&low_level_cmd_sender::timer_callback, this));

        // Initialize lowcmd
        init_cmd();

        // Running time count
        t = 0;
    }

private:
    void timer_callback()
    {
        // Test code here
        t += 0.02;

        // Toque controle, set RL_2 toque
        cmd_msg.motor_cmd[LeftAnklePitch].q = 0; // Set to stop position(rad)
        cmd_msg.motor_cmd[LeftAnklePitch].kp = 0;
        cmd_msg.motor_cmd[LeftAnklePitch].dq = 0; // Set to stop angular velocity(rad/s)
        cmd_msg.motor_cmd[LeftAnklePitch].kd = 0;
        cmd_msg.motor_cmd[LeftAnklePitch].tau = 1; // target toque is set to 1N.m

        // Poinstion(rad) control, set RL_0 rad
        cmd_msg.motor_cmd[RightAnklePitch].q = 0;   // Taregt angular(rad)
        cmd_msg.motor_cmd[RightAnklePitch].kp = 10; // Poinstion(rad) control kp gain
        cmd_msg.motor_cmd[RightAnklePitch].dq = 0;  // Taregt angular velocity(rad/ss)
        cmd_msg.motor_cmd[RightAnklePitch].kd = 1;  // Poinstion(rad) control kd gain
        cmd_msg.motor_cmd[RightAnklePitch].tau = 0; // Feedforward toque 1N.m

        get_crc(cmd_msg); // Check motor cmd crc
        std::cout << cmd_msg.crc << std::endl;

        cmd_puber->publish(cmd_msg); // Publish lowcmd message
    }

    void init_cmd()
    {

        for (int i = 0; i < 35; i++)
        {
            cmd_msg.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
            cmd_msg.motor_cmd[i].q = 0.0;
            cmd_msg.motor_cmd[i].kp = 0.0;
            cmd_msg.motor_cmd[i].dq = 0.0;
            cmd_msg.motor_cmd[i].kd = 0.0;
            cmd_msg.motor_cmd[i].tau = 0.0;
            cmd_msg.motor_cmd[i].reserve = 0;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;                             // ROS2 timer
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr cmd_puber; // ROS2 Publisher

    unitree_hg::msg::LowCmd cmd_msg; // Unitree go2 lowcmd message
    double t;                        // Running time count
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                             // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_;                  // Create a timer callback object to send cmd in time intervals
    auto node = std::make_shared<low_level_cmd_sender>(); // Create a ROS2 node and make share with low_level_cmd_sender class
    rclcpp::spin(node);                                   // Run ROS2 node
    rclcpp::shutdown();                                   // Exit
    return 0;
}
