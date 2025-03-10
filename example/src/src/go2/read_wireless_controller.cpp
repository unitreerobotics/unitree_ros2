/**
 * This example demonstrates how to use ROS2 to receive wireless controller states of unitree go2 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg//wireless_controller.hpp"

using std::placeholders::_1;

class wireless_controller_suber : public rclcpp::Node
{
public:
  wireless_controller_suber() : Node("wireless_controller_suber")
  {
    // the cmd_puber is set to subscribe "/wirelesscontroller" topic
    suber = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller", 10, std::bind(&wireless_controller_suber::topic_callback, this, _1));
  }

private:
  void topic_callback(unitree_go::msg::WirelessController::SharedPtr data)
  {
    // lx: Left joystick x value
    // ly: Left joystick y value
    // rx: Right joystick x value
    // ry: Right joystick y value
    // keys value

    RCLCPP_INFO(this->get_logger(), "Wireless controller -- lx: %f; ly: %f; rx: %f; ry: %f; key value: %d",
                data->lx, data->ly, data->rx, data->ry, data->keys);
  }

  rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr suber;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv); // Initialize rclcpp
  // Run ROS2 node which is make share with wireless_controller_suber class
  rclcpp::spin(std::make_shared<wireless_controller_suber>()); 
  rclcpp::shutdown();
  return 0;
}