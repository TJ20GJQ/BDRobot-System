#include "robot_self_filter/robot_self_filter.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_self_filter::RobotSelfFilter>());
    rclcpp::shutdown();
    return 0;
}
