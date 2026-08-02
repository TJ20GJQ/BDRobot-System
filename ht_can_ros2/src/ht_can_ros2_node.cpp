#include <cstdio>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ht_can_ros2/ht_can_control.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ht_can_control::HtCanControl>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::Rate rate(1ms);

  while(rclcpp::ok())
  {
    executor.spin_some();

    //do sth

    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
