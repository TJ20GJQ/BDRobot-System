#include <bdrobot_arm_controller/moveit_base.hpp>

int main(int argc, char* argv[])
{ 
  rclcpp::init(argc, argv);  // 初始化ROS节点
  
  auto move_group_node = rclcpp::Node::make_shared("fk_demo_node");  // 创建并共享一个名为"fk_demo_node"的节点
  
  rclcpp::executors::SingleThreadedExecutor executor;  // 创建一个单线程执行器
  executor.add_node(move_group_node);  // 将节点添加到执行器中
  std::thread([&executor]() { executor.spin(); }).detach();  // 启动执行器，使其在单独的线程中运行

  bdrobot_arm_controller::BDRobotArm bdrobot_arm(move_group_node);  // 创建BDRobotArm对象，用于控制机械臂
  bdrobot_arm.set_MaxVelocity_Accelerate(1.0, 1.0);
  bdrobot_arm.moveJ({-0.5, -0.5, 0.2, 0.0, 0.1});  // 控制机械臂移动到指定位置
  
  bdrobot_arm.get_current_pose();
  rclcpp::sleep_for(std::chrono::seconds(10));  // 等待10秒
  bdrobot_arm.move_Init();  // 移动至自定义点
  
  rclcpp::shutdown();  // 关闭ROS节点
  return 0;
}
