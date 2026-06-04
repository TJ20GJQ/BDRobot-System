#include <bdrobot_arm_controller/moveit_base.hpp>


int main(int argc, char* argv[])
{ 
  rclcpp::init(argc, argv);  // 初始化ROS节点
  
  auto move_group_node = rclcpp::Node::make_shared("move_cartesian_demo_node");  // 创建并共享一个名为"move_cartesian_demo_node"的节点
  
  rclcpp::executors::SingleThreadedExecutor executor;  // 创建一个单线程执行器
  executor.add_node(move_group_node);  // 将节点添加到执行器中
  std::thread([&executor]() { executor.spin(); }).detach();  // 启动执行器，使其在单独的线程中运行

  bdrobot_arm_controller::BDRobotArm bdrobot_arm(move_group_node);  // 创建BDRobotArm对象，用于控制机械臂
  bdrobot_arm.set_MaxVelocity_Accelerate(1.0, 1.0);

  bdrobot_arm.move_Init();  // 移动至自定义点
  rclcpp::sleep_for(std::chrono::seconds(5));  // 等待5秒

  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Pose> poses;
  pose = bdrobot_arm.get_current_pose();
  RCLCPP_INFO(move_group_node->get_logger(), "current_pose: %f, %f, %f, %f, %f, %f, %f\n", pose.position.x,
          pose.position.y,
          pose.position.z,
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w);
  poses.push_back(pose);

  // pose.position.z += 0.1;
  // poses.push_back(pose);

  // pose.position.z += 0.2;
  // poses.push_back(pose);

  pose.position.y -= 0.1;
  pose.position.z += 0.5;
  poses.push_back(pose);

  bdrobot_arm.moveCartesian(poses);  // 执行笛卡尔路径规划

    // 关闭ROS节点
  rclcpp::shutdown();
  return 0;
}
