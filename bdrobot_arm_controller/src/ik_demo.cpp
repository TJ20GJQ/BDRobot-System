#include <bdrobot_arm_controller/moveit_base.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
void custom_task(rclcpp::Node::SharedPtr node)
{
	int counter = 0;
	while (rclcpp::ok()) {
		counter++;
		RCLCPP_INFO(node->get_logger(), "自定义任务执行: %d", counter);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}

int main(int argc, char* argv[])
{ 
	rclcpp::init(argc, argv);  // 初始化ROS节点

	auto move_group_node = rclcpp::Node::make_shared("ik_demo_node");  // 创建并共享一个名为"ik_demo_node"的节点
// ROS 2 使用 执行器 来管理回调函数的执行：
// 执行器 是 ROS 2 中负责处理回调函数的组件
// 它监听订阅的话题、服务请求、定时器等事件
// 当事件发生时，执行器会在其运行的线程中调用相应的回调函数
// executor.spin()：阻塞当前线程，持续处理回调，直到节点被关闭（所以有订阅的情况下，需要在单独的线程中运行回调）
	rclcpp::executors::SingleThreadedExecutor executor;  // 创建一个单线程执行器
	executor.add_node(move_group_node);  // 将节点添加到执行器中
	std::thread([&executor]() { executor.spin(); }).detach();  // 启动执行器，使其在单独的线程中运行 .detach()：即使main结束依然运行
	
	bdrobot_arm_controller::BDRobotArm bdrobot_arm(move_group_node);  // 创建BDRobotArm对象，用于控制机械臂
	bdrobot_arm.set_visualization(true);
	bdrobot_arm.set_MaxVelocity_Accelerate(1.0, 1.0);

	rclcpp::sleep_for(std::chrono::seconds(5));  // 等待5秒
	geometry_msgs::msg::Pose current_pose = bdrobot_arm.get_current_pose();

	// 测试位姿有效性（暂未构建出测试数据）
	geometry_msgs::msg::Pose invalid_pose;
	invalid_pose.position.x = current_pose.position.x;
	invalid_pose.position.y = current_pose.position.y-0.1;
	invalid_pose.position.z = current_pose.position.z+0.5;
	invalid_pose.orientation.x = current_pose.orientation.x;
	invalid_pose.orientation.y = current_pose.orientation.y;
	invalid_pose.orientation.z = current_pose.orientation.z;
	invalid_pose.orientation.w = current_pose.orientation.w;
	RCLCPP_INFO(move_group_node->get_logger(), "invalid_pose: %d", bdrobot_arm.checkIKvalid(invalid_pose));
	
	bdrobot_arm.get_rviz_tools()->prompt("next");

	// 测试笛卡尔路径规划
	auto success = bdrobot_arm.moveP(current_pose.position.x,
			current_pose.position.y-0.1,
			current_pose.position.z+0.5,
			current_pose.orientation.x,
			current_pose.orientation.y,
			current_pose.orientation.z,
			current_pose.orientation.w
			);
	
	current_pose = bdrobot_arm.get_current_pose();
	
	if (success == moveit::core::MoveItErrorCode::SUCCESS) {
		RCLCPP_INFO(move_group_node->get_logger(), "目标路径规划成功，开始执行");
	} else {
		RCLCPP_ERROR(move_group_node->get_logger(), "目标路径规划失败，请调整目标位姿");
		while(true);
		return 0;
	}

	rclcpp::sleep_for(std::chrono::seconds(5));  // 等待5秒
	// bdrobot_arm.move_Init();  // 移动至自定义点
	
	rclcpp::shutdown();  // 关闭ROS节点
	return 0;
}
