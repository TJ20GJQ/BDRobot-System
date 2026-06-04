#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <bdrobot_arm_controller/moveit_base.hpp>
#include <bdrobot_arm_controller/action/move_to_goal.hpp>
#include <bdrobot_arm_controller/srv/check_ik_solution.hpp>

using MoveToGoal = bdrobot_arm_controller::action::MoveToGoal;
using GoalHandleMoveToGoal = rclcpp_action::ServerGoalHandle<MoveToGoal>;
using CheckIKSolution = bdrobot_arm_controller::srv::CheckIKSolution;

class MoveToGoalActionServer : public rclcpp::Node
{
public:
    explicit MoveToGoalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("move_to_goal_action_server", options)
    {
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "MoveToGoal Action Server created, waiting for initialization");

        // 创建CheckIKSolution服务
        this->check_ik_service_ = create_service<CheckIKSolution>(
            "check_ik_solution",
            std::bind(&MoveToGoalActionServer::check_ik_callback, this, _1, _2));

        // 创建Action服务器
        this->action_server_ = rclcpp_action::create_server<MoveToGoal>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "move_to_goal",
            std::bind(&MoveToGoalActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveToGoalActionServer::handle_cancel, this, _1),
            std::bind(&MoveToGoalActionServer::handle_accepted, this, _1));
    }

    void initialize()
    {
        // 定义规划组和末端执行器
        static const std::string PLANNING_GROUP = "arm_group";
        static const std::string PLANNING_EEF = "bounder_link";
        
        move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), PLANNING_GROUP);

        // 创建BDRobotArm对象
        bdrobot_arm_ = std::make_unique<bdrobot_arm_controller::BDRobotArm>(
            this->shared_from_this(), move_group_interface_.get(), PLANNING_EEF);
 
        // 设置最大速度和加速度
        bdrobot_arm_->set_MaxVelocity_Accelerate(0.5, 0.5);

        RCLCPP_INFO(this->get_logger(), "MoveToGoal Action Server initialized");
    }

private:
    rclcpp_action::Server<MoveToGoal>::SharedPtr action_server_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::unique_ptr<bdrobot_arm_controller::BDRobotArm> bdrobot_arm_;
    rclcpp::Service<CheckIKSolution>::SharedPtr check_ik_service_;

    void check_ik_callback(const CheckIKSolution::Request::SharedPtr request,
                          const CheckIKSolution::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received IK check request for target pose: position=(%.2f, %.2f, %.2f)", 
            request->target_pose.position.x, 
            request->target_pose.position.y, 
            request->target_pose.position.z);
 
        try {
            // 检查目标位姿是否有效
            bool solution_exists = bdrobot_arm_->checkIKvalid(request->target_pose);
            
            response->solution_exists = solution_exists;
            
            if (solution_exists) {
                response->message = "IK solution exists for the given pose";
                RCLCPP_INFO(this->get_logger(), "IK solution exists");
            } else {
                response->message = "No IK solution exists for the given pose";
                RCLCPP_WARN(this->get_logger(), "No IK solution exists");
            }
        } catch (const std::exception& e) {
            response->solution_exists = false;
            response->message = std::string("Exception occurred while checking IK: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Exception in IK check: %s", e.what());
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToGoal::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received goal request with target pose: position=(%.2f, %.2f, %.2f, )", 
            goal->target_pose.position.x, 
            goal->target_pose.position.y, 
            goal->target_pose.position.z);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        // 检查目标位姿是否有效
        // if (bdrobot_arm_->checkIKvalid(goal->target_pose)) {
        //     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "Target pose is not reachable");
        //     return rclcpp_action::GoalResponse::REJECT;
        // }
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
    {
        using namespace std::placeholders;
        // 这个方法在一个独立的线程中运行
        std::thread{std::bind(&MoveToGoalActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveToGoal> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        // 发送初始反馈
        auto feedback = std::make_shared<MoveToGoal::Feedback>();
        feedback->progress = 0.0;
        feedback->status = "Starting to move to target pose";
        goal_handle->publish_feedback(feedback);

        // 获取目标位姿
        const auto goal = goal_handle->get_goal();
        auto target_pose = goal->target_pose;
        auto current_pose = bdrobot_arm_->get_current_pose();

        // 发布中间反馈
        feedback->progress = 0.1;
        feedback->status = "Planning motion to target pose";
        goal_handle->publish_feedback(feedback);

        // 使用BDRobotArm移动到目标位姿
        bool success = bdrobot_arm_->moveP(current_pose.position.x + target_pose.position.x, 
                                         current_pose.position.y + target_pose.position.y, 
                                         current_pose.position.z + target_pose.position.z);

        // 发布进度反馈
        feedback->progress = 1.0;
        if (success) {
            feedback->status = "Successfully reached target pose";
            RCLCPP_INFO(this->get_logger(), "Successfully moved to target pose");
        } else {
            feedback->status = "Failed to reach target pose";
            RCLCPP_ERROR(this->get_logger(), "Failed to move to target pose");
        }

        // 设置结果
        auto result = std::make_shared<MoveToGoal::Result>();
        result->success = success;
        result->message = success ? "Successfully moved to target pose" : "Failed to move to target pose";

        if (rclcpp::ok()) {
            goal_handle->succeed(result);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<MoveToGoalActionServer>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    std::thread spin_thread([&]() {
        executor.spin();
    });

    action_server->initialize();  // 必须放在executor.spin()之后

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}