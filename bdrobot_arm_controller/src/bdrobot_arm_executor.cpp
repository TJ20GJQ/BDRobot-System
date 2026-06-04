#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <bdrobot_arm_controller/moveit_base.hpp>
#include <bdrobot_arm_controller/action/execute_goals.hpp>

using ExecuteGoals = bdrobot_arm_controller::action::ExecuteGoals;
using GoalHandleExecuteGoals = rclcpp_action::ServerGoalHandle<ExecuteGoals>;

class BDRobotArmExecuter : public rclcpp::Node
{
public:
    explicit BDRobotArmExecuter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("bdrobot_arm_executer", options)
    {
        using namespace std::placeholders;
        RCLCPP_INFO(this->get_logger(), "BDRobotArmExecuter created, waiting for initialization");

        this->action_server_ = rclcpp_action::create_server<ExecuteGoals>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "execute_goals",
            std::bind(&BDRobotArmExecuter::handle_goal, this, _1, _2),
            std::bind(&BDRobotArmExecuter::handle_cancel, this, _1),
            std::bind(&BDRobotArmExecuter::handle_accepted, this, _1));
    }

    void initialize()
    {
        bdrobot_arm_ = std::make_unique<bdrobot_arm_controller::BDRobotArm>(this->shared_from_this());
        bdrobot_arm_->set_MaxVelocity_Accelerate(1.0, 0.5);
        RCLCPP_INFO(this->get_logger(), "BDRobotArm initialized");
    }

private:
    rclcpp_action::Server<ExecuteGoals>::SharedPtr action_server_;
    std::unique_ptr<bdrobot_arm_controller::BDRobotArm> bdrobot_arm_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteGoals::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received goal: position=(%.2f, %.2f, %.2f)",
            goal->target.position.x, goal->target.position.y, goal->target.position.z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleExecuteGoals> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteGoals> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&BDRobotArmExecuter::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteGoals> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        const auto & target_pose = goal->target;

        auto feedback = std::make_shared<ExecuteGoals::Feedback>();
        feedback->status = "executing";
        bdrobot_arm_->get_current_pose();
        RCLCPP_INFO(this->get_logger(), "Target pose: position=(%.2f, %.2f, %.2f), orientation=(%.2f, %.2f, %.2f, %.2f)",
            target_pose.position.x, target_pose.position.y, target_pose.position.z,
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
        goal_handle->publish_feedback(feedback);

        auto result_msg = bdrobot_arm_->preaim_bounder(target_pose);
        moveit::core::MoveItErrorCode err = result_msg.error_code;

        auto result = std::make_shared<ExecuteGoals::Result>();

        if (err == moveit::core::MoveItErrorCode::SUCCESS) {
            feedback->status = "success";
            result->success = true;
            bdrobot_arm_->get_current_pose();
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            result->success = false;
            if (err == moveit::core::MoveItErrorCode::NO_IK_SOLUTION) {
                feedback->status = "no_ik_solution";
                feedback->chassis_delta_x = result_msg.delta_x;
                feedback->chassis_delta_y = result_msg.delta_y;
                feedback->chassis_delta_z = result_msg.delta_z;
                feedback->chassis_delta_yaw = result_msg.delta_yaw;
                RCLCPP_WARN(this->get_logger(),
                    "No IK solution, chassis deltas: x=%.3f y=%.3f z=%.3f yaw=%.3f",
                    result_msg.delta_x, result_msg.delta_y, result_msg.delta_z, result_msg.delta_yaw);
            } else if (err == moveit::core::MoveItErrorCode::GOAL_STATE_INVALID) {
                feedback->status = "goal_state_invalid";
                RCLCPP_ERROR(this->get_logger(), "Goal state invalid, pitch is out of range");
            } else {
                feedback->status = "failed";
                RCLCPP_ERROR(this->get_logger(), "Goal failed with error code %d", err.val);
            }
        }

        goal_handle->publish_feedback(feedback);

        if (rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 防止反馈未及时更新
            goal_handle->succeed(result);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto bdrobotarm_server = std::make_shared<BDRobotArmExecuter>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(bdrobotarm_server);
    std::thread spin_thread([&]() {
        executor.spin();
    });

    bdrobotarm_server->initialize();

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
