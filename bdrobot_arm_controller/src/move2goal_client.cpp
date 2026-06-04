#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <bdrobot_arm_controller/action/move_to_goal.hpp>

using MoveToGoal = bdrobot_arm_controller::action::MoveToGoal;

class MoveToGoalActionClient : public rclcpp::Node
{
public:
    using GoalHandleMoveToGoal = rclcpp_action::ClientGoalHandle<MoveToGoal>;

    explicit MoveToGoalActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("move_to_goal_action_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<MoveToGoal>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "move_to_goal");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MoveToGoalActionClient::send_goal, this));
    }

private:
    rclcpp_action::Client<MoveToGoal>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Logger logger_ = rclcpp::get_logger("MoveToGoalActionClient");
    bool goal_sent_ = false;

    void send_goal()
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(logger_, "Action server not available after waiting");
            return;
        }

        if (goal_sent_) {
            this->timer_->cancel();
            return;
        }

        auto goal_msg = MoveToGoal::Goal();
        // 设置目标位姿（示例值，请根据需要调整）
        goal_msg.target_pose.position.x = 0.0;
        goal_msg.target_pose.position.y = 0.0;
        goal_msg.target_pose.position.z = 0.5;
        goal_msg.target_pose.orientation.x = 0.0;
        goal_msg.target_pose.orientation.y = 0.0;
        goal_msg.target_pose.orientation.z = 0.0;
        goal_msg.target_pose.orientation.w = 1.0;

        RCLCPP_INFO(logger_, "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveToGoal>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveToGoalActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MoveToGoalActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MoveToGoalActionClient::result_callback, this, _1);

        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;
    }

    void goal_response_callback(GoalHandleMoveToGoal::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(logger_, "Goal was rejected by server");
        } else {
            RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleMoveToGoal::SharedPtr,
        const std::shared_ptr<const MoveToGoal::Feedback> feedback)
    {
        RCLCPP_INFO(logger_,
            "Feedback received: Progress=%.2f, Status=%s",
            feedback->progress, feedback->status.c_str());
    }

    void result_callback(const GoalHandleMoveToGoal::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(logger_, "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger_, "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(logger_, "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(logger_, "Unknown result code");
            return;
        }

        RCLCPP_INFO(logger_, "Result received: Success=%s, Message=%s",
            result.result->success ? "true" : "false",
            result.result->message.c_str());

        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<MoveToGoalActionClient>();

    RCLCPP_INFO(action_client->get_logger(), "Waiting for action server...");
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}