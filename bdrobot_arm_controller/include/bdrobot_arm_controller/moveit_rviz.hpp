#pragma once
#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Geometry>

namespace bdrobot_arm_controller
{
/**
 * @brief 机械臂RViz可视化工具类
 * 用于在RViz中显示机械臂相关的可视化信息
 */
class BDRobotRviz
{
private:
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_; ///< MoveIt可视化工具
    rclcpp::Node::SharedPtr node_; ///< ROS节点
    std::string base_frame_; ///< 基础坐标系
    moveit::core::RobotModelConstPtr robot_model_; ///< 机器人模型指针

public:
    /**
     * @brief 构造函数
     * @param node ROS节点
     * @param base_frame 基础坐标系，默认为"base_link"
     */
    BDRobotRviz(rclcpp::Node::SharedPtr node_, 
        const std::string& base_frame = "base_link",
        const moveit::core::RobotModelConstPtr& robot_model = nullptr):
        node_(node_), base_frame_(base_frame), robot_model_(robot_model)
    {
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            node_, base_frame_, rviz_visual_tools::RVIZ_MARKER_TOPIC, robot_model_);
        clear();
        visual_tools_->loadRemoteControl();
    }
    
    /**
     * @brief 析构函数
     */
    ~BDRobotRviz(){}
    
    /**
     * @brief 绘制标题文本
     * @param text 要显示的文本
     */
    void drawTitle(const std::string& text)
    {
        auto msg = Eigen::Isometry3d::Identity();
        msg.translation().z() = 1.0;  // Place text 1m above the base link
        visual_tools_->publishText(msg, text, rviz_visual_tools::WHITE,
                                  rviz_visual_tools::XLARGE);
    }
    
    /**
     * @brief 显示提示信息
     * @param text 提示文本
     */
    void prompt(const std::string& text)
    {
        visual_tools_->prompt(text);
    }
    
    /**
     * @brief 绘制轨迹路径
     * @param trajectory 机器人轨迹
     * @param joint_model_group 关节模型组
     */
    void drawTrajectoryToolPath(const moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        auto jmg = robot_model_->getJointModelGroup("arm_group");
        const moveit::core::LinkModel* tip_link = jmg->getLinkModel("bounder_link");
        visual_tools_->publishTrajectoryLine(trajectory, tip_link, jmg);
    }
    
    /**
     * @brief 触发所有可视化标记的显示
     */
    void trigger()
    {
        visual_tools_->trigger();
    }
    
    /**
     * @brief 清除所有可视化标记
     */
    void clear()
    {
        visual_tools_->deleteAllMarkers();
    }
    
    /**
     * @brief 获取MoveItVisualTools实例
     * @return MoveItVisualTools智能指针
     */
    moveit_visual_tools::MoveItVisualToolsPtr getVisualTools(){
        return visual_tools_;
    }
};
using BDRobotRvizPtr = std::shared_ptr<BDRobotRviz>;
}  // namespace bdrobot_arm_controller