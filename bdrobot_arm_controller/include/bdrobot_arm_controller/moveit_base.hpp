#pragma once
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include "moveit_rviz.hpp"

namespace bdrobot_arm_controller
{
#define LOWER_LIMIT 0
#define UPPER_LIMIT 1

struct BDRobotArmRegulatorErrorMsg
{
    moveit::core::MoveItErrorCode error_code;  // 运动结果
    double delta_x;  // x轴待移动量
    double delta_y;  // y轴待移动量
    double delta_z;  // z轴待移动量
    double delta_yaw;  // yaw轴待移动量
};

/**
 * @brief 机械臂类
 * 用于控制机械臂的运动和规划
 */
class BDRobotArm
{
private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;  // 规划
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;  // 避障
    bdrobot_arm_controller::BDRobotRvizPtr rviz_tools;  // RViz可视化工具
    std::vector<std::tuple<double, double>> joint_limits;
    std::vector<double> target_joint_positions;  // 当前目标关节角度
    bool _verbose = true;  // 是否打印详情
public:
    BDRobotArm(rclcpp::Node::SharedPtr node_): node_(node_)
    {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "arm_group");
        move_group_->setGoalPositionTolerance(0.001);  // 设置位置误差容忍度，单位：米
        move_group_->setGoalOrientationTolerance(0.00001);  // 设置姿态误差容忍度，单位：弧度
        move_group_->setGoalJointTolerance(0.001);  // 设置关节角度误差容忍度，单位：弧度
        move_group_->allowReplanning(true);  // 允许重新规划
        move_group_->setPlanningTime(10.0);  // 设置最大规划时间
        move_group_->setNumPlanningAttempts(10); 
        move_group_->setPlanningPipelineId("ompl");  // 设置规划库 ompl/pilz_industrial_motion_planner
        move_group_->setPlannerId("RRTConnectkConfigDefault");  // 设置规划器算法 RRTConnectkConfigDefault/RRT/TRRT...需要在moveit_config/config/ompl_planning.yaml中配置
        move_group_->setPoseReferenceFrame("base_link");  // 必须指定为模型根坐标系（或者不显式设置）
        move_group_->setMaxVelocityScalingFactor(0.5);  // 设置最大速度缩放因子
        move_group_->setMaxAccelerationScalingFactor(0.5);  // 设置最大加速度缩放因子
        move_group_->setEndEffectorLink("bounder_link");  // 设置末端执行器链接
        move_group_->startStateMonitor();  // 启动状态监控，必须加，不然不会根据/joint_states更新机器人状态
        get_joint_limits();
        // planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>(node_, move_group_->getRobotModel());
    }
    ~BDRobotArm()
    {
        move_group_ = nullptr;
    }
    
    /**
     * @brief 设置debug模式（默认开启）
     * @param _verbose 是否打印详情
     */
    void set_debug_mode(bool _verbose)
    {
        this->_verbose = _verbose;
    }

    /**
     * @brief 获取可视化工具实例
     * @return bdrobot_arm_controller::BDRobotRvizPtr 可视化工具实例指针，若未初始化则初始化
     */
    bdrobot_arm_controller::BDRobotRvizPtr get_rviz_tools()
    {
        if(rviz_tools == nullptr)
        {
            set_visualization(true);
        }
        return rviz_tools;
    }

    /**
     * @brief 设置可视化工具用于显示机械臂规划的路径等
     * @param visualization 是否可视化
     */
    void set_visualization(bool visualization)
    {
        if(visualization)
        {
            rviz_tools = std::make_shared<BDRobotRviz>(node_, move_group_->getPoseReferenceFrame(), move_group_->getRobotModel());
        }
        else
        {
            rviz_tools = nullptr;
        }
    }

    /**
     * @brief 获取所有关节的限位
     */
    void get_joint_limits()
    {
        // 清空之前的关节限位数据
        joint_limits.clear();
        
        // 获取机器人模型
        const moveit::core::RobotModelConstPtr& robot_model = move_group_->getRobotModel();
        const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(move_group_->getName());
        
        // 获取该组中所有关节模型
        const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
                
        // 遍历所有关节模型，获取其限位
        for(const moveit::core::JointModel* joint_model : joint_models)
        {
            const std::vector<moveit::core::VariableBounds>& variable_bounds = joint_model->getVariableBounds();
            
            // 对于每个关节变量，获取最小和最大位置
            for(size_t i = 0; i < variable_bounds.size(); ++i)
            {
                double min_pos = variable_bounds[i].min_position_;
                double max_pos = variable_bounds[i].max_position_;
                
                // 添加到关节限位向量中
                joint_limits.push_back(std::make_tuple(min_pos, max_pos));
                
                RCLCPP_INFO(node_->get_logger(), "Joint: %s, Variable %zu, Min: %f, Max: %f", 
                           joint_model->getName().c_str(), i, min_pos, max_pos);
            }
        }
    }

    /**
     * @brief 获取当前所有关节角度
     * @return std::vector<double> 当前所有关节角度
     */
    std::vector<double> get_current_joints()
    {
        std::vector<double> current_joints;
        current_joints = move_group_->getCurrentJointValues();

        if(_verbose)
        {
            RCLCPP_INFO(node_->get_logger(), "Current joint positions: [%f %f %f %f %f]\n", 
            current_joints[0], current_joints[1], current_joints[2], current_joints[3], current_joints[4]);
        }
        return current_joints;
    }
    
    /**
     * @brief 获取当前末端相对reference_frame位姿
     * @return geometry_msgs::msg::Pose 当前末端执行器位姿
     */
    geometry_msgs::msg::Pose get_current_pose()
    {
        geometry_msgs::msg::Pose current_pose;
        current_pose = move_group_->getCurrentPose().pose;  // 获取当前末端执行器的位姿

        if(_verbose)
        {
            double roll, pitch, yaw;
            geometry_msgs::msg::Quaternion q1;  // 将位姿中的四元数赋值给q1
            tf2::Quaternion quat;
            
            q1.x = current_pose.orientation.x;
            q1.y = current_pose.orientation.y;
            q1.z = current_pose.orientation.z;
            q1.w = current_pose.orientation.w;
            tf2::fromMsg(q1, quat);  // 将四元数q1转换为tf2::Quaternion类型        
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);  // 从四元数中计算roll, pitch, yaw值

            RCLCPP_INFO(node_->get_logger(), "Current pose [x y z r p y]: [%f %f %f %f %f %f]\n", 
            current_pose.position.x, current_pose.position.y, current_pose.position.z, roll, pitch, yaw);
        }
        return current_pose;
    }

    /**
     * @brief 设置末端执行器链接
     * @param end_effector_link 末端执行器链接
     */ 
    void set_end_effector_link(std::string end_effector_link)
    {
        move_group_->setEndEffectorLink(end_effector_link);
    }

    /**
     * @brief 设置最大速度和加速度
     * @param max_velocity_scaling_factor 最大速度缩放因子
     * @param max_acceleration_scaling_factor 最大加速度缩放因子
     */
    void set_MaxVelocity_Accelerate(double max_velocity_scaling_factor, double max_acceleration_scaling_factor)
    {
        move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
        move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
    }

    /**
     * @brief 设置规划器用于机械臂路径规划和轨迹优化
     * @param pipeline_id 规划库
     * @param planner_id 规划器算法
     */
    void set_planner(std::string pipeline_id, std::string planner_id)
    {
        move_group_->setPlanningPipelineId(pipeline_id);
        move_group_->setPlannerId(planner_id);
    }

     /*========================================= Robot通用 =========================================*/
    /**
     * @brief 移动到预设初始点位置
     */
    void move_Init()
    {
        move_group_->setNamedTarget("init"); 
        move_group_->move();  // 阻塞运行 plan+execute
    }

    /**
     * @brief 移动到预设最高位置
     */
    void move_Highest()
    {
        move_group_->setNamedTarget("highest"); 
        move_group_->move();
    }

    /**
     * @brief 移动到预设最远位置
     */
    void move_Farthest()
    {
        move_group_->setNamedTarget("farthest"); 
        move_group_->move();
    }

    /**
     * @brief 正运动学：移动到关节空间某一位置
     * @param joint 目标关节角度
     * @return moveit::core::MoveItErrorCode 运动结果
     */
    moveit::core::MoveItErrorCode moveJ(std::vector<double> joint)
    {
        if(_verbose)
        {
            RCLCPP_INFO(node_->get_logger(), "Trying to move to joint position: [%f %f %f %f %f]\n", 
            joint[0], joint[1], joint[2], joint[3], joint[4]);
        }
        move_group_->setJointValueTarget(joint);

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        moveit::core::MoveItErrorCode result = move_group_->plan(arm_plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            if(rviz_tools != nullptr){
                rviz_tools->drawTrajectoryToolPath(arm_plan.trajectory_);
                rviz_tools->trigger();
            }
            move_group_->execute(arm_plan);
        }
        return result;
    }
    
    /**
     * @brief 检查目标位姿是否在逆运动学解的范围内，并进行自碰撞检测（改）
     * @param target_pose 目标位姿
     * @param max_attempts 最大尝试次数
     * @return bool 是否有效
     */
    bool checkIKvalid(geometry_msgs::msg::Pose target_pose, int max_attempts = 1)  // 暂未构建出测试数据
    {
        moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();  // 获取机器人当前状态
        const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup(move_group_->getName());

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        auto ps = std::make_shared<planning_scene::PlanningScene>(move_group_->getRobotModel());  // 自碰撞检测（没有添加环境模型）

        // 碰撞检测
        bool found_ik = false;
        while (!found_ik && max_attempts-- > 0) {
            found_ik = current_state->setFromIK(joint_model_group, target_pose, 0.5);  // 求解逆运动学，默认不考虑碰撞
            RCLCPP_INFO(node_->get_logger(), "IK solution for goal %s", found_ik ? "found successfully" : "not found");
            if (found_ik) {
                collision_result.clear();
                ps->checkCollision(collision_request, collision_result, *current_state);
                if (collision_result.collision) {
                    RCLCPP_ERROR(node_->get_logger(), "IK solution for goal detected collision");
                    found_ik = false;
                } else {
                    RCLCPP_INFO(node_->get_logger(), "IK solution for goal is valid");
                }
            }
        }
        return found_ik;
    }
    
    /**
     * @brief 逆运动学：移动到位姿空间某一位置
     * @param target_pose 目标位姿
     * @return moveit::core::MoveItErrorCode 运动结果
     */
    moveit::core::MoveItErrorCode moveP(geometry_msgs::msg::Pose target_pose)
    {
        return moveP(target_pose.position.x, 
                    target_pose.position.y, 
                    target_pose.position.z, 
                    target_pose.orientation.x, 
                    target_pose.orientation.y, 
                    target_pose.orientation.z, 
                    target_pose.orientation.w);
    }

    /**
     * @brief 逆运动学：移动到位姿空间某一位置
     * @param x 目标位置x坐标
     * @param y 目标位置y坐标
     * @param z 目标位置z坐标
     * @return moveit::core::MoveItErrorCode 运动结果
     */
    moveit::core::MoveItErrorCode moveP(double x, double y, double z)
    {
        geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
        
        return moveP(x, 
                    y, 
                    z, 
                    current_pose.orientation.x, 
                    current_pose.orientation.y, 
                    current_pose.orientation.z, 
                    current_pose.orientation.w);
    }

    /**
     * @brief 逆运动学：移动到位姿空间某一位置
     * @param x 目标位置x坐标
     * @param y 目标位置y坐标
     * @param z 目标位置z坐标
     * @param rx 目标姿态x轴欧拉角（度）
     * @param ry 目标姿态y轴欧拉角（度）
     * @param rz 目标姿态z轴欧拉角（度）
     * @return moveit::core::MoveItErrorCode 运动结果
     */
    moveit::core::MoveItErrorCode moveP(double x, double y, double z, double rx, double ry, double rz)
    {
        rx = rx/57.29578;
        ry = ry/57.29578;
        rz = rz/57.29578;
        
        tf2::Quaternion qx,qy,qz;
        qx.setRotation(tf2::Vector3(1,0,0), rx);
        qy.setRotation(tf2::Vector3(0,1,0), ry);
        qz.setRotation(tf2::Vector3(0,0,1), rz);
        tf2::Quaternion q_final = qz*qy*qx;  // 旋转顺序zyx

        return moveP(x, y, z, q_final.x(), q_final.y(), q_final.z(), q_final.w());
    }

    /**
     * @brief 逆运动学：移动到位姿空间某一位置
     * @param x 目标位置x坐标
     * @param y 目标位置y坐标
     * @param z 目标位置z坐标
     * @param qx 目标姿态x轴四元数
     * @param qy 目标姿态y轴四元数
     * @param qz 目标姿态z轴四元数
     * @param qw 目标姿态w轴四元数
     * @return moveit::core::MoveItErrorCode 运动结果
     */
    moveit::core::MoveItErrorCode moveP(double x, double y, double z, double qx, double qy, double qz, double qw)
    {
        tf2::Quaternion quat;
        geometry_msgs::msg::Pose target_pose;

        quat = tf2::Quaternion(qx, qy, qz, qw);  // 设置目标姿态
        quat.normalize();
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = quat.getX();
        target_pose.orientation.y = quat.getY();
        target_pose.orientation.z = quat.getZ();
        target_pose.orientation.w = quat.getW();

        if(_verbose)
        {
            RCLCPP_INFO(node_->get_logger(), "Trying to move to pose: [%f %f %f, %f %f %f %f]\n", 
                        x, y, z, qx, qy, qz, qw);
        }
        
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(target_pose);
        RCLCPP_INFO(node_->get_logger(), "target_pose: %f, %f, %f, %f, %f, %f, %f", target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
        
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        moveit::core::MoveItErrorCode result = move_group_->plan(arm_plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            if(rviz_tools != nullptr){
                rviz_tools->drawTrajectoryToolPath(arm_plan.trajectory_);
                rviz_tools->trigger();
            }
            move_group_->execute(arm_plan);
        }
        return result;
    }

    /*========================================= BDRobot专用 =========================================*/
    /**
     * @brief 逆运动学：移动到base_link位姿空间某一位置（四自由度），检查关节范围，不检查碰撞
     * @param x 目标位置x坐标
     * @param y 目标位置y坐标
     * @param z 目标位置z坐标
     * @param yaw 目标姿态yaw轴(rad)
     * @return std::tuple<moveit::core::MoveItErrorCode, std::vector<double>> 运动结果与5个关节的IK解
     */
    std::tuple<moveit::core::MoveItErrorCode, std::vector<double>> computeBDRobotArmIK(double x, double y, double z, double yaw)
    {
        moveit::core::MoveItErrorCode err_code = moveit::core::MoveItErrorCode::SUCCESS;

        std::vector<double> joint_positions;
        moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();  // 获取机器人当前状态
        current_state->copyJointGroupPositions(
            current_state->getRobotModel()->getJointModelGroup(move_group_->getName()),
            joint_positions
        );

        geometry_msgs::msg::Point current_point = move_group_->getCurrentPose().pose.position;

        // 计算关节位置
        // YAW
        if(yaw < std::get<LOWER_LIMIT>(joint_limits[4]) || yaw > std::get<UPPER_LIMIT>(joint_limits[4])){
            err_code = moveit::core::MoveItErrorCode::NO_IK_SOLUTION;
        }
        double delta_yaw = yaw + M_PI - joint_positions[4];
        joint_positions[4] = yaw + M_PI;  // base坐标系与关节坐标系差180度
        // 转换为[-M_PI, M_PI]范围
        if(delta_yaw > M_PI){
            delta_yaw -= 2*M_PI;
        }
        else if(delta_yaw < -M_PI){
            delta_yaw += 2*M_PI;
        }
        if(joint_positions[4] > M_PI){
            joint_positions[4] -= 2*M_PI;
        }
        else if(joint_positions[4] < -M_PI){
            joint_positions[4] += 2*M_PI;
        }
 
        // Z
        double delta_z = z - current_point.z;
        if(joint_positions[0] + joint_positions[1] + delta_z > std::get<UPPER_LIMIT>(joint_limits[0]) + std::get<UPPER_LIMIT>(joint_limits[1]) ||
           joint_positions[0] + joint_positions[1] + delta_z < std::get<LOWER_LIMIT>(joint_limits[0]) + std::get<LOWER_LIMIT>(joint_limits[1])){
            err_code = moveit::core::MoveItErrorCode::NO_IK_SOLUTION;
        }

        if(delta_z >= 0){  // 上升优先运行关节2
            if(delta_z + joint_positions[1] >= std::get<UPPER_LIMIT>(joint_limits[1])){
                joint_positions[0] += delta_z - std::get<UPPER_LIMIT>(joint_limits[1]) + joint_positions[1];
                joint_positions[1] = std::get<UPPER_LIMIT>(joint_limits[1]);
            }
            else{
                joint_positions[0] = joint_positions[0];
                joint_positions[1] += delta_z;
            }
        }
        else{  // 下降优先运行关节1
            if(delta_z + joint_positions[0] <= std::get<LOWER_LIMIT>(joint_limits[0])){
                joint_positions[1] += delta_z - std::get<LOWER_LIMIT>(joint_limits[0]) + joint_positions[0];
                joint_positions[0] = std::get<LOWER_LIMIT>(joint_limits[0]);
            }
            else{
                joint_positions[0] += delta_z;
                joint_positions[1] = joint_positions[1];
            }
        }
 
        // X、Y
        // 获取TF，以base_link为参考坐标系
        const Eigen::Isometry3d& T_base_bounder = current_state->getGlobalLinkTransform("bounder_link");
        const Eigen::Isometry3d& T_base_arm5 = current_state->getGlobalLinkTransform("arm5_link");

        // 获取bounder旋转后相对base_link的x/y分量
        static const Eigen::Isometry3d T_arm5_bounder = T_base_arm5.inverse() * T_base_bounder;  // arm5_link到bounder的变换（使用static只计算一次，避免重复计算）
        Eigen::Isometry3d Rz = Eigen::Isometry3d::Identity();
        Rz.rotate(Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Isometry3d new_T_base_bounder = T_base_arm5 * Rz * T_arm5_bounder;

        double delta_x = - (x - new_T_base_bounder.translation()[0]);
        double delta_y = - (y - new_T_base_bounder.translation()[1]);
        if(joint_positions[3] + delta_x > std::get<UPPER_LIMIT>(joint_limits[3]) || joint_positions[3] + delta_x < std::get<LOWER_LIMIT>(joint_limits[3])){
            err_code = moveit::core::MoveItErrorCode::NO_IK_SOLUTION;
        }
        joint_positions[3] += delta_x;
        if(joint_positions[2] + delta_y > std::get<UPPER_LIMIT>(joint_limits[2]) || joint_positions[2] + delta_y < std::get<LOWER_LIMIT>(joint_limits[2])){
            err_code = moveit::core::MoveItErrorCode::NO_IK_SOLUTION;
        }
        joint_positions[2] += delta_y;

        return {err_code, joint_positions};
    }


    /**
     * @brief 移动到指定点，使末端与法向量平行且朝向该点（解析解）
     * @param target_pose 目标位姿，目标点在base_link坐标系下的位置和姿态
     * @param strike_distance 预瞄距离，默认0.15米
     * @return BDRobotArmRegulatorErrorMsg 运动结果及底盘微调量
     */
    BDRobotArmRegulatorErrorMsg preaim_bounder(geometry_msgs::msg::Pose target_pose, double strike_distance = 0.15)
    {
        BDRobotArmRegulatorErrorMsg err_msg;
        
        // ====================== 从 target_pose 中提取 X/Y/Z/YAW ======================
        tf2::Quaternion q;
        tf2::fromMsg(target_pose.orientation, q);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if (pitch > 3 * M_PI / 180 || pitch < -3 * M_PI / 180){
            RCLCPP_WARN(node_->get_logger(), "Pitch is %f degree, which is out of range. Please align the bounder to horizontal manually.", pitch * 180 / M_PI);
            err_msg.error_code = moveit::core::MoveItErrorCode::GOAL_STATE_INVALID;
            return err_msg;
        }

        std::vector<double> joint_positions;
        moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();  // 获取机器人当前状态
        current_state->copyJointGroupPositions(
            current_state->getRobotModel()->getJointModelGroup(move_group_->getName()),
            joint_positions
        );

        auto [bound_err_code, bound_joint_positions] = computeBDRobotArmIK(target_pose.position.x, target_pose.position.y, target_pose.position.z, yaw);
        if(bound_err_code != moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_WARN(node_->get_logger(), "Bounding is not available now. Please follow the instructions below to adjust.");
            err_msg.error_code = bound_err_code;

            if(bound_joint_positions[0] < std::get<LOWER_LIMIT>(joint_limits[0]) || bound_joint_positions[0] > std::get<UPPER_LIMIT>(joint_limits[0])){
                err_msg.delta_z = bound_joint_positions[0] - (bound_joint_positions[0] > std::get<UPPER_LIMIT>(joint_limits[0]) ? std::get<UPPER_LIMIT>(joint_limits[0]) : std::get<LOWER_LIMIT>(joint_limits[0]));
                RCLCPP_WARN(node_->get_logger(), "Arm1_joint is now at %f, which should be %f for bounding, while the joint is limited to [%f, %f]. "
                "Please move chassis at least %fm along z axis of base_link first. ",
                joint_positions[0], bound_joint_positions[0], std::get<LOWER_LIMIT>(joint_limits[0]), std::get<UPPER_LIMIT>(joint_limits[0]),
                err_msg.delta_z);
            }
            else if(bound_joint_positions[1] < std::get<LOWER_LIMIT>(joint_limits[1]) || bound_joint_positions[1] > std::get<UPPER_LIMIT>(joint_limits[1])){
                err_msg.delta_z = bound_joint_positions[1] - (bound_joint_positions[1] > std::get<UPPER_LIMIT>(joint_limits[1]) ? std::get<UPPER_LIMIT>(joint_limits[1]) : std::get<LOWER_LIMIT>(joint_limits[1]));
                RCLCPP_WARN(node_->get_logger(), "Arm2_joint is now at %f, which should be %f for bounding, while the joint is limited to [%f, %f]. "
                "Please move chassis at least %fm along z axis of base_link first. ",
                joint_positions[1], bound_joint_positions[1], std::get<LOWER_LIMIT>(joint_limits[1]), std::get<UPPER_LIMIT>(joint_limits[1]),
                err_msg.delta_z);
            }

            if(bound_joint_positions[2] < std::get<LOWER_LIMIT>(joint_limits[2]) || bound_joint_positions[2] > std::get<UPPER_LIMIT>(joint_limits[2])){
                err_msg.delta_y = - (bound_joint_positions[2] - (bound_joint_positions[2] > std::get<UPPER_LIMIT>(joint_limits[2]) ? std::get<UPPER_LIMIT>(joint_limits[2]) : std::get<LOWER_LIMIT>(joint_limits[2])));
                RCLCPP_WARN(node_->get_logger(), "Arm3_joint is now at %f, which should be %f for bounding, while the joint is limited to [%f, %f]. "
                "Please move chassis at least %fm along y axis of base_link first. ",
                joint_positions[2], bound_joint_positions[2], std::get<LOWER_LIMIT>(joint_limits[2]), std::get<UPPER_LIMIT>(joint_limits[2]),
                err_msg.delta_y);
            }

            if(bound_joint_positions[3] < std::get<LOWER_LIMIT>(joint_limits[3]) || bound_joint_positions[3] > std::get<UPPER_LIMIT>(joint_limits[3])){
                err_msg.delta_x = - (bound_joint_positions[3] - (bound_joint_positions[3] > std::get<UPPER_LIMIT>(joint_limits[3]) ? std::get<UPPER_LIMIT>(joint_limits[3]) : std::get<LOWER_LIMIT>(joint_limits[3])));
                RCLCPP_WARN(node_->get_logger(), "Arm4_joint is now at %f, which should be %f for bounding, while the joint is limited to [%f, %f]. "
                "Please move chassis at least %fm along x axis of base_link first. ",
                joint_positions[3], bound_joint_positions[3], std::get<LOWER_LIMIT>(joint_limits[3]), std::get<UPPER_LIMIT>(joint_limits[3]),
                err_msg.delta_x);
            }

            if(bound_joint_positions[4] < std::get<LOWER_LIMIT>(joint_limits[4]) || bound_joint_positions[4] > std::get<UPPER_LIMIT>(joint_limits[4])){
                err_msg.delta_yaw = bound_joint_positions[4] - (bound_joint_positions[4] > std::get<UPPER_LIMIT>(joint_limits[4]) ? std::get<UPPER_LIMIT>(joint_limits[4]) : std::get<LOWER_LIMIT>(joint_limits[4]));
                RCLCPP_WARN(node_->get_logger(), "Arm5_joint is now at %f, which should be %f for bounding, while the joint is limited to [%f, %f]. "
                "Please rotate chassis at least %f degree around z axis of base_link first. ",
                joint_positions[4], bound_joint_positions[4], std::get<LOWER_LIMIT>(joint_limits[4]), std::get<UPPER_LIMIT>(joint_limits[4]),
                err_msg.delta_yaw);
            }

            return err_msg;
        }

        // ====================== 计算预留一定距离后的X/Y关节，为弹击杆和垂直校正留出距离 ======================
        Eigen::Isometry3d target_pose_preaim = Eigen::Isometry3d::Identity();
        target_pose_preaim.translation() = Eigen::Vector3d(
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        );
        Eigen::Quaterniond orientation(
            target_pose.orientation.w,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z
        );
        target_pose_preaim.linear() = orientation.toRotationMatrix();
        // 沿末端执行器X轴负方向预留一定距离
        Eigen::Vector3d local_negative_x_axis = target_pose_preaim.linear() * Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d displacement = strike_distance * local_negative_x_axis;
        target_pose_preaim.translation() -= displacement;

        auto [preaim_err_code, preaim_joint_positions] = computeBDRobotArmIK(target_pose_preaim.translation().x(), target_pose_preaim.translation().y(), target_pose_preaim.translation().z(), yaw);
        if(preaim_err_code != moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_WARN(node_->get_logger(), "Preaiming is not available now. Please follow the instructions below to adjust.");
            err_msg.error_code = preaim_err_code;

            if(preaim_joint_positions[0] < std::get<LOWER_LIMIT>(joint_limits[0]) || preaim_joint_positions[0] > std::get<UPPER_LIMIT>(joint_limits[0])){
                err_msg.delta_z = preaim_joint_positions[0] - (preaim_joint_positions[0] > std::get<UPPER_LIMIT>(joint_limits[0]) ? std::get<UPPER_LIMIT>(joint_limits[0]) : std::get<LOWER_LIMIT>(joint_limits[0]));
                RCLCPP_WARN(node_->get_logger(), "Arm1_joint is now at %f, which should be %f for preaiming, while the joint is limited to [%f, %f]. "
                "Please move chassis at least %fm along z axis of base_link first. ",
                joint_positions[0], preaim_joint_positions[0], std::get<LOWER_LIMIT>(joint_limits[0]), std::get<UPPER_LIMIT>(joint_limits[0]),
                err_msg.delta_z);
            }
            else if(preaim_joint_positions[1] < std::get<LOWER_LIMIT>(joint_limits[1]) || preaim_joint_positions[1] > std::get<UPPER_LIMIT>(joint_limits[1])){
                err_msg.delta_z = preaim_joint_positions[1] - (preaim_joint_positions[1] > std::get<UPPER_LIMIT>(joint_limits[1]) ? std::get<UPPER_LIMIT>(joint_limits[1]) : std::get<LOWER_LIMIT>(joint_limits[1]));
                RCLCPP_WARN(node_->get_logger(), "Arm2_joint is now at %f, which should be %f for preaiming, while the joint is limited to [%f, %f]. "
                "Please move chassis at least %fm along z axis of base_link first. ",
                joint_positions[1], preaim_joint_positions[1], std::get<LOWER_LIMIT>(joint_limits[1]), std::get<UPPER_LIMIT>(joint_limits[1]),
                err_msg.delta_z);
            }

            if(preaim_joint_positions[2] < std::get<LOWER_LIMIT>(joint_limits[2]) || preaim_joint_positions[2] > std::get<UPPER_LIMIT>(joint_limits[2])){
                err_msg.delta_y = - (preaim_joint_positions[2] - (preaim_joint_positions[2] > std::get<UPPER_LIMIT>(joint_limits[2]) ? std::get<UPPER_LIMIT>(joint_limits[2]) : std::get<LOWER_LIMIT>(joint_limits[2])));
                RCLCPP_WARN(node_->get_logger(), "Arm3_joint is now at %f, which should be %f for preaiming, while the joint is limited to [%f, %f]. "
                "Please move chassis at least %fm along y axis of base_link first. ",
                joint_positions[2], preaim_joint_positions[2], std::get<LOWER_LIMIT>(joint_limits[2]), std::get<UPPER_LIMIT>(joint_limits[2]),
                err_msg.delta_y);
            }

            if(preaim_joint_positions[3] < std::get<LOWER_LIMIT>(joint_limits[3]) || preaim_joint_positions[3] > std::get<UPPER_LIMIT>(joint_limits[3])){
                err_msg.delta_x = - (preaim_joint_positions[3] - (preaim_joint_positions[3] > std::get<UPPER_LIMIT>(joint_limits[3]) ? std::get<UPPER_LIMIT>(joint_limits[3]) : std::get<LOWER_LIMIT>(joint_limits[3])));
                RCLCPP_WARN(node_->get_logger(), "Arm4_joint is now at %f, which should be %f for preaiming, while the joint is limited to [%f, %f]. "
                "Please move chassis at least %fm along x axis of base_link first. ",
                joint_positions[3], preaim_joint_positions[3], std::get<LOWER_LIMIT>(joint_limits[3]), std::get<UPPER_LIMIT>(joint_limits[3]),
                err_msg.delta_x);
            }

            if(preaim_joint_positions[4] < std::get<LOWER_LIMIT>(joint_limits[4]) || preaim_joint_positions[4] > std::get<UPPER_LIMIT>(joint_limits[4])){
                err_msg.delta_yaw = preaim_joint_positions[4] - (preaim_joint_positions[4] > std::get<UPPER_LIMIT>(joint_limits[4]) ? std::get<UPPER_LIMIT>(joint_limits[4]) : std::get<LOWER_LIMIT>(joint_limits[4]));
                RCLCPP_WARN(node_->get_logger(), "Arm5_joint is now at %f, which should be %f for preaiming, while the joint is limited to [%f, %f]. "
                "Please rotate chassis at least %f degree around z axis of base_link first. ",
                joint_positions[4], preaim_joint_positions[4], std::get<LOWER_LIMIT>(joint_limits[4]), std::get<UPPER_LIMIT>(joint_limits[4]),
                err_msg.delta_yaw);
            }

            return err_msg;
        }
        
        if(_verbose){
            RCLCPP_INFO(node_->get_logger(), "Computed preaiming joint positions: %.2f, %.2f, %.2f, %.2f, %.2f",
            preaim_joint_positions[0], preaim_joint_positions[1], preaim_joint_positions[2], preaim_joint_positions[3], preaim_joint_positions[4]);
        }
        
        // ====================== 发送关节目标并运动 ======================
        err_msg.error_code = moveJ(preaim_joint_positions);
        return err_msg;
    }
    
    /**
     * @brief 按压至目标点
     * @param target_pose 目标位姿，目标点在base_link坐标系下的位置和姿态
     * @return moveit::core::MoveItErrorCode 运动结果
     */
    moveit::core::MoveItErrorCode press_bounder(geometry_msgs::msg::Pose target_pose)
    {
        return moveit::core::MoveItErrorCode::SUCCESS;
    }

    // bool move_C(std::string axis, float radius)
    // {
    //     geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose(end_effector_link).pose;
    //     std::vector<geometry_msgs::msg::Pose> waypoints;
    //     waypoints.push_back(start_pose);
    //     if(axis=="xy" || axis=="yx")  // 在x,y平面内生成圆路点位置,姿态保持与起始点一致
    //     {
    //         float a = start_pose.position.x;
    //         float b = start_pose.position.y;
    //         for(float th = 0.0; th < 6.28; th = th + 0.01)
    //         {
    //             start_pose.position.x = a + (radius - radius * cos(th));
    //             start_pose.position.y = b + radius * sin(th);
    //             waypoints.push_back(start_pose);
    //         }
    //     }else if(axis=="yz" || axis=="zy")  // 在y,z平面内生成圆路点位置,姿态保持与起始点一致
    //     {
    //         float a = start_pose.position.y;
    //         float b = start_pose.position.z;
    //         for(float th = 0.0; th < 6.28; th = th + 0.01)
    //         {
    //             start_pose.position.z = b + (radius - radius * cos(th));
    //             start_pose.position.y = a + radius * sin(th);
    //             waypoints.push_back(start_pose);
    //         }
    //     }else if(axis=="xz" || axis=="zx")  // 在x,z平面内生成圆路点位置,姿态保持与起始点一致
    //     {
    //         float a = start_pose.position.x;
    //         float b = start_pose.position.z;
    //         for(float th = 0.0; th < 6.28; th = th + 0.01)
    //         {
    //             start_pose.position.z = b + radius * sin(th);
    //             start_pose.position.x = a + (radius - radius * cos(th));
    //             waypoints.push_back(start_pose);
    //         }
    //     }
    //     moveit_msgs::msg::RobotTrajectory trajectory;
    //     const double jump_threshold = 0.0;
    //     const double eef_step = 0.01;
    //     double fraction = 0.0;
    //     int maxtries = 100;   
    //     int attempts = 0;     
    
    //     while (fraction < 1.0 && attempts < maxtries)
    //     {
    //         fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //         attempts++;
    //     }
    
    //     if(fraction == 1)  // 规划成功率0-1
    //     {   
    //         printf("Path computed successfully. Moving the arm.");

    //         // 运动规划数据
    //         arm_plan.trajectory_ = trajectory;
    //         // 执行
    //         move_group->execute(arm_plan);
    //         sleep(1);
    //         return true;
    //     }
    //     else
    //     {
    //         printf("Move_C Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    //         return false;
    //     }
    // }

    /**
     * @brief 笛卡尔路径规划，保持末端直线运动且姿态不变
     * @param poses 目标位姿序列
     */
    void moveCartesian(std::vector<geometry_msgs::msg::Pose> &poses)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;
        waypoints.push_back(start_pose);
        for(uint8_t i = 0; i < poses.size(); i++)
        {
            poses[i].orientation = start_pose.orientation;
            waypoints.push_back(poses[i]);
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = 0.0;
        int maxtries = 1000;  // 最大尝试规划次数
        int attempts = 0;  // 已经尝试规划次数

        while(fraction < 1.0 && attempts < maxtries)
        {
            fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;
        }

        RCLCPP_ERROR(node_->get_logger(), "Path planning with %0.6f success after %d attempts.", fraction, maxtries);
        arm_plan.trajectory_ = trajectory;  // 运动规划数据
        move_group_->execute(arm_plan);  // 执行
    }
};
}  // namespace bdrobot_arm_controller