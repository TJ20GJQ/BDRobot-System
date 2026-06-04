#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using Eigen::Quaternionf;
using Eigen::Vector3f;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;

namespace robot_self_filter
{

struct LinkCapsule
{
    Vector3f center_offset;  // 胶囊中心在连杆坐标系中的偏移
    Vector3f axis;           // 胶囊轴线方向（单位向量）
    float length;            // 胶囊长度
    float radius;            // 胶囊半径
};

struct Capsule
{
    Vector3f start;   // 胶囊轴线起点（世界坐标）
    Vector3f end;     // 胶囊轴线终点（世界坐标）
    float r;          // 胶囊半径

    // 最小包围盒（计算一次，逐点复用的预检结构）
    Vector3f aabb_min;
    Vector3f aabb_max;
    float aabb_volume_ratio{0.0f};  // V_capsule / V_aabb，> 阈值时用 AABB 快速检测
};

class RobotSelfFilter : public rclcpp::Node
{
public:
    explicit RobotSelfFilter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ---- 配置加载 ----
    void loadFilterConfig();

    // ---- 几何工具 ----
    static float pointToSegmentDistance(const Vector3f& p, const Vector3f& a, const Vector3f& b);
    static float dist(const Vector3f& a, const Vector3f& b);
    static Capsule createCapsuleFromPose(const LinkCapsule& cfg, const Vector3f& pos, const Quaternionf& ori);
    static Capsule createSweepCapsule(const Capsule& prev, const Capsule& curr);
    static void buildCapsuleAABB(Capsule& cap);
    static float capsuleAABBVolumeRatio(const Capsule& cap);
    static bool pointInAABB(const Vector3f& pt, const Vector3f& aabb_min, const Vector3f& aabb_max);
    static bool pointInCapsule(const Vector3f& pt, const Capsule& cap);
    bool isSegmentIntersectAABB(const Vector3f& p0, const Vector3f& p1, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max) const;
    bool isCapsuleIntersectAABB(const Capsule& cap) const;

    // ---- TF 插值 ----
    static geometry_msgs::msg::TransformStamped interpolateTransform(
        const geometry_msgs::msg::TransformStamped& tf1,
        const geometry_msgs::msg::TransformStamped& tf2,
        const rclcpp::Time& query_time);

    // ---- 点云回调 ----
    void pointCloudCallback(const CustomMsg::SharedPtr msg);

    // ---- 可视化 ----
    void publishCapsuleMarkers(const rclcpp::Time& stamp);
    static geometry_msgs::msg::Quaternion quaternionFromAxis(const Vector3f& axis);

    // ---- 参数 ----
    std::string lidar_topic_;
    std::string lidar_filtered_topic_;
    std::string lidar_frame_;
    std::string config_file_;
    float lidar_hz_{10.0f};
    bool viz_enabled_{false};

    // 直通滤波
    bool pass_through_enabled_{false};
    float x_min_{0}, x_max_{0};
    float y_min_{0}, y_max_{0};
    float z_min_{0}, z_max_{0};

    // 胶囊滤波
    bool capsule_filter_enabled_{false};
    float expansion_factor_{1.0f};
    float aabb_ratio_threshold_{0.75f};  // 体积比阈值：V_cap/V_aabb > 此值时用 AABB 替代胶囊检测
    int openmp_core_num_{1};  // > 1 时使用 OpenMP 并行

    // ---- 连杆数据 ----
    std::vector<std::string> links_;
    std::unordered_map<std::string, LinkCapsule> link_properties_;
    std::unordered_map<std::string, Capsule> link_sweep_capsule_;

    rclcpp::Time prev_stamp_;
    bool init_prev_{false};

    // ---- TF & 通信 ----
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<CustomMsg>::SharedPtr sub_;
    rclcpp::Publisher<CustomMsg>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
};

}  // namespace robot_self_filter
