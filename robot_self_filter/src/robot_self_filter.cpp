#include "robot_self_filter/robot_self_filter.hpp"
#include <omp.h>
#include <chrono>

namespace robot_self_filter
{

// ==============================================================================
// 构造
// ==============================================================================

RobotSelfFilter::RobotSelfFilter(const rclcpp::NodeOptions& options)
    : Node("robot_self_filter", options)
{
    // 声明参数并获取默认值（支持命令行覆盖）
    this->declare_parameter("config_file", "");
    config_file_ = this->get_parameter("config_file").as_string();
    if (config_file_.empty()) {
        // 默认路径：相对于功能包 share 目录
        config_file_ = "/home/yuyouling/bdrobot_ws/src/robot_self_filter/config/self_filter_config.yaml";
    }

    loadFilterConfig();

    for (const auto& pair : link_properties_) {
        links_.push_back(pair.first);
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        this->get_clock(),
        tf2::durationFromSec(1.0f / lidar_hz_ + 0.5));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<CustomMsg>(
        lidar_topic_, 1,
        std::bind(&RobotSelfFilter::pointCloudCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<CustomMsg>(lidar_filtered_topic_, 1);
    if(viz_enabled_)
        viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/capsules", 1);

    RCLCPP_INFO(this->get_logger(), "RobotSelfFilter node started, OpenMP cores: %d",
                openmp_core_num_);
    prev_stamp_ = this->get_clock()->now();
}

// ==============================================================================
// 配置加载
// ==============================================================================

void RobotSelfFilter::loadFilterConfig()
{
    YAML::Node config = YAML::LoadFile(config_file_);

    auto read_optional = [&](const char* key, auto& target) {
        if (config[key]) {
            target = config[key].as<std::decay_t<decltype(target)>>();
        }
    };

    read_optional("lidar_topic",            lidar_topic_);
    read_optional("lidar_filtered_topic",    lidar_filtered_topic_);
    read_optional("lidar_frame",            lidar_frame_);
    read_optional("lidar_hz",               lidar_hz_);
    read_optional("viz_enabled",            viz_enabled_);
    read_optional("pass_through_enabled",   pass_through_enabled_);
    read_optional("x_min", x_min_);  read_optional("x_max", x_max_);
    read_optional("y_min", y_min_);  read_optional("y_max", y_max_);
    read_optional("z_min", z_min_);  read_optional("z_max", z_max_);
    read_optional("capsule_filter_enabled", capsule_filter_enabled_);
    read_optional("expansion_factor", expansion_factor_);
    read_optional("aabb_ratio_threshold", aabb_ratio_threshold_);
    read_optional("openmp_core_num", openmp_core_num_);
    if (openmp_core_num_ > 1) {
        omp_set_num_threads(openmp_core_num_);
    }

    if (config["link_properties"]) {
        for (YAML::const_iterator it = config["link_properties"].begin();
             it != config["link_properties"].end(); ++it) {
            std::string link_name = it->first.as<std::string>();
            YAML::Node lc = it->second;

            LinkCapsule prop;
            prop.radius = lc["radius"].as<float>();
            prop.length = lc["length"].as<float>();
            prop.center_offset = Vector3f(
                lc["center_offset_x"].as<float>(),
                lc["center_offset_y"].as<float>(),
                lc["center_offset_z"].as<float>());
            prop.axis = Vector3f(
                lc["axis_x"].as<float>(),
                lc["axis_y"].as<float>(),
                lc["axis_z"].as<float>()).normalized();

            link_properties_[link_name] = prop;
        }
    }
}

// ==============================================================================
// 几何工具函数
// ==============================================================================

float RobotSelfFilter::pointToSegmentDistance(const Vector3f& p, const Vector3f& a, const Vector3f& b)
{
    Vector3f ab = b - a;
    Vector3f ap = p - a;
    float t = ap.dot(ab) / ab.squaredNorm();
    if (t < 0.0f || t > 1.0f) {
        return INFINITY;
    }
    return (p - (a + t * ab)).norm();
}

float RobotSelfFilter::dist(const Vector3f& a, const Vector3f& b)
{
    return (a - b).norm();
}

Capsule RobotSelfFilter::createCapsuleFromPose(
    const LinkCapsule& cfg, const Vector3f& pos, const Quaternionf& ori)
{
    Capsule cap;
    Vector3f axis_local = cfg.axis.normalized();
    Vector3f center_local = cfg.center_offset;

    Vector3f center_world = pos + ori * center_local;
    Vector3f axis_world = ori * axis_local;
    Vector3f half = axis_world * (cfg.length * 0.5f);

    cap.start = center_world - half;
    cap.end   = center_world + half;
    cap.r     = cfg.radius;
    return cap;
}

Capsule RobotSelfFilter::createSweepCapsule(const Capsule& prev, const Capsule& curr)
{
    std::vector<Vector3f> pts = {prev.start, prev.end, curr.start, curr.end};

    // 找最远两点作为大包轴线
    int best_i = 0, best_j = 1;
    float max_dist = 0.0f;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = i + 1; j < 4; ++j) {
            float d = dist(pts[i], pts[j]);
            if (d > max_dist) {
                max_dist = d;
                best_i = i;
                best_j = j;
            }
        }
    }

    Capsule sweep;
    sweep.start = pts[best_i];
    sweep.end   = pts[best_j];

    // 计算其他点对轴线的最大偏移，放大半径
    float max_offset = 0.0f;
    for (const auto& p : pts) {
        float d = pointToSegmentDistance(p, sweep.start, sweep.end);
        max_offset = std::max(max_offset, d);
    }

    sweep.r = prev.r + max_offset;
    return sweep;
}

bool RobotSelfFilter::pointInCapsule(const Vector3f& pt, const Capsule& cap)
{
    Vector3f ab = cap.end - cap.start;
    Vector3f ap = pt - cap.start;
    float t = ap.dot(ab) / ab.squaredNorm();
    if (t < 0.0f || t > 1.0f) return false;
    return (pt - (cap.start + t * ab)).squaredNorm() < cap.r * cap.r;
}

// ==============================================================================
// 胶囊包围盒
// ==============================================================================

void RobotSelfFilter::buildCapsuleAABB(Capsule& cap)
{
    Vector3f axis = cap.end - cap.start;
    float len = axis.norm();
    
    // 退化情况：胶囊体退化为球体（起点终点重合）
    if (len < 1e-6f) {
        cap.aabb_min = cap.start.array() - cap.r;
        cap.aabb_max = cap.start.array() + cap.r;
        return;
    }
    
    // 轴线单位向量
    Vector3f axis_unit = axis / len;
    
    // 在垂直于轴线的方向上扩展半径
    // 对于每个坐标轴i，扩展量 = r * sqrt(1 - axis_unit[i]^2)
    Vector3f ext;
    for (int i = 0; i < 3; ++i) {
        ext[i] = cap.r * std::sqrt(1.0f - axis_unit[i] * axis_unit[i]);
    }
    
    // 计算 AABB
    Vector3f segment_min = cap.start.cwiseMin(cap.end);
    Vector3f segment_max = cap.start.cwiseMax(cap.end);
    
    cap.aabb_min = segment_min - ext;
    cap.aabb_max = segment_max + ext;
}

float RobotSelfFilter::capsuleAABBVolumeRatio(const Capsule& cap)
{
    float len = (cap.end - cap.start).norm();
    float v_cap = EIGEN_PI * cap.r * cap.r * len;
    Vector3f extent = cap.aabb_max - cap.aabb_min;
    float v_aabb = extent.x() * extent.y() * extent.z();
    if (v_aabb < 1e-9f) return 0.0f;
    return v_cap / v_aabb;
}

bool RobotSelfFilter::pointInAABB(const Vector3f& pt, const Vector3f& aabb_min, const Vector3f& aabb_max)
{
    return !(pt.x() < aabb_min.x() || pt.x() > aabb_max.x() ||
           pt.y() < aabb_min.y() || pt.y() > aabb_max.y() ||
           pt.z() < aabb_min.z() || pt.z() > aabb_max.z());
}

// ==============================================================================
// AABB 线段相交检测
// ==============================================================================

bool RobotSelfFilter::isSegmentIntersectAABB(const Vector3f& p0,
                                            const Vector3f& p1,
                                            float x_min, float x_max,
                                            float y_min, float y_max,
                                            float z_min, float z_max) const
{
    auto inside = [this, x_min, x_max, y_min, y_max, z_min, z_max](const Vector3f& p) {
        return p.x() >= x_min && p.x() <= x_max &&
               p.y() >= y_min && p.y() <= y_max &&
               p.z() >= z_min && p.z() <= z_max;
    };

    if (inside(p0) || inside(p1)) {
        return true;
    }

    Vector3f dir = p1 - p0;
    float tmin = 0.0f, tmax = 1.0f;

    auto slab_test = [&](float p0_i, float dir_i, float lo, float hi) -> bool {
        if (std::fabs(dir_i) < 1e-9f) {
            return p0_i >= lo && p0_i <= hi;
        }
        float t1 = (lo - p0_i) / dir_i;
        float t2 = (hi - p0_i) / dir_i;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        return tmin <= tmax;
    };

    if (!slab_test(p0.x(), dir.x(), x_min, x_max)) return false;
    if (!slab_test(p0.y(), dir.y(), y_min, y_max)) return false;
    if (!slab_test(p0.z(), dir.z(), z_min, z_max)) return false;

    return true;
}

bool RobotSelfFilter::isCapsuleIntersectAABB(const Capsule& cap) const
{
    // 等价于：将 AABB 向外膨胀 r，检测线段是否与膨胀 AABB 相交
    return isSegmentIntersectAABB(
        cap.start, cap.end,
        x_min_ - cap.r, x_max_ + cap.r,
        y_min_ - cap.r, y_max_ + cap.r,
        z_min_ - cap.r, z_max_ + cap.r);
}

// ==============================================================================
// TF 插值
// ==============================================================================

geometry_msgs::msg::TransformStamped RobotSelfFilter::interpolateTransform(
    const geometry_msgs::msg::TransformStamped& tf1,
    const geometry_msgs::msg::TransformStamped& tf2,
    const rclcpp::Time& query_time)
{
    const auto* t_prev = &tf1;
    const auto* t_curr = &tf2;

    if (tf1.header.stamp.sec > tf2.header.stamp.sec) {
        t_prev = &tf2;
        t_curr = &tf1;
    }

    double t1 = t_prev->header.stamp.sec;
    double t2 = t_curr->header.stamp.sec;
    double t_query = query_time.seconds();

    double ratio = (std::abs(t2 - t1) < 1e-9)
                       ? 0.0
                       : (t_query - t1) / (t2 - t1);

    geometry_msgs::msg::TransformStamped result;
    result.header.stamp = query_time;
    result.header.frame_id = t_prev->header.frame_id;
    result.child_frame_id = t_prev->child_frame_id;

    // 位置线性插值
    tf2::Vector3 pos1(
        t_prev->transform.translation.x,
        t_prev->transform.translation.y,
        t_prev->transform.translation.z);
    tf2::Vector3 pos2(
        t_curr->transform.translation.x,
        t_curr->transform.translation.y,
        t_curr->transform.translation.z);
    tf2::Vector3 pos_interp = pos1.lerp(pos2, ratio);

    result.transform.translation.x = pos_interp.x();
    result.transform.translation.y = pos_interp.y();
    result.transform.translation.z = pos_interp.z();

    // 姿态 SLERP
    tf2::Quaternion q1(
        t_prev->transform.rotation.x,
        t_prev->transform.rotation.y,
        t_prev->transform.rotation.z,
        t_prev->transform.rotation.w);
    tf2::Quaternion q2(
        t_curr->transform.rotation.x,
        t_curr->transform.rotation.y,
        t_curr->transform.rotation.z,
        t_curr->transform.rotation.w);

    if (q1.dot(q2) < 0.0) {
        q1.inverse();
    }

    tf2::Quaternion q_interp = q1.slerp(q2, ratio);
    q_interp.normalize();

    result.transform.rotation.x = q_interp.x();
    result.transform.rotation.y = q_interp.y();
    result.transform.rotation.z = q_interp.z();
    result.transform.rotation.w = q_interp.w();

    return result;
}

// ==============================================================================
// 点云回调
// ==============================================================================

void RobotSelfFilter::pointCloudCallback(const CustomMsg::SharedPtr msg)
{
    if(!pass_through_enabled_ && !capsule_filter_enabled_) {
        pub_->publish(*msg);
        return;
    }

    rclcpp::Time current_stamp = msg->header.stamp;
    
    // ---- 第1步：为每个连杆构建扫掠胶囊 ----
    // auto start_time = std::chrono::high_resolution_clock::now();

    for (const auto& link : links_) {
        try {
            auto prev_trans = tf_buffer_->lookupTransform(
                lidar_frame_, link, prev_stamp_);
            auto new_trans = tf_buffer_->lookupTransform(
                lidar_frame_, link, tf2::TimePointZero);
            auto trans = interpolateTransform(prev_trans, new_trans, current_stamp);

            auto prev_capsule = createCapsuleFromPose(
                link_properties_[link],
                Vector3f(prev_trans.transform.translation.x,
                         prev_trans.transform.translation.y,
                         prev_trans.transform.translation.z),
                Quaternionf(prev_trans.transform.rotation.w,
                            prev_trans.transform.rotation.x,
                            prev_trans.transform.rotation.y,
                            prev_trans.transform.rotation.z));

            auto curr_capsule = createCapsuleFromPose(
                link_properties_[link],
                Vector3f(trans.transform.translation.x,
                         trans.transform.translation.y,
                         trans.transform.translation.z),
                Quaternionf(trans.transform.rotation.w,
                            trans.transform.rotation.x,
                            trans.transform.rotation.y,
                            trans.transform.rotation.z));

            link_sweep_capsule_[link] = createSweepCapsule(prev_capsule, curr_capsule);
            link_sweep_capsule_[link].r *= expansion_factor_;
            buildCapsuleAABB(link_sweep_capsule_[link]);
            link_sweep_capsule_[link].aabb_volume_ratio = capsuleAABBVolumeRatio(link_sweep_capsule_[link]);
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(),
                         "[%f] Failed to get %s TF transform at %f",
                         current_stamp.seconds(), link.c_str(), prev_stamp_.seconds());
            prev_stamp_ = current_stamp;
            return;
        }
    }
    prev_stamp_ = current_stamp;

    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // RCLCPP_INFO(this->get_logger(), "构建扫掠胶囊耗时: %ld us", duration.count());

    // ---- 第2步：筛选与AABB相交的连杆 ----
    // start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::string> valid_links;
    for (const auto& link : links_) {
        if (isCapsuleIntersectAABB(link_sweep_capsule_[link])) {
            valid_links.push_back(link);
        }
    }

    // end_time = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // RCLCPP_INFO(this->get_logger(), "筛选交连杆耗时: %ld us", duration.count());

    // ---- 第3步：滤波（OpenMP 加速可选） ----
    auto start_time3 = std::chrono::high_resolution_clock::now();

    auto& points = msg->points;
    size_t n = points.size();

    // 判定谓词：返回 true 表示剔除该点
    auto should_discard = [this, &valid_links](const CustomPoint& p) -> bool {
        // 直通滤波保护：直通区域外的点保留
        if (pass_through_enabled_) {
            bool outside_roi = (p.x < x_min_ || p.x > x_max_ ||
                                p.y < y_min_ || p.y > y_max_ ||
                                p.z < z_min_ || p.z > z_max_);
            if (outside_roi) {
                return false;
            }
        }
        // 胶囊精细滤波：按体积比选择 AABB 快速检测或胶囊精确检测
        if (capsule_filter_enabled_) {
            const Vector3f pt(p.x, p.y, p.z);
            for (const auto& link : valid_links) {
                const auto& cap = link_sweep_capsule_[link];
                if (cap.aabb_volume_ratio > aabb_ratio_threshold_) {
                    // AABB 包围盒紧密 → 直通检测，更快
                    if (pointInAABB(pt, cap.aabb_min, cap.aabb_max)) {
                        return true;
                    }
                } else {
                    // 包围盒不紧密 → 精确胶囊检测
                    if (pointInCapsule(pt, cap)) {
                        return true;
                    }
                }
            }
            return false;
        }
        return true;
    };

    
    if (openmp_core_num_ > 1 && n > 1024) {
        // OpenMP 并行路径：并行判定 + 串行压实
        size_t kept = 0;
        std::vector<bool> discard(n);
        const auto* raw_points = points.data();

        #pragma omp parallel for schedule(static)
        for (size_t i = 0; i < n; ++i) {
            discard[i] = should_discard(raw_points[i]) ? true : false;
        }

        // 原地压实
        for (size_t i = 0; i < n; ++i) {
            if (!discard[i]) {
                if (kept != i) {
                    points[kept] = points[i];
                }
                ++kept;
            }
        }
        points.resize(kept);
    } else {
        // 单线程路径
        points.erase(
            std::remove_if(points.begin(), points.end(), should_discard),
            points.end());
    }

    auto end_time3 = std::chrono::high_resolution_clock::now();
    auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>(end_time3 - start_time3);
    RCLCPP_INFO(this->get_logger(), "滤波 %ld us | 输入 %d 点 → 输出 %ld 点%s",
                duration3.count(), msg->point_num, points.size(),
                (openmp_core_num_ > 1 && n > 1024) ?
                    (" | OpenMP " + std::to_string(omp_get_max_threads()) + " 核").c_str() : "");

    msg->point_num = points.size();
    pub_->publish(*msg);
    if(viz_enabled_)
        publishCapsuleMarkers(current_stamp);
}

// ==============================================================================
// 可视化
// ==============================================================================

geometry_msgs::msg::Quaternion RobotSelfFilter::quaternionFromAxis(const Vector3f& axis)
{
    // 将胶囊轴线方向转为四元数（RViz CYLINDER 默认沿 Z 轴）
    Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), axis.normalized());
    geometry_msgs::msg::Quaternion qm;
    qm.x = q.x(); qm.y = q.y(); qm.z = q.z(); qm.w = q.w();
    return qm;
}

void RobotSelfFilter::publishCapsuleMarkers(const rclcpp::Time& stamp)
{
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;

    for (const auto& link : links_) {
        const auto& cap = link_sweep_capsule_[link];
        if (cap.r <= 0.0f) continue;

        Vector3f direction = cap.end - cap.start;
        float len = direction.norm();
        Vector3f center = (cap.start + cap.end) * 0.5f;
        auto q = quaternionFromAxis(direction);

        // 按体积比区分颜色：紧密 AABB → 绿色（走快速路径），松散 → 黄色（走精确路径）
        bool use_aabb = cap.aabb_volume_ratio > aabb_ratio_threshold_;
        float cr = use_aabb ? 0.2f : 1.0f;
        float cg = use_aabb ? 1.0f : 0.8f;
        float cb = 0.2f;

        auto make_marker = [&](int type) -> visualization_msgs::msg::Marker {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = lidar_frame_;
            m.header.stamp = stamp;
            m.ns = "robot_capsules";
            m.id = id++;
            m.type = type;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = center.x();
            m.pose.position.y = center.y();
            m.pose.position.z = center.z();
            m.pose.orientation = q;
            m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = 0.35f;
            m.lifetime = rclcpp::Duration::from_seconds(10.0f / lidar_hz_);
            return m;
        };

        // 圆柱体（杆身）
        auto cyl = make_marker(visualization_msgs::msg::Marker::CYLINDER);
        cyl.scale.x = cap.r * 2.0f;
        cyl.scale.y = cap.r * 2.0f;
        cyl.scale.z = len;
        ma.markers.push_back(cyl);

        // // 两端球体
        // for (const auto& ep : {cap.start, cap.end}) {
        //     auto sph = make_marker(visualization_msgs::msg::Marker::SPHERE);
        //     sph.pose.position.x = ep.x();
        //     sph.pose.position.y = ep.y();
        //     sph.pose.position.z = ep.z();
        //     // 球体不需要方向
        //     sph.pose.orientation.x = 0; sph.pose.orientation.y = 0;
        //     sph.pose.orientation.z = 0; sph.pose.orientation.w = 1;
        //     sph.scale.x = cap.r * 2.0f;
        //     sph.scale.y = cap.r * 2.0f;
        //     sph.scale.z = cap.r * 2.0f;
        //     ma.markers.push_back(sph);
        // }
    }

    viz_pub_->publish(ma);
}

}  // namespace robot_self_filter
