#include "double_lidar_fusion/lidar_fusion.hpp"
#include <cmath>
#include <limits>

using sensor_msgs::msg::LaserScan;

LidarFusion::LidarFusion(const rclcpp::NodeOptions & options)
: Node("double_lidar_fusion", options)
{
  declare_parameter("scan1_topic",          "/scan1");
  declare_parameter("scan2_topic",          "/scan2");
  declare_parameter("fused_topic",          "scan");
  declare_parameter("frame_id",             "laser");
  declare_parameter("lidar1_angle_deg",      45.0);
  declare_parameter("lidar1_x_offset_m",     0.3);
  declare_parameter("lidar1_y_offset_m",     0.235);
  declare_parameter("lidar2_angle_deg",     -135.0);
  declare_parameter("lidar2_x_offset_m",     -0.3);
  declare_parameter("lidar2_y_offset_m",     -0.235);

  get_parameter("scan1_topic",          scan1_topic_);
  get_parameter("scan2_topic",          scan2_topic_);
  get_parameter("fused_topic",          fused_topic_);
  get_parameter("frame_id",             frame_id_);
  get_parameter("lidar1_angle_deg",     lidar1_angle_deg_);
  get_parameter("lidar1_x_offset_m",    lidar1_x_offset_m_);
  get_parameter("lidar1_y_offset_m",    lidar1_y_offset_m_);
  get_parameter("lidar2_angle_deg",     lidar2_angle_deg_);
  get_parameter("lidar2_x_offset_m",    lidar2_x_offset_m_);
  get_parameter("lidar2_y_offset_m",    lidar2_y_offset_m_);

  // 打印初始化参数信息
  RCLCPP_INFO(this->get_logger(), "============= 双雷达融合节点启动 =============");
  RCLCPP_INFO(this->get_logger(), "雷达1话题: %s", scan1_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "雷达2话题: %s", scan2_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "融合输出话题: %s", fused_topic_.c_str());
  
  // 设置 QoS 为 SensorData (Best Effort)，兼容大多数雷达驱动
  auto qos = rclcpp::SensorDataQoS();
  
  scan1_sub_.subscribe(this, scan1_topic_, qos.get_rmw_qos_profile());
  scan2_sub_.subscribe(this, scan2_topic_, qos.get_rmw_qos_profile());

  scans_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(20), scan1_sub_, scan2_sub_); // 队列大小设为20增加容错
  scans_sync_->registerCallback(
      std::bind(&LidarFusion::fuseScans, this, std::placeholders::_1, std::placeholders::_2));

  fused_scan_pub_ = create_publisher<LaserScan>(fused_topic_, 10);

  // 添加一个定时器用于监控：如果一直没有进入回调，定期打印警告
  monitor_timer_ = this->create_wall_timer(
      std::chrono::seconds(3), 
      [this]() {
          if (!has_received_scans_) {
              RCLCPP_WARN(this->get_logger(), 
                  "尚未接收到同步的雷达数据！请检查: 1.雷达话题名是否正确 2.两台雷达是否都在发数据 3.时间戳是否同步");
          }
      });
}

/* ============================================================ */
void LidarFusion::fuseScans(const LaserScan::ConstSharedPtr & scan1,
                            const LaserScan::ConstSharedPtr & scan2)
{
  has_received_scans_ = true; // 标记已收到数据

  // 使用 THROTTLE 防止日志刷屏，每 2000 毫秒 (2秒) 打印一次
  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
  //     "成功触发同步回调！(scan1 时间戳: %u.%u, scan2 时间戳: %u.%u)",
  //     scan1->header.stamp.sec, scan1->header.stamp.nanosec,
  //     scan2->header.stamp.sec, scan2->header.stamp.nanosec);

  if (angle_increment_rad_ == 0.0) {
    angle_increment_rad_ = scan1->angle_increment;
    bucket_count_ = static_cast<size_t>(std::round(2 * M_PI / angle_increment_rad_));
    RCLCPP_INFO(this->get_logger(), "初始化融合参数: 角度分辨率=%.4f rad, 总点数=%zu", 
                angle_increment_rad_, bucket_count_);
  }

  constexpr double ANGLE_MIN = -M_PI;         // 输出激光从 -π 开始
  constexpr double ANGLE_MAX =  M_PI;
  const size_t     TOTAL_BUCKETS = bucket_count_;
  constexpr float  MERGE_THRESHOLD_M = 0.03f; // 3 cm 死区，抑制闪烁
  constexpr float  MIN_VALID_RANGE = 0.05f;   // 最小有效距离，过滤近场噪声和0值

  LaserScan fused_scan;
  fused_scan.header.stamp       = scan1->header.stamp;             // 取第一台时间戳
  fused_scan.header.frame_id    = frame_id_;
  fused_scan.angle_min          = ANGLE_MIN;
  fused_scan.angle_max          = ANGLE_MAX;
  fused_scan.angle_increment    = angle_increment_rad_;
  fused_scan.range_min          = std::min(scan1->range_min, scan2->range_min);
  fused_scan.range_max          = std::max(scan1->range_max, scan2->range_max);
  fused_scan.scan_time          = scan1->scan_time;
  fused_scan.time_increment     = fused_scan.scan_time /
                                  static_cast<double>(TOTAL_BUCKETS);
  fused_scan.ranges.assign(TOTAL_BUCKETS, std::numeric_limits<float>::infinity());
  fused_scan.intensities.assign(TOTAL_BUCKETS, 0.0f);

  /* ---------- 预计算 cos/sin ---------- */
  const double cos_lidar1 = std::cos(lidar1_angle_deg_ * M_PI / 180.0);
  const double sin_lidar1 = std::sin(lidar1_angle_deg_ * M_PI / 180.0);
  const double cos_lidar2 = std::cos(lidar2_angle_deg_ * M_PI / 180.0);
  const double sin_lidar2 = std::sin(lidar2_angle_deg_ * M_PI / 180.0);

  /* ---------- 定义融合 Lambda ---------- */
  auto integrateOneLidar = [&](const LaserScan::ConstSharedPtr & scan,
                               double cos_theta, double sin_theta,
                               double x_offset, double y_offset)
  {
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      const float raw_range = scan->ranges[i];

      // 严格过滤无效值，包括0值、inf、nan和过小值
      if (!std::isfinite(raw_range)) continue;
      if (raw_range < MIN_VALID_RANGE) continue;  // 过滤0值和极小值
      if (raw_range < scan->range_min || raw_range > scan->range_max) continue;

      // 使用该扫描自己的角度分辨率计算局部角度
      const double local_angle = scan->angle_min + scan->angle_increment * static_cast<double>(i);
      const double local_x     = raw_range * std::cos(local_angle);
      const double local_y     = raw_range * std::sin(local_angle);

      /* --- 局部坐标 -> 机器人基坐标 --- */
      const double base_x = local_x * cos_theta - local_y * sin_theta + x_offset;
      const double base_y = local_x * sin_theta + local_y * cos_theta + y_offset;

      const float  fused_range  = std::hypot(base_x, base_y);
      const float  fused_angle  = std::atan2(base_y, base_x);

      /* 把角度包裹到 (-π, π]  */
      int bucket_index = static_cast<int>(std::floor((fused_angle - ANGLE_MIN) /
                                                     angle_increment_rad_));
      if (bucket_index < 0 || static_cast<size_t>(bucket_index) >= TOTAL_BUCKETS) continue;

      float & current_value = fused_scan.ranges[static_cast<size_t>(bucket_index)];
      if (std::isinf(current_value)) {
        current_value = fused_range;
      } else if (std::fabs(current_value - fused_range) < MERGE_THRESHOLD_M) {
        current_value = 0.5f * (current_value + fused_range);
      } else if (fused_range < current_value) {
        current_value = fused_range;                               // 取最近者
      }
    }
  };

  /* ----------  融合两台雷达 ---------- */
  integrateOneLidar(scan1, cos_lidar1, sin_lidar1,
                    lidar1_x_offset_m_, lidar1_y_offset_m_);
  integrateOneLidar(scan2, cos_lidar2, sin_lidar2,
                    lidar2_x_offset_m_, lidar2_y_offset_m_);

  // 统计最终有效的点数，用于调试
  int valid_points = 0;
  for (const auto& r : fused_scan.ranges) {
      if (std::isfinite(r)) valid_points++;
  }
  
  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
  //     "发布融合点云, 有效点数: %d / %zu", valid_points, TOTAL_BUCKETS);

  /* ---------- 发布结果 ---------- */
  fused_scan_pub_->publish(fused_scan);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFusion>());
  rclcpp::shutdown();
  return 0;
}