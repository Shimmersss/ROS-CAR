#ifndef LIDAR_FUSION_HPP_
#define LIDAR_FUSION_HPP_

#include <memory>
#include <string>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/**
 * @brief 订阅两台 2D 激光雷达的 LaserScan，融合成 360° LaserScan。
 *
 * 可通过参数或 launch 文件配置各 Topic 名、frame_id 以及两台雷达的外参
 * （旋转角度 + XY 平移，均为雷达坐标系到机器人基坐标系）
 */
class LidarFusion : public rclcpp::Node
{
public:
  explicit LidarFusion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LidarFusion() override = default;

private:
  /* ---------- 同步策略 ---------- */
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::LaserScan,
      sensor_msgs::msg::LaserScan>;

  /* ---------- ROS 接口 ---------- */
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan1_sub_;   // /scan1
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan2_sub_;   // /scan2
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> scans_sync_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_scan_pub_; // /scan

  /* ---------- 参数 ---------- */
  std::string frame_id_;
  std::string fused_topic_;   // 发布 topic（默认 "scan"）
  std::string scan1_topic_;
  std::string scan2_topic_;

  double  lidar1_angle_deg_{0.0};   // 雷达 1 旋转角（度）
  double  lidar1_x_offset_m_{0.0};  // 雷达 1 X 平移（米）
  double  lidar1_y_offset_m_{0.0};  // 雷达 1 Y 平移（米）
  double  lidar2_angle_deg_{0.0};   // 雷达 2 旋转角
  double  lidar2_x_offset_m_{0.0};  // 雷达 2 X 平移
  double  lidar2_y_offset_m_{0.0};  // 雷达 2 Y 平移

  /* ---------- 运行时缓存 ---------- */
  double   angle_increment_rad_{0.0};   // 单束角度分辨率 (rad)
  size_t   bucket_count_{0};            // 360° 被划分的桶数
  uint64_t published_count_{0};         // 已发布的融合帧数
  bool has_received_scans_ = false;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  /* ---------- 回调 ---------- */
  void fuseScans(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan1,
                 const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan2);
};

#endif  