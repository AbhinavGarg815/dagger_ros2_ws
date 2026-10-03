// safety_scan: /rslidar_points -> /scan_safety, the collision monitors' only obstacle source.
//
// The same projection as pointcloud_to_laserscan, which produced /scan_safety until
// 2026-10-02 (p2l_safety): points inside a height band, nearest return per bearing,
// in base_footprint, identical scan geometry. Two differences:
//
// 1. BODY BOX. Points whose x/y in base_footprint fall inside body_box (all heights)
//    are dropped BEFORE the projection. They are the rover itself: the top frame
//    (side rails y ~ +-0.51, rear rail x ~ -0.84, z 0.81-1.11) and two posts at
//    (0.31-0.41, +-0.30); report Q65. It has to happen on the cloud: in a scan only
//    the nearest return per bearing is left, and the frame occupies every bearing
//    from +-90 to 180 deg, so filtering the scan would blind the whole rear half.
//    With the body gone, range_min drops from 1.0 to 0.3 m and a person beside the
//    hull -- inside the 1.03 m circle the corners sweep when turning in place --
//    is seen (walk capture: 0 rays before, 81-87 after).
//
// 2. STATIC MOUNT. The cloud -> base_footprint transform comes from /tf_static only
//    (the LiDAR mount is rigid): no listener on /tf, no message filter. The cloud
//    subscription is permanent, so p2l's lazy-subscribe wedge (L9/P9) cannot happen.
//
// No transform yet -> no scan. cmd_vel_safety zeroes all motion when /scan_safety is
// older than 0.5 s, so every failure of this node is a stop.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Eigen 3.4.0 on Ubuntu 22.04 / aarch64 warns -Wclass-memaccess (as in tf2_sensor_msgs).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;

class SafetyScan : public rclcpp::Node
{
public:
  SafetyScan()
  : Node("safety_scan")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    min_height_ = declare_parameter<double>("min_height", 0.30);
    // 1.35: the rover's top (LiDAR, ~1.17 m) + margin; anything entirely above passes
    // over (Q71). nav2.launch.py sets it from SAFETY_BAND_TOP.
    max_height_ = declare_parameter<double>("max_height", 1.35);
    range_min_ = declare_parameter<double>("range_min", 0.3);
    range_max_ = declare_parameter<double>("range_max", 20.0);
    const bool use_inf = declare_parameter<bool>("use_inf", true);
    const double inf_epsilon = declare_parameter<double>("inf_epsilon", 1.0);
    body_filter_ = declare_parameter<bool>("body_filter", true);
    box_ = declare_parameter<std::vector<double>>("body_box", {-0.90, 0.50, -0.60, 0.60});
    const double stats_period = declare_parameter<double>("stats_period", 60.0);
    if (box_.size() != 4 || !(box_[0] < box_[1]) || !(box_[2] < box_[3])) {
      throw std::invalid_argument("body_box must be [x_min, x_max, y_min, y_max]");
    }

    // Header fields exactly as p2l fills them; they are float32 in the message, and
    // the bin count and bin index are computed from those float values as p2l does.
    proto_.header.frame_id = target_frame_;
    proto_.angle_min = declare_parameter<double>("angle_min", -3.14159);
    proto_.angle_max = declare_parameter<double>("angle_max", 3.14159);
    proto_.angle_increment = declare_parameter<double>("angle_increment", 0.0087);
    proto_.time_increment = 0.0;
    proto_.scan_time = declare_parameter<double>("scan_time", 0.1);
    proto_.range_min = range_min_;
    proto_.range_max = range_max_;
    const auto n_bins = static_cast<uint32_t>(
      std::ceil((proto_.angle_max - proto_.angle_min) / proto_.angle_increment));
    proto_.ranges.assign(
      n_bins, use_inf ? std::numeric_limits<float>::infinity() :
      static_cast<float>(range_max_ + inf_epsilon));

    pub_ = create_publisher<LaserScan>("scan", rclcpp::SensorDataQoS());
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & t : msg->transforms) {
          buffer_.setTransform(t, "tf_static", true);
        }
        have_tf_ = false;  // look it up again: a restarted robot model may have changed it
      });
    cloud_sub_ = create_subscription<PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS().keep_last(2),
      [this](PointCloud2::ConstSharedPtr msg) {on_cloud(msg);});
    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period), [this]() {stats();});

    RCLCPP_INFO(
      get_logger(), "safety_scan: %u bins in %s, band %.2f-%.2f m, range %.2f-%.2f m, "
      "body box %s x[%.2f, %.2f] y[%.2f, %.2f]", n_bins, target_frame_.c_str(),
      min_height_, max_height_, range_min_, range_max_, body_filter_ ? "ON" : "OFF",
      box_[0], box_[1], box_[2], box_[3]);
  }

private:
  bool transform_for(const std::string & frame)
  {
    if (have_tf_ && frame == tf_frame_) {
      return true;
    }
    geometry_msgs::msg::TransformStamped t;
    if (frame == target_frame_) {
      t.transform.rotation.w = 1.0;
    } else {
      try {
        t = buffer_.lookupTransform(target_frame_, frame, tf2::TimePointZero);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "safety_scan: no scan until /tf_static has "
          "%s <- %s (%s)", target_frame_.c_str(), frame.c_str(), e.what());
        return false;
      }
    }
    // Built as tf2_sensor_msgs::doTransform builds it (float, translation * rotation),
    // so the projected points are bit-identical to p2l's.
    const auto & tr = t.transform.translation;
    const auto & q = t.transform.rotation;
    tf_ = Eigen::Translation3f(tr.x, tr.y, tr.z) * Eigen::Quaternion<float>(q.w, q.x, q.y, q.z);
    tf_frame_ = frame;
    have_tf_ = true;
    RCLCPP_INFO(
      get_logger(), "safety_scan: %s -> %s from /tf_static: t (%.4f, %.4f, %.4f), "
      "q (%.4f, %.4f, %.4f, %.4f)", frame.c_str(), target_frame_.c_str(), tr.x, tr.y, tr.z,
      q.x, q.y, q.z, q.w);
    return true;
  }

  void on_cloud(const PointCloud2::ConstSharedPtr & msg)
  {
    if (!transform_for(msg->header.frame_id)) {
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    auto scan = std::make_unique<LaserScan>(proto_);
    scan->header.stamp = msg->header.stamp;
    const double a_min = scan->angle_min, a_max = scan->angle_max;
    const double a_inc = scan->angle_increment;
    const int n_bins = static_cast<int>(scan->ranges.size());
    size_t body = 0;

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const Eigen::Vector3f p = tf_ * Eigen::Vector3f(*ix, *iy, *iz);
      const float x = p.x(), y = p.y(), z = p.z();
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        continue;
      }
      if (body_filter_ && x >= box_[0] && x <= box_[1] && y >= box_[2] && y <= box_[3]) {
        ++body;
        continue;
      }
      if (z > max_height_ || z < min_height_) {
        continue;
      }
      // float hypot/atan2, as in p2l (its unqualified calls on floats resolve to the
      // float overloads): in double, a point sitting on a bin edge changed bins and
      // 93 of 1.1 M bins differed from the live /scan_safety; in float, none (Q65).
      const double range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }
      const double angle = std::atan2(y, x);
      if (angle < a_min || angle > a_max) {
        continue;
      }
      const int index = std::min(static_cast<int>((angle - a_min) / a_inc), n_bins - 1);
      if (range < scan->ranges[index]) {
        scan->ranges[index] = range;
      }
    }
    pub_->publish(std::move(scan));

    ++clouds_;
    body_points_ += body;
    compute_s_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }

  void stats()
  {
    if (clouds_ == 0) {
      RCLCPP_WARN(get_logger(), "safety_scan: no clouds processed in the last period");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "safety_scan: %zu clouds, body box removed %.0f points/cloud, %.2f ms/cloud",
      clouds_, static_cast<double>(body_points_) / clouds_, 1000.0 * compute_s_ / clouds_);
    clouds_ = 0;
    body_points_ = 0;
    compute_s_ = 0.0;
  }

  std::string target_frame_;
  double min_height_, max_height_, range_min_, range_max_;
  bool body_filter_;
  std::vector<double> box_;
  LaserScan proto_;

  tf2::BufferCore buffer_;
  bool have_tf_ = false;
  std::string tf_frame_;
  Eigen::Transform<float, 3, Eigen::Affine> tf_;

  size_t clouds_ = 0, body_points_ = 0;
  double compute_s_ = 0.0;

  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyScan>());
  rclcpp::shutdown();
  return 0;
}
