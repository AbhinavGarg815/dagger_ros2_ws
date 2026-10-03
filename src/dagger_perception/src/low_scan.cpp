// low_scan: Patchwork++ non-ground cloud -> /scan_low, the costmap low_obstacle_layer's source.
//
// WHY (Q30, Q68). The LiDAR, 1.12 m up, stops seeing a low object once the rover is
// within ~3.5-4.5 m of it: the beams pass over. The costmap's normal layer then clears
// it with "nothing there" rays. Live 2026-10-03: Nav2 planned 0.8 m around a 26 cm box
// seen from 7.2 m, the box was erased with the rover still 3.8 m short, and the next
// replan drove over it. The low_obstacle_layer ray-clears only beyond 4 m, so whatever
// enters it is REMEMBERED while the rover closes in -- which means only things that are
// truly low and static may enter it.
//
// Per bearing (same 723 bins as the other scans), from one pass over the cloud:
//   low  = nearest return 12-35 cm high,   tall = nearest return 40-160 cm high.
//   low with something tall within +-tall_bins bearings, nearer than low + tall_margin
//        -> NaN  (part of something tall: legs under a torso; Q32's blobs came from a
//                 torso one ray away, so the check spans neighbouring bearings)
//   low only, cell CONFIRMED                     -> its range   (marked, then remembered)
//   low only, not confirmed yet                  -> NaN         (not marked, not cleared)
//   tall only                                    -> NaN         (cannot clear through it)
//   neither                                      -> inf         (clears, beyond 4 m only)
// CONFIRMED = the low return's 15 cm cell in the ODOM frame (with its 8 neighbours) was
// hit in >= min_hits scans within persist_window s, and those hits span >= min_span s.
// A box stays in its cell for seconds; a walker crosses a 45 cm neighbourhood in ~0.3 s
// at 1.4 m/s (without min_span, 3 consecutive scans of a walker still landed in one
// neighbourhood). Without odometry nothing is confirmed (fails toward "not remembered").

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#include <Eigen/Geometry>
#pragma GCC diagnostic pop

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::PointCloud2;

class LowScan : public rclcpp::Node
{
public:
  LowScan()
  : Node("low_scan")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    low_min_ = declare_parameter<double>("low_min_height", 0.12);
    low_max_ = declare_parameter<double>("low_max_height", 0.35);
    tall_min_ = declare_parameter<double>("tall_min_height", 0.40);
    tall_max_ = declare_parameter<double>("tall_max_height", 1.60);
    tall_margin_ = declare_parameter<double>("tall_margin", 0.5);
    tall_bins_ = declare_parameter<int>("tall_bins", 2);
    range_min_ = declare_parameter<double>("range_min", 0.5);
    range_max_ = declare_parameter<double>("range_max", 30.0);
    box_ = declare_parameter<std::vector<double>>("body_box", {-0.90, 0.50, -0.60, 0.60});
    persist_ = declare_parameter<bool>("persistence", true);
    cell_ = declare_parameter<double>("cell_size", 0.15);
    window_ = declare_parameter<double>("persist_window", 2.0);
    min_hits_ = declare_parameter<int>("min_hits", 3);
    min_span_ = declare_parameter<double>("min_span", 0.5);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    const double stats_period = declare_parameter<double>("stats_period", 60.0);
    if (box_.size() != 4 || !(box_[0] < box_[1]) || !(box_[2] < box_[3])) {
      throw std::invalid_argument("body_box must be [x_min, x_max, y_min, y_max]");
    }

    proto_.header.frame_id = target_frame_;
    proto_.angle_min = declare_parameter<double>("angle_min", -3.14159);
    proto_.angle_max = declare_parameter<double>("angle_max", 3.14159);
    proto_.angle_increment = declare_parameter<double>("angle_increment", 0.0087);
    proto_.time_increment = 0.0;
    proto_.scan_time = declare_parameter<double>("scan_time", 0.1);
    proto_.range_min = range_min_;
    proto_.range_max = range_max_;
    n_bins_ = static_cast<int>(
      std::ceil((proto_.angle_max - proto_.angle_min) / proto_.angle_increment));

    pub_ = create_publisher<LaserScan>("scan", rclcpp::SensorDataQoS());
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & t : msg->transforms) {
          buffer_.setTransform(t, "tf_static", true);
        }
        have_tf_ = false;
      });
    odom_sub_ = create_subscription<Odometry>(
      "odom", rclcpp::QoS(10).best_effort(),
      [this](Odometry::ConstSharedPtr msg) {
        const auto & p = msg->pose.pose;
        odom_x_ = p.position.x;
        odom_y_ = p.position.y;
        odom_yaw_ = std::atan2(
          2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
          1.0 - 2.0 * (p.orientation.y * p.orientation.y + p.orientation.z * p.orientation.z));
        odom_time_ = std::chrono::steady_clock::now();
        have_odom_ = true;
      });
    cloud_sub_ = create_subscription<PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS().keep_last(2),
      [this](PointCloud2::ConstSharedPtr msg) {on_cloud(msg);});
    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period), [this]() {stats();});

    RCLCPP_INFO(
      get_logger(), "low_scan: low %.2f-%.2f m, tall %.2f-%.2f m within +-%d bins, persistence %s "
      "(%.2f m cells, %d hits over >= %.1f s within %.1f s)", low_min_, low_max_, tall_min_, tall_max_,
      tall_bins_, persist_ ? "ON" : "OFF", cell_, min_hits_, min_span_, window_);
  }

private:
  using Cell = std::pair<int64_t, int64_t>;

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
          get_logger(), *get_clock(), 5000, "low_scan: no scan until /tf_static has %s <- %s (%s)",
          target_frame_.c_str(), frame.c_str(), e.what());
        return false;
      }
    }
    const auto & tr = t.transform.translation;
    const auto & q = t.transform.rotation;
    tf_ = Eigen::Translation3f(tr.x, tr.y, tr.z) * Eigen::Quaternion<float>(q.w, q.x, q.y, q.z);
    tf_frame_ = frame;
    have_tf_ = true;
    RCLCPP_INFO(get_logger(), "low_scan: %s -> %s from /tf_static", frame.c_str(), target_frame_.c_str());
    return true;
  }

  // Distinct scans within the window that hit this cell or a neighbour: enough of them,
  // spread over at least min_span seconds?
  bool confirmed(const Cell & c) const
  {
    std::set<uint64_t> scans;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        auto it = hits_.find({c.first + dx, c.second + dy});
        if (it != hits_.end()) {
          scans.insert(it->second.begin(), it->second.end());
        }
      }
    }
    if (static_cast<int>(scans.size()) < min_hits_) {
      return false;
    }
    const double span = hit_time_.at(*scans.rbegin()) - hit_time_.at(*scans.begin());
    return span >= min_span_;
  }

  void on_cloud(const PointCloud2::ConstSharedPtr & msg)
  {
    if (!transform_for(msg->header.frame_id)) {
      return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const double inf = std::numeric_limits<double>::infinity();
    const double a_min = proto_.angle_min, a_max = proto_.angle_max, a_inc = proto_.angle_increment;
    std::vector<double> low(n_bins_, inf), tall(n_bins_, inf);
    std::vector<float> low_x(n_bins_, 0.f), low_y(n_bins_, 0.f);

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const Eigen::Vector3f p = tf_ * Eigen::Vector3f(*ix, *iy, *iz);
      const float x = p.x(), y = p.y(), z = p.z();
      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        continue;
      }
      if (x >= box_[0] && x <= box_[1] && y >= box_[2] && y <= box_[3]) {
        continue;                       // the rover itself (frame, posts)
      }
      const bool is_low = z >= low_min_ && z <= low_max_;
      const bool is_tall = z >= tall_min_ && z <= tall_max_;
      if (!is_low && !is_tall) {
        continue;
      }
      const double range = std::hypot(x, y);
      if (range < range_min_ || range > range_max_) {
        continue;
      }
      const double angle = std::atan2(y, x);
      if (angle < a_min || angle > a_max) {
        continue;
      }
      const int b = std::min(static_cast<int>((angle - a_min) / a_inc), n_bins_ - 1);
      if (is_low && range < low[b]) {
        low[b] = range;
        low_x[b] = x;
        low_y[b] = y;
      }
      if (is_tall && range < tall[b]) {
        tall[b] = range;
      }
    }

    // Odometry for the persistence cells.
    const double now_s = rclcpp::Time(msg->header.stamp).seconds();
    const bool odom_ok = have_odom_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - odom_time_).count() < odom_timeout_;
    const double c = std::cos(odom_yaw_), s = std::sin(odom_yaw_);

    auto scan = std::make_unique<LaserScan>(proto_);
    scan->header.stamp = msg->header.stamp;
    scan->ranges.assign(n_bins_, std::numeric_limits<float>::infinity());
    std::vector<std::pair<int, Cell>> candidates;
    for (int b = 0; b < n_bins_; ++b) {
      if (!std::isfinite(low[b])) {
        if (std::isfinite(tall[b])) {
          scan->ranges[b] = std::numeric_limits<float>::quiet_NaN();   // tall only: no clearing
        }
        continue;                                                     // neither: inf (clears)
      }
      // Something tall nearer than (or just behind) the low return, on this bearing or a
      // neighbouring one, makes it part of a tall thing. A wall well behind a box does not.
      bool tall_near = false;
      for (int k = -tall_bins_; k <= tall_bins_ && !tall_near; ++k) {
        const int j = (b + k + n_bins_) % n_bins_;
        tall_near = std::isfinite(tall[j]) && tall[j] < low[b] + tall_margin_;
      }
      if (tall_near) {
        scan->ranges[b] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      if (!persist_) {
        scan->ranges[b] = low[b];
        ++n_confirmed_;
        continue;
      }
      scan->ranges[b] = std::numeric_limits<float>::quiet_NaN();     // until confirmed
      if (!odom_ok) {
        continue;
      }
      const double ox = odom_x_ + c * low_x[b] - s * low_y[b];
      const double oy = odom_y_ + s * low_x[b] + c * low_y[b];
      candidates.push_back({b, {static_cast<int64_t>(std::floor(ox / cell_)),
          static_cast<int64_t>(std::floor(oy / cell_))}});
    }
    if (persist_) {
      ++scan_index_;
      for (const auto & bc : candidates) {          // one hit per cell per scan
        auto & d = hits_[bc.second];
        if (d.empty() || d.back() != scan_index_) {
          d.push_back(scan_index_);
          hit_time_[scan_index_] = now_s;
        }
      }
      // forget hits older than the window
      while (!hit_time_.empty() && now_s - hit_time_.begin()->second > window_) {
        const uint64_t old = hit_time_.begin()->first;
        hit_time_.erase(hit_time_.begin());
        for (auto it = hits_.begin(); it != hits_.end(); ) {
          while (!it->second.empty() && it->second.front() <= old) {
            it->second.pop_front();
          }
          it = it->second.empty() ? hits_.erase(it) : std::next(it);
        }
      }
      for (const auto & bc : candidates) {
        if (confirmed(bc.second)) {
          scan->ranges[bc.first] = low[bc.first];
          ++n_confirmed_;
        }
      }
      n_candidates_ += candidates.size();
    }
    pub_->publish(std::move(scan));
    ++clouds_;
    compute_s_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  }

  void stats()
  {
    if (clouds_ == 0) {
      RCLCPP_WARN(get_logger(), "low_scan: no clouds processed in the last period");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "low_scan: %zu clouds, %.1f low-only candidates and %.1f confirmed rays per scan, "
      "%zu cells tracked, %.2f ms/cloud", clouds_, static_cast<double>(n_candidates_) / clouds_,
      static_cast<double>(n_confirmed_) / clouds_, hits_.size(), 1000.0 * compute_s_ / clouds_);
    clouds_ = n_candidates_ = n_confirmed_ = 0;
    compute_s_ = 0.0;
  }

  std::string target_frame_;
  double low_min_, low_max_, tall_min_, tall_max_, tall_margin_, range_min_, range_max_;
  int tall_bins_, min_hits_, n_bins_;
  double min_span_;
  std::vector<double> box_;
  bool persist_;
  double cell_, window_, odom_timeout_;
  LaserScan proto_;

  tf2::BufferCore buffer_;
  bool have_tf_ = false;
  std::string tf_frame_;
  Eigen::Transform<float, 3, Eigen::Affine> tf_;

  bool have_odom_ = false;
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_yaw_ = 0.0;
  std::chrono::steady_clock::time_point odom_time_;

  std::map<Cell, std::deque<uint64_t>> hits_;     // cell -> scan indices that hit it
  std::map<uint64_t, double> hit_time_;           // scan index -> cloud stamp (s)
  uint64_t scan_index_ = 0;

  size_t clouds_ = 0, n_candidates_ = 0, n_confirmed_ = 0;
  double compute_s_ = 0.0;

  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LowScan>());
  rclcpp::shutdown();
  return 0;
}
