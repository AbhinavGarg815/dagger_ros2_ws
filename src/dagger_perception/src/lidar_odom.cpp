// lidar_odom: LiDAR odometry from /rslidar_points, independent of GNSS (P20, report Q72-Q73).
//
// ⚠️ WORK IN PROGRESS (2026-10-03) -- NOT LAUNCHED ANYWHERE. Replaying detour_fox3_144415
// (parked start) it matches the Python small_gicp pipeline exactly for ~35 scans, then the
// constant-velocity guess creeps (-0.003 -> -0.014 -> -0.02 m/scan) and it diverges, while
// the Python reference (same parameters, float or double transform) stays within 1 mm.
// tools/test_lidar_odom.py: synthetic 6/8, replay fails. Next: find the C++/Python difference
// (debug:=true logs map size / guess / result / error per scan; compare with the Python run).
//
// /odometry/filtered took its velocity from the FCU's EK3 estimate, i.e. from GNSS velocity:
// next to walls and under canopies (h_acc > 3 m, dual-antenna heading invalid) it read the
// rover as stopped or backing while it drove 1.1 m forward (bag detour_fox3_144415).
//
// Method (chosen by benchmark on 6 drives, report Q73): small_gicp VGICP, scan-to-model.
//   cloud -> base_footprint (mount from /tf_static only, as safety_scan) -> body box and
//   range filter -> voxel downsampling (0.2 m) -> per-point covariances (10 neighbours)
//   -> VGICP against a Gaussian voxel map (0.5 m voxels, LRU: voxels unseen for 100 scans
//   are dropped) from a constant-velocity guess -> insert the scan into the map.
// No deskewing: with constant-velocity deskew LiDAR-only pipelines gain little, and the
// rover moves <= 0.6 m/s / 0.6 rad/s.
//
// Output: nav_msgs/Odometry in odom_frame (this node's own drifting world) -> base_footprint,
// pose and twist; the TWIST is what the EKF fuses. Its covariance comes from the
// registration's 6x6 information matrix (scaled, floored); along a direction the scan cannot
// pin down (a corridor's axis, open ground: smallest translational information below
// degenerate_info) the variance is inflated so the EKF leans on its other velocity source.
// A rejected scan (too few points / inliers, a jump) publishes NO odometry -- the EKF then
// runs on its other inputs, as before this node existed. After reset_after rejections in a row
// the map is rebuilt from the next scan (pose kept). dagger_msgs/LidarOdomStatus every scan.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <small_gicp/ann/gaussian_voxelmap.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>
#pragma GCC diagnostic pop

#include "dagger_msgs/msg/lidar_odom_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using dagger_msgs::msg::LidarOdomStatus;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

class LidarOdom : public rclcpp::Node
{
public:
  LidarOdom()
  : Node("lidar_odom")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "lidar_odom");
    box_ = declare_parameter<std::vector<double>>("body_box", {-0.90, 0.50, -0.60, 0.60});
    range_min_ = declare_parameter<double>("range_min", 0.3);
    range_max_ = declare_parameter<double>("range_max", 60.0);
    downsample_ = declare_parameter<double>("downsample", 0.2);
    neighbours_ = declare_parameter<int>("num_neighbors", 10);
    voxel_ = declare_parameter<double>("voxel_resolution", 0.5);
    lru_horizon_ = declare_parameter<int>("lru_horizon", 100);
    lru_clear_cycle_ = declare_parameter<int>("lru_clear_cycle", 10);
    max_corr_ = declare_parameter<double>("max_correspondence_distance", 1.0);
    max_iter_ = declare_parameter<int>("max_iterations", 20);
    threads_ = declare_parameter<int>("num_threads", 1);
    min_points_ = declare_parameter<int>("min_points", 500);
    min_inlier_ratio_ = declare_parameter<double>("min_inlier_ratio", 0.3);
    max_speed_ = declare_parameter<double>("max_speed", 2.0);          // m/s, a step faster = a jump
    max_turn_rate_ = declare_parameter<double>("max_turn_rate", 2.0);  // rad/s
    reset_after_ = declare_parameter<int>("reset_after", 10);
    cov_scale_ = declare_parameter<double>("cov_scale", 1.0);
    lin_floor_ = declare_parameter<double>("twist_linear_sigma_min", 0.02);    // m/s
    ang_floor_ = declare_parameter<double>("twist_angular_sigma_min", 0.005);  // rad/s
    degenerate_info_ = declare_parameter<double>("degenerate_info", 1000.0);
    degenerate_sigma_ = declare_parameter<double>("degenerate_sigma", 1.0);    // m/s
    const double stats_period = declare_parameter<double>("stats_period", 60.0);
    debug_ = declare_parameter<bool>("debug", false);
    if (box_.size() != 4 || !(box_[0] < box_[1]) || !(box_[2] < box_[3])) {
      throw std::invalid_argument("body_box must be [x_min, x_max, y_min, y_max]");
    }

    reg_.rejector.max_dist_sq = max_corr_ * max_corr_;
    reg_.reduction.num_threads = threads_;
    reg_.optimizer.max_iterations = max_iter_;
    reg_.criteria.rotation_eps = 0.1 * M_PI / 180.0;
    reg_.criteria.translation_eps = 1e-3;

    odom_pub_ = create_publisher<Odometry>("odometry", 10);
    status_pub_ = create_publisher<LidarOdomStatus>("status", 10);
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & t : msg->transforms) {
          buffer_.setTransform(t, "tf_static", true);
        }
        have_tf_ = false;
      });
    cloud_sub_ = create_subscription<PointCloud2>(
      "cloud_in", rclcpp::SensorDataQoS().keep_last(2),
      [this](PointCloud2::ConstSharedPtr msg) {on_cloud(msg);});
    stats_timer_ = create_wall_timer(
      std::chrono::duration<double>(stats_period), [this]() {stats();});

    RCLCPP_INFO(
      get_logger(), "lidar_odom: VGICP scan-to-model in %s, voxels %.2f m (LRU %d), "
      "downsample %.2f m, range %.1f-%.1f m, body box x[%.2f, %.2f] y[%.2f, %.2f], %d thread(s)",
      target_frame_.c_str(), voxel_, lru_horizon_, downsample_, range_min_, range_max_,
      box_[0], box_[1], box_[2], box_[3], threads_);
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
          get_logger(), *get_clock(), 5000, "lidar_odom: no odometry until /tf_static has "
          "%s <- %s (%s)", target_frame_.c_str(), frame.c_str(), e.what());
        return false;
      }
    }
    const auto & tr = t.transform.translation;
    const auto & q = t.transform.rotation;
    tf_ = Eigen::Translation3f(tr.x, tr.y, tr.z) * Eigen::Quaternion<float>(q.w, q.x, q.y, q.z);
    tf_frame_ = frame;
    have_tf_ = true;
    return true;
  }

  void reject(LidarOdomStatus & st, const std::string & why)
  {
    st.accepted = false;
    st.reject_reason = why;
    ++rejected_total_;
    ++window_rejected_;
    if (++consecutive_rejects_ >= reset_after_ && map_) {
      map_.reset();          // rebuilt from the next good scan, pose kept
      delta_.setIdentity();
      ++resets_total_;
      consecutive_rejects_ = 0;
      RCLCPP_ERROR(
        get_logger(), "lidar_odom: %d scans rejected in a row (last: %s) -- map reset",
        reset_after_, why.c_str());
    }
  }

  void on_cloud(const PointCloud2::ConstSharedPtr & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    if (!transform_for(msg->header.frame_id)) {
      return;
    }
    LidarOdomStatus st;
    st.header.stamp = msg->header.stamp;
    st.header.frame_id = target_frame_;
    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    if (have_last_ && stamp <= last_stamp_) {          // time went back (bag restart): start over
      map_.reset();
      delta_.setIdentity();
      have_last_ = false;
    }

    std::vector<Eigen::Vector4d> pts;
    pts.reserve(static_cast<size_t>(msg->width) * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    const double rmin2 = range_min_ * range_min_, rmax2 = range_max_ * range_max_;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
        continue;
      }
      const Eigen::Vector3f p = tf_ * Eigen::Vector3f(*ix, *iy, *iz);
      if (p.x() > box_[0] && p.x() < box_[1] && p.y() > box_[2] && p.y() < box_[3]) {
        continue;            // the rover itself (top frame, GNSS posts), as safety_scan
      }
      const double r2 = p.squaredNorm();
      if (r2 < rmin2 || r2 > rmax2) {
        continue;
      }
      pts.emplace_back(p.x(), p.y(), p.z(), 1.0);
    }
    st.points = pts.size();
    ++window_scans_;

    if (static_cast<int>(pts.size()) < min_points_) {
      reject(st, "too few points");
      finish(st, t0);
      return;
    }
    const auto raw = std::make_shared<small_gicp::PointCloud>(pts);
    const auto src = small_gicp::voxelgrid_sampling_omp(*raw, downsample_, threads_);
    const auto tree = std::make_shared<small_gicp::KdTree<small_gicp::PointCloud>>(
      src, small_gicp::KdTreeBuilderOMP(threads_));
    small_gicp::estimate_covariances_omp(*src, *tree, neighbours_, threads_);
    st.source_points = src->size();

    if (!map_) {             // first scan, or after a reset: the map starts here
      map_ = std::make_shared<small_gicp::GaussianVoxelMap>(voxel_);
      map_->lru_horizon = lru_horizon_;
      map_->lru_clear_cycle = lru_clear_cycle_;
      map_->insert(*src, T_);
      last_stamp_ = stamp;
      have_last_ = true;
      st.accepted = true;
      st.reject_reason = "map (re)started";
      finish(st, t0);
      return;
    }

    const Eigen::Isometry3d init = T_ * delta_;
    const auto result = reg_.align(*map_, *src, *map_, init);
    const Eigen::Isometry3d step = T_.inverse() * result.T_target_source;
    const double dt = stamp - last_stamp_;
    if (debug_) {
      RCLCPP_INFO(
        get_logger(), "DBG map %zu src %zu init (%.4f %.4f) res (%.4f %.4f) it %zu inl %zu err %.3f",
        map_->size(), src->size(), init.translation().x(), init.translation().y(),
        result.T_target_source.translation().x(), result.T_target_source.translation().y(),
        result.iterations, result.num_inliers, result.error);
    }
    st.dt = dt;
    st.iterations = std::min<size_t>(result.iterations, 255);
    st.converged = result.converged;
    st.inlier_ratio = src->empty() ? 0.0 : static_cast<double>(result.num_inliers) / src->size();
    const double step_rot = Eigen::AngleAxisd(step.rotation()).angle();

    // Translational information with rotation marginalised (Schur complement). small_gicp
    // orders the tangent space [rotation, translation], right perturbation (body frame).
    const Eigen::Matrix3d Hrr = result.H.topLeftCorner<3, 3>();
    const Eigen::Matrix3d Hrt = result.H.topRightCorner<3, 3>();
    const Eigen::Matrix3d Htt = result.H.bottomRightCorner<3, 3>();
    const Eigen::Matrix3d It = Htt - Hrt.transpose() * Hrr.ldlt().solve(Hrt);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(It);
    st.info_min = es.eigenvalues()(0);
    st.info_max = es.eigenvalues()(2);
    const Eigen::Vector3d weak = es.eigenvectors().col(0);
    st.weak_axis = {static_cast<float>(weak.x()), static_cast<float>(weak.y()), static_cast<float>(weak.z())};
    st.degenerate = st.info_min < degenerate_info_;

    const double step_dt = std::max(dt, 0.05);
    if (st.inlier_ratio < min_inlier_ratio_) {
      reject(st, "few inliers");
    } else if (step.translation().norm() > max_speed_ * step_dt + 0.05 ||
      step_rot > max_turn_rate_ * step_dt + 0.02)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "lidar_odom: jump rejected: %.3f m / %.2f deg in %.3f s "
        "(guess %.3f m / %.2f deg, inliers %.2f, %zu iterations)", step.translation().norm(),
        step_rot * 180.0 / M_PI, dt, (T_.inverse() * init).translation().norm(),
        Eigen::AngleAxisd((T_.inverse() * init).rotation()).angle() * 180.0 / M_PI,
        st.inlier_ratio, result.iterations);
      reject(st, "jump");
    } else {
      consecutive_rejects_ = 0;
      delta_ = step;
      T_ = result.T_target_source;
      map_->insert(*src, T_);
      last_stamp_ = stamp;
      st.accepted = true;
      window_degenerate_ += st.degenerate;
      window_info_min_ = std::min(window_info_min_, static_cast<double>(st.info_min));
      publish(msg->header.stamp, step, dt, result.H, weak, st.degenerate);
    }
    finish(st, t0);
  }

  void publish(
    const builtin_interfaces::msg::Time & stamp, const Eigen::Isometry3d & step, double dt,
    const Eigen::Matrix<double, 6, 6> & H, const Eigen::Vector3d & weak, bool degenerate)
  {
    Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = target_frame_;
    const Eigen::Quaterniond q(T_.rotation());
    o.pose.pose.position.x = T_.translation().x();
    o.pose.pose.position.y = T_.translation().y();
    o.pose.pose.position.z = T_.translation().z();
    o.pose.pose.orientation.w = q.w();
    o.pose.pose.orientation.x = q.x();
    o.pose.pose.orientation.y = q.y();
    o.pose.pose.orientation.z = q.z();
    // The pose drifts without bound and is not meant for fusion; nominal values.
    for (int i = 0; i < 3; ++i) {
      o.pose.covariance[i * 6 + i] = 0.01;
      o.pose.covariance[(i + 3) * 6 + i + 3] = 0.001;
    }
    if (dt > 0.0) {
      const Eigen::AngleAxisd aa(step.rotation());
      const Eigen::Vector3d v = step.translation() / dt;
      const Eigen::Vector3d w = aa.axis() * aa.angle() / dt;
      o.twist.twist.linear.x = v.x();
      o.twist.twist.linear.y = v.y();
      o.twist.twist.linear.z = v.z();
      o.twist.twist.angular.x = w.x();
      o.twist.twist.angular.y = w.y();
      o.twist.twist.angular.z = w.z();
      // Step covariance ~ cov_scale * H^-1 ([rot, trans]) -> twist ([lin, ang]) / dt^2.
      const Eigen::Matrix<double, 6, 6> C = cov_scale_ * H.ldlt().solve(
        Eigen::Matrix<double, 6, 6>::Identity()) / (dt * dt);
      Eigen::Matrix<double, 6, 6> Ct;
      Ct.topLeftCorner<3, 3>() = C.bottomRightCorner<3, 3>();
      Ct.bottomRightCorner<3, 3>() = C.topLeftCorner<3, 3>();
      Ct.topRightCorner<3, 3>() = C.bottomLeftCorner<3, 3>();
      Ct.bottomLeftCorner<3, 3>() = C.topRightCorner<3, 3>();
      Ct.topLeftCorner<3, 3>() += lin_floor_ * lin_floor_ * Eigen::Matrix3d::Identity();
      Ct.bottomRightCorner<3, 3>() += ang_floor_ * ang_floor_ * Eigen::Matrix3d::Identity();
      if (degenerate) {
        Ct.topLeftCorner<3, 3>() += degenerate_sigma_ * degenerate_sigma_ * weak * weak.transpose();
      }
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
          o.twist.covariance[r * 6 + c] = Ct(r, c);
        }
      }
    }
    odom_pub_->publish(o);
  }

  void finish(LidarOdomStatus & st, std::chrono::steady_clock::time_point t0)
  {
    st.cpu_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    st.rejected_total = rejected_total_;
    st.resets_total = resets_total_;
    window_ms_.push_back(st.cpu_ms);
    status_pub_->publish(st);
  }

  void stats()
  {
    if (window_scans_ == 0) {
      RCLCPP_WARN(get_logger(), "lidar_odom: no scans in the last period");
      return;
    }
    std::sort(window_ms_.begin(), window_ms_.end());
    const double mean = std::accumulate(window_ms_.begin(), window_ms_.end(), 0.0) / window_ms_.size();
    const double p95 = window_ms_[static_cast<size_t>(0.95 * (window_ms_.size() - 1))];
    RCLCPP_INFO(
      get_logger(), "lidar_odom: %u scans, %u rejected, %u degenerate, min info %.0f, "
      "%.1f / %.1f ms mean / p95, pose (%.2f, %.2f, %.2f), resets %u", window_scans_,
      window_rejected_, window_degenerate_, window_info_min_, mean, p95, T_.translation().x(),
      T_.translation().y(), T_.translation().z(), resets_total_);
    window_scans_ = window_rejected_ = window_degenerate_ = 0;
    window_info_min_ = std::numeric_limits<double>::infinity();
    window_ms_.clear();
  }

  std::string target_frame_, odom_frame_, tf_frame_;
  std::vector<double> box_;
  double range_min_, range_max_, downsample_, voxel_, max_corr_, min_inlier_ratio_;
  double max_speed_, max_turn_rate_, cov_scale_, lin_floor_, ang_floor_;
  double degenerate_info_, degenerate_sigma_;
  int neighbours_, lru_horizon_, lru_clear_cycle_, max_iter_, threads_, min_points_, reset_after_;

  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> reg_;
  std::shared_ptr<small_gicp::GaussianVoxelMap> map_;
  Eigen::Isometry3d T_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d delta_ = Eigen::Isometry3d::Identity();
  double last_stamp_ = 0.0;
  bool have_last_ = false;
  bool debug_ = false;
  int consecutive_rejects_ = 0;
  uint32_t rejected_total_ = 0, resets_total_ = 0;
  uint32_t window_scans_ = 0, window_rejected_ = 0, window_degenerate_ = 0;
  double window_info_min_ = std::numeric_limits<double>::infinity();
  std::vector<double> window_ms_;

  tf2::BufferCore buffer_;
  Eigen::Transform<float, 3, Eigen::Affine> tf_;
  bool have_tf_ = false;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<LidarOdomStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdom>());
  rclcpp::shutdown();
  return 0;
}
