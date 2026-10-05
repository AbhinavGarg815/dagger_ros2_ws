// StickyGoalChecker: Nav2 goal checker for the Dagger UGV (report Q76).
//
// Nav2's SimpleGoalChecker with one change: once the goal POSITION is reached (within
// xy_goal_tolerance), the xy tolerance it reports widens to xy_goal_tolerance_sticky until the
// goal heading is reached or the goal changes. RegulatedPurePursuit reads this tolerance every
// cycle (getTolerances) to decide between "rotate to the goal heading" and "drive to the goal".
//
// Why: the rover's tracks do not pivot exactly about base_footprint (0.07 m behind it in clean
// turns, more in stop-start ones), so a 180 deg final turn moves base_footprint 0.14-0.38 m.
// Arriving 0.36 m out with a fixed 0.4 m tolerance, the turn pushed it outside, RPP drove back
// toward the goal (turning away from the goal heading), re-entered, turned again... ~40 times
// (live 2026-10-05, bag detour_heading1_122047). With LiDAR odometry that drift is visible; the
// old FCU-based odometry read ~0 m/s in turns and hid it.
//
// reset() is ignored: the controller server may call it on every replan of the same goal. A new
// goal is detected from the goal pose itself (moved > same_goal_distance or turned >
// same_goal_yaw). Drifting beyond the sticky tolerance re-arms the normal approach.
#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace dagger_nav_plugins
{

class StickyGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

  bool sticky() const {return sticky_;}

protected:
  bool sameGoal(const geometry_msgs::msg::Pose & goal) const;

  double xy_tol_{0.4}, xy_sticky_tol_{0.8}, yaw_tol_{0.2};
  double same_goal_distance_{0.25}, same_goal_yaw_{0.1};
  bool sticky_{false};
  geometry_msgs::msg::Pose sticky_goal_;
  rclcpp::Logger logger_{rclcpp::get_logger("StickyGoalChecker")};
  std::string plugin_name_;
};

}  // namespace dagger_nav_plugins
