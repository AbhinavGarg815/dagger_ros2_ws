#include "dagger_nav_plugins/sticky_goal_checker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dagger_nav_plugins
{

void StickyGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("StickyGoalChecker: unable to lock the parent node");
  }
  logger_ = node->get_logger();
  const auto p = [&](const std::string & name, double def, double & out) {
      nav2_util::declare_parameter_if_not_declared(
        node, plugin_name + "." + name, rclcpp::ParameterValue(def));
      node->get_parameter(plugin_name + "." + name, out);
    };
  p("xy_goal_tolerance", 0.4, xy_tol_);
  p("xy_goal_tolerance_sticky", 0.8, xy_sticky_tol_);
  p("yaw_goal_tolerance", 0.2, yaw_tol_);
  p("same_goal_distance", 0.25, same_goal_distance_);
  p("same_goal_yaw", 0.1, same_goal_yaw_);
  xy_sticky_tol_ = std::max(xy_sticky_tol_, xy_tol_);
  RCLCPP_INFO(
    logger_, "%s: StickyGoalChecker xy %.2f m (%.2f m once reached, until the heading is), "
    "yaw %.2f rad", plugin_name_.c_str(), xy_tol_, xy_sticky_tol_, yaw_tol_);
}

void StickyGoalChecker::reset()
{
  // Deliberately keeps the state: replans of the same goal may call reset(); a new goal is
  // recognised in isGoalReached() from the goal pose.
}

bool StickyGoalChecker::sameGoal(const geometry_msgs::msg::Pose & goal) const
{
  const double d = std::hypot(
    goal.position.x - sticky_goal_.position.x, goal.position.y - sticky_goal_.position.y);
  const double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(goal.orientation), tf2::getYaw(sticky_goal_.orientation));
  return d <= same_goal_distance_ && std::fabs(dyaw) <= same_goal_yaw_;
}

bool StickyGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  const double d = std::hypot(
    query_pose.position.x - goal_pose.position.x, query_pose.position.y - goal_pose.position.y);
  if (sticky_ && !sameGoal(goal_pose)) {
    sticky_ = false;  // a new goal
  }
  if (!sticky_) {
    if (d > xy_tol_) {
      return false;
    }
    sticky_ = true;
    sticky_goal_ = goal_pose;
    RCLCPP_INFO(
      logger_, "%s: goal position reached (%.2f m) -- xy tolerance %.2f m until the heading is",
      plugin_name_.c_str(), d, xy_sticky_tol_);
  } else if (d > xy_sticky_tol_) {
    sticky_ = false;
    RCLCPP_WARN(
      logger_, "%s: drifted %.2f m from the goal while turning -- approaching it again",
      plugin_name_.c_str(), d);
    return false;
  }
  const double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation), tf2::getYaw(goal_pose.orientation));
  return std::fabs(dyaw) < yaw_tol_;
}

bool StickyGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance, geometry_msgs::msg::Twist & vel_tolerance)
{
  const double invalid = std::numeric_limits<double>::lowest();
  const double xy = sticky_ ? xy_sticky_tol_ : xy_tol_;
  pose_tolerance.position.x = xy;
  pose_tolerance.position.y = xy;
  pose_tolerance.position.z = invalid;
  pose_tolerance.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw_tol_);
  vel_tolerance.linear.x = vel_tolerance.linear.y = vel_tolerance.linear.z = invalid;
  vel_tolerance.angular.x = vel_tolerance.angular.y = vel_tolerance.angular.z = invalid;
  return true;
}

}  // namespace dagger_nav_plugins

PLUGINLIB_EXPORT_CLASS(dagger_nav_plugins::StickyGoalChecker, nav2_core::GoalChecker)
