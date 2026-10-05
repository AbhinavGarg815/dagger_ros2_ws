// Unit test of StickyGoalChecker with the numbers of the live hunting case (report Q76).
#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "dagger_nav_plugins/sticky_goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using dagger_nav_plugins::StickyGoalChecker;
using geometry_msgs::msg::Pose;

static Pose pose(double x, double y, double yaw)
{
  Pose p; p.position.x = x; p.position.y = y;
  p.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return p;
}

class Fixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_goal_checker");
    gc.initialize(node, "general_goal_checker", nullptr);
  }
  double tol() {Pose p; geometry_msgs::msg::Twist v; gc.getTolerances(p, v); return p.position.x;}
  bool reached(const Pose & robot, const Pose & goal) {return gc.isGoalReached(robot, goal, geometry_msgs::msg::Twist());}
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  StickyGoalChecker gc;
};

TEST_F(Fixture, LiveHuntingCaseCompletes)
{
  const Pose goal = pose(-2.02, 0.20, 43.0 * M_PI / 180.0);
  // approaching from 1 m: not reached, normal tolerance
  EXPECT_FALSE(reached(pose(-1.02, 0.20, -137.0 * M_PI / 180.0), goal));
  EXPECT_DOUBLE_EQ(tol(), 0.4);
  EXPECT_FALSE(gc.sticky());
  // arrives 0.36 m out, 180 deg to turn: position reached, heading not -> sticky, 0.8 m
  EXPECT_FALSE(reached(pose(-1.66, 0.20, -137.0 * M_PI / 180.0), goal));
  EXPECT_TRUE(gc.sticky());
  EXPECT_DOUBLE_EQ(tol(), 0.8);
  // a replan of the same goal (path end a few cm off) calls reset(): state kept
  gc.reset();
  const Pose goal_replanned = pose(-2.05, 0.18, 43.0 * M_PI / 180.0);
  // half way through the turn the pivot offset pushed base_footprint to 0.44 m: still turning
  EXPECT_FALSE(reached(pose(-1.62, 0.25, -47.0 * M_PI / 180.0), goal_replanned));
  EXPECT_TRUE(gc.sticky());
  EXPECT_DOUBLE_EQ(tol(), 0.8);
  // heading reached 0.5 m out (would have been "outside" with a fixed 0.4 m): success
  EXPECT_TRUE(reached(pose(-1.55, 0.30, 40.0 * M_PI / 180.0), goal_replanned));
}

TEST_F(Fixture, NewGoalClearsTheStickyState)
{
  const Pose goal = pose(0.0, 0.0, 0.0);
  EXPECT_FALSE(reached(pose(0.3, 0.0, M_PI), goal));
  EXPECT_TRUE(gc.sticky());
  // a new goal 3 m away
  EXPECT_FALSE(reached(pose(0.3, 0.0, M_PI), pose(3.3, 0.0, 0.0)));
  EXPECT_FALSE(gc.sticky());
  EXPECT_DOUBLE_EQ(tol(), 0.4);
  // same position, a different heading (> same_goal_yaw) is a new goal too
  EXPECT_FALSE(reached(pose(0.3, 0.0, M_PI), goal));
  EXPECT_TRUE(gc.sticky());
  EXPECT_FALSE(reached(pose(0.3, 0.0, M_PI), pose(0.0, 0.0, 0.5)));
  EXPECT_TRUE(gc.sticky());                    // re-armed for the new goal: 0.3 m is inside 0.4
  EXPECT_FALSE(reached(pose(0.6, 0.0, M_PI), pose(0.0, 0.0, 0.5)));
  EXPECT_TRUE(gc.sticky());                    // ...and 0.6 m inside its sticky 0.8 m
}

TEST_F(Fixture, DriftingBeyondTheStickyToleranceReArms)
{
  const Pose goal = pose(0.0, 0.0, 0.0);
  EXPECT_FALSE(reached(pose(0.35, 0.0, M_PI), goal));
  EXPECT_TRUE(gc.sticky());
  EXPECT_FALSE(reached(pose(0.9, 0.0, 0.05), goal));   // 0.9 m > 0.8 m even with the heading right
  EXPECT_FALSE(gc.sticky());
  EXPECT_DOUBLE_EQ(tol(), 0.4);
  EXPECT_FALSE(reached(pose(0.5, 0.0, 0.05), goal));   // back at 0.5 m: outside the normal 0.4 m
  EXPECT_TRUE(reached(pose(0.3, 0.0, 0.05), goal));    // inside 0.4 m with the heading: reached
}

TEST_F(Fixture, BehavesLikeSimpleGoalCheckerOtherwise)
{
  const Pose goal = pose(1.0, 1.0, 0.0);
  EXPECT_TRUE(reached(pose(1.1, 1.1, 0.1), goal));     // 0.14 m, 0.1 rad < 0.2
  EXPECT_FALSE(reached(pose(1.1, 1.1, 0.3), pose(5.0, 5.0, 0.3)));
  EXPECT_FALSE(reached(pose(5.0, 5.3, 0.6), pose(5.0, 5.0, 0.3)));  // 0.3 m inside, 0.3 rad off
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
