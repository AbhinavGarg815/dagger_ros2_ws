#!/usr/bin/env bash
# Apply a path-following tuning profile LIVE to the running controller_server.
#   bash tools/pf_tune.sh show     # print current values
#   bash tools/pf_tune.sh tuned    # long-lookahead profile (= nav2_dagger.yaml since 2026-09-14)
#   bash tools/pf_tune.sh base     # legacy short-lookahead values (d9e4c06) for A/B only
# Non-motion. Values only persist until T2 restarts; copy winners into nav2_dagger.yaml.
set -u
N=/controller_server
P=FollowPath
KEYS=(lookahead_dist min_lookahead_dist max_lookahead_dist use_velocity_scaled_lookahead_dist
      lookahead_time desired_linear_vel approach_velocity_scaling_dist
      min_approach_linear_velocity rotate_to_heading_angular_vel max_angular_accel
      regulated_linear_scaling_min_radius)

set_p() { printf '  %-55s ' "$1=$2"; ros2 param set $N "$1" "$2" 2>&1 | tail -1; }

case "${1:-show}" in
  show)
    for k in "${KEYS[@]}"; do printf '  %-40s ' "$k"; ros2 param get $N $P.$k 2>&1 | tail -1; done
    printf '  %-40s ' xy_goal_tolerance; ros2 param get $N general_goal_checker.xy_goal_tolerance | tail -1
    ;;
  tuned)
    # Order matters: RPP clamps lookahead_dist into [min,max], so widen bounds first.
    set_p $P.max_lookahead_dist 3.0
    set_p $P.min_lookahead_dist 1.5
    set_p $P.lookahead_dist 2.0
    set_p $P.use_velocity_scaled_lookahead_dist false
    set_p $P.desired_linear_vel 0.4
    set_p $P.approach_velocity_scaling_dist 1.0
    set_p $P.min_approach_linear_velocity 0.15
    set_p $P.rotate_to_heading_angular_vel 0.45
    set_p general_goal_checker.xy_goal_tolerance 0.5
    ;;
  base)
    set_p $P.lookahead_dist 0.6
    set_p $P.min_lookahead_dist 0.3
    set_p $P.max_lookahead_dist 0.9
    set_p $P.use_velocity_scaled_lookahead_dist false
    set_p $P.desired_linear_vel 0.5
    set_p $P.approach_velocity_scaling_dist 0.6
    set_p $P.min_approach_linear_velocity 0.05
    set_p $P.rotate_to_heading_angular_vel 0.3
    set_p general_goal_checker.xy_goal_tolerance 0.3
    ;;
  *) echo "usage: $0 show|tuned|base"; exit 1 ;;
esac
