# lidar_odom (dagger_perception): one message per LiDAR scan, accepted or not.
std_msgs/Header header       # the scan's stamp, frame base_footprint

float32 cpu_ms               # processing time of this scan (wall, ms)
float32 dt                   # time since the previous accepted scan (s); > 0.15 = scans dropped
uint32 points                # after the body box / range filter
uint32 source_points         # after the voxel downsampling (what is registered)
float32 inlier_ratio         # inliers / source points
uint8 iterations
bool converged
bool accepted                # false: rejected (too few points, few inliers, a jump); no odometry published
string reject_reason

# Translational information of the registration (Schur complement of the 6x6 GICP Hessian,
# rotation marginalised). A small smallest eigenvalue = a direction the scan cannot pin down
# (a corridor's axis, open ground); that direction's twist covariance is inflated.
float32 info_min
float32 info_max
float32[3] weak_axis         # unit eigenvector of info_min in base_footprint
bool degenerate              # info_min < degenerate_info

uint32 rejected_total
uint32 resets_total
