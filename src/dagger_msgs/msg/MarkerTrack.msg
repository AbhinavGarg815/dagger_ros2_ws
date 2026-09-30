# One tracked ArUco marker (dagger_bringup marker_tracker).

int32 id                        # track id = MarkerTracks.id_base + marker_id (never collides with people)
int32 marker_id
float32[8] corners              # last seen outline, native image pixels (see MarkerDetection)
bool visible                    # seen in THIS frame
bool confirmed                  # seen often enough to be real (only these can be selected)
float32 age_s                   # since first seen
float32 unseen_s                # since last seen (0 when visible)

float32 bearing_deg             # from base_footprint +x, +left
float32 range_m                 # from base_footprint; NaN if unknown
string range_source             # "lidar" (camera bearing + LiDAR range agreeing with the marker) | "marker" (from its size)
# Heading of whatever CARRIES the marker, assuming it is mounted on its BACK and
# faces the rover: the reverse of the marker's normal, in base_footprint (0 = the
# carrier points the same way as the rover). NaN if unknown.
float32 heading_deg
geometry_msgs/Point position    # in MarkerTracks.odom_frame; NaN if unknown
