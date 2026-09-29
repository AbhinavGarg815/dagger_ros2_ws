# One tracked person (dagger_bringup person_tracker).

int32 id                        # stable while the tracker keeps this person; never reused

# Box in the camera image as the DRIVER delivers it (native orientation, pixels).
# The browser UI rotates for display using PersonTracks.image_rotation_deg.
float32 x1
float32 y1
float32 x2
float32 y2

float32 score                   # detector confidence of the detection matched this frame, 0 if coasting
bool visible                    # matched to a detection in THIS frame
uint32 age_frames               # frames since the track was born
uint32 missed_frames            # consecutive frames without a matching detection

float32 bearing_deg             # from base_footprint +x, +left (camera ray)
float32 range_m                 # LiDAR range from base_footprint; NaN if not ranged
geometry_msgs/Point position    # in PersonTracks.odom_frame when ranged; NaN otherwise
float32 appearance_sim          # re-ID similarity to the SELECTED person's appearance; NaN if n/a
