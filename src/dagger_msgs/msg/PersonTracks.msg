# All people the tracker holds, for ONE camera frame.

std_msgs/Header header          # stamp + frame_id of the camera frame these boxes belong to
uint32 image_width
uint32 image_height
int32 image_rotation_deg        # how the camera is mounted; rotate by this to show it upright
string odom_frame

int32 selected_id               # -1 = nobody selected
string target_state             # NONE | TRACKING | HOLD_LIDAR | LOST
PersonTrack[] tracks
