# All ArUco markers the marker tracker holds, for ONE camera frame.

std_msgs/Header header          # stamp + frame_id of the camera frame
uint32 image_width
uint32 image_height
int32 image_rotation_deg        # how the camera is mounted; rotate by this to show it upright
string odom_frame
int32 id_base                   # MarkerTrack.id = id_base + marker_id

int32 selected_id               # -1 = no marker selected (a PERSON may be selected instead)
string target_state             # NONE | TRACKING | LOST | SEARCHING
MarkerTrack[] tracks
