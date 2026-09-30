# All ArUco markers of ONE processed camera frame.

std_msgs/Header header          # stamp + frame_id of that camera frame
uint32 image_width
uint32 image_height
string dictionary               # e.g. DICT_4X4_50
MarkerDetection[] markers
