# All person detections of ONE processed camera frame, with appearance signatures.

std_msgs/Header header          # stamp + frame_id of that camera frame
uint32 image_width
uint32 image_height
PersonDetection[] detections
