# One ArUco marker found in a camera frame (dagger_bringup person_detector).

int32 marker_id                 # the marker's id in MarkerDetections.dictionary
# Corners in the camera image as the DRIVER delivers it (native orientation,
# sub-pixel), in ArUco order: the marker's top-left, top-right, bottom-right,
# bottom-left as printed.
float32[8] corners              # x0, y0, x1, y1, x2, y2, x3, y3
