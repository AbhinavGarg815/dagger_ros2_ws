# One person detection in one camera frame (person_detector).

# Box in the image as the DRIVER delivers it (native orientation, pixels).
float32 x1
float32 y1
float32 x2
float32 y2
float32 score                   # detector confidence
float32[] embedding             # re-ID appearance signature (L2-normalised, 512-d); empty if not computed
