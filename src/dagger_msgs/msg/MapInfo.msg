# One saved SLAM map, as the map manager offers it (dagger_bringup/map_manager).
string name
float32 width_m
float32 height_m
# Tied to the Earth (<name>_geo.yaml): the picture's corners as lat/lon, in this
# order: top-left, top-right, bottom-right, bottom-left. Empty when not placed.
bool placed
float64[] corner_lat
float64[] corner_lon
# How far map and GPS disagreed when it was tied (rms, metres); 0 when not placed.
float32 placed_error_m
# The picture: a PNG, grey 0 = wall, 255 = free, transparent = never seen.
string picture
uint32 picture_width_px
uint32 picture_height_px
# Changes whenever the picture does (its file time, seconds).
uint64 picture_version
# When the map was saved (wall clock, ms since the Unix epoch).
uint64 created_ms
# When placed: the tie itself (<name>_geo.yaml), so lat/lon <-> map points can be
# converted exactly as the picture was placed: [datum_lat, datum_lon, yaw_rad, tx, ty]
# with p_enu = R(yaw) * p_map + t. Empty when not placed.
float64[] geo
