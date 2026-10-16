# The map being made (latched on /maps/mapping, dagger_bringup/map_manager).
# active: SLAM is mapping (the operator drives); saving: the files are being written.
bool active
bool saving
# The map so far (name empty): size, its latest picture, corners once tied to the Earth.
MapInfo live
# A name not yet used, for the save box: "Map 3".
string suggested_name
