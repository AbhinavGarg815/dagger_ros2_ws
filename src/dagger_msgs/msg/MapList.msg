# The operator's saved maps (latched on /maps/list), and the one drawn under missions.
MapInfo[] maps
string shown
# The map SLAM localization runs on (the shown one, when placed): map missions on it
# may run once /slam/localized is true. Empty: none.
string localizing
