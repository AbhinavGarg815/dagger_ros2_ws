# The RTK base station as the rover hears it (latched on /base/status, dagger_bringup/base_station).
bool heard                  # its heartbeats arrive (radio)
uint8 MODE_NO_FIX=1
uint8 MODE_SURVEYING=2
uint8 MODE_FIXED=3
uint8 mode
string site                 # FIXED on a saved site: its name; else empty
float32 survey_accuracy_m   # SURVEYING: how good so far
float32 survey_target_m     # ... the goal
uint32 survey_s             # ... for how long
bool survey_done            # a survey finished and is not kept as a site yet
string suggested_site_name  # for the save box: "Site 3"
