#!/usr/bin/env bash
# Hold the rover stack until the clock is right (dagger-clock.service, at boot).
#
# Why: this Jetson's real-time clock has no backup battery, so at power-on the
# clock is restored to the last shutdown time and jumps to the real time once
# NTP reaches a time server -- which, with office Wi-Fi, happens a minute or two
# AFTER the stack has started. A jump of days breaks ROS nodes that work on
# timestamps: on 2026-10-13 and 2026-10-15 /scan stopped (pointcloud_to_laserscan),
# gps_heading_imu stuck "seeding", and the FCU estimate drifted -> Direction,
# Obstacle sensor and Position all NOT READY until the stack was restarted.
#
# First, the time from GPS (tools/gps_time.py, through the Cube -- before the stack
# opens its port): right anywhere under the sky, with no internet (2026-10-21: Wi-Fi
# came up after the stack had started and the clock jumped 44 h under it). A later
# time server then only nudges it.
# No GPS time within GPS_WAIT_S (indoors): with an internet route (a default route),
# wait up to WAIT_S for the first sync; without one there is no time server, so the
# clock cannot jump later either: start at once.
# The real fix is a coin cell for the RTC; then this returns immediately.
WAIT_S=85                  # with GPS_WAIT_S, inside the unit's TimeoutStartSec (180 s)
GPS_WAIT_S=90
synced() { [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = "yes" ]; }
if synced; then echo "clock already synchronized"; exit 0; fi
if python3 "$(dirname "$0")/../gps_time.py" --set --wait "$GPS_WAIT_S"; then exit 0; fi
if ! ip route show default | grep -q .; then
    echo "no internet route: no time server to wait for -- starting with the clock as it is"
    exit 0
fi
echo "waiting up to ${WAIT_S}s for the clock to synchronize"
for _ in $(seq "$WAIT_S"); do
    synced && { echo "clock synchronized: $(date)"; exit 0; }
    sleep 1
done
echo "clock not synchronized after ${WAIT_S}s -- starting anyway (it may still jump: restart dagger.target after it does)"
exit 0
