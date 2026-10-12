#!/usr/bin/env bash
# Install (or update) the Dagger systemd units. NEEDS SUDO.
#   sudo bash tools/systemd/install.sh            # install + enable at boot (does not start)
#   sudo bash tools/systemd/install.sh --remove   # stop, disable, remove
# Then: sudo systemctl start dagger.target ; journalctl -fu dagger-gateway
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
UNITS="dagger.target dagger-drive.service dagger-nav.service dagger-follow.service dagger-gateway.service"
if [ "${1:-}" = "--remove" ]; then
    systemctl disable --now $UNITS || true
    for u in $UNITS; do rm -f "/etc/systemd/system/$u"; done
    systemctl daemon-reload
    echo "removed"
    exit 0
fi
for u in $UNITS; do install -m 644 "$HERE/$u" /etc/systemd/system/; done
systemctl daemon-reload
systemctl enable $UNITS
echo "installed and enabled at boot (not started). Start now: sudo systemctl start dagger.target"
