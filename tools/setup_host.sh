#!/usr/bin/env bash
# One-time host tuning for the Dagger LiDAR/DDS data path. NEEDS SUDO.
#   sudo bash tools/setup_host.sh
#
# WHY: the Helios-16P streams ~1.9 MB/s of UDP; the default 212 KB kernel receive
# buffer holds only ~0.1 s, so any CPU stall drops whole LiDAR frames (~8 M
# RcvbufErrors, 22 % of all UDP, observed 2026-09-14). Fast-DDS also falls back to
# UDP loopback for anything its shared-memory segment cannot take.
# Persisted in /etc/sysctl.d/60-dagger-net.conf so it survives reboots.
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
cat > /etc/sysctl.d/60-dagger-net.conf <<'CONF'
# Dagger: LiDAR UDP ingest + Fast-DDS loopback (2026-09-16)
net.core.rmem_max = 33554432
net.core.rmem_default = 33554432
net.core.wmem_max = 33554432
net.core.wmem_default = 16777216
net.ipv4.udp_mem = 262144 524288 1048576
CONF
sysctl -p /etc/sysctl.d/60-dagger-net.conf
echo "done -- restart T1/T2 so the drivers open sockets with the new sizes"
