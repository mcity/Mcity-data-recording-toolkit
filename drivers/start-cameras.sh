#!/bin/bash

# Startup script: LUCID sync-trigger drivers (+ optional JPEG republisher).
#
# Launches lucid-drivers-sync-trigger.sh (gnome-terminal tabs, one per camera).
#
# JPEG republisher: OFF by default since 2026-11-06. It made 10 Hz JPEG copies
# (/arenacamN/images/compressed) for the stitcher on the old second PC (.27),
# because raw frames died crossing the network. On the single car PC the
# stitcher reads the raw /arenacamN/images locally, nothing reads the copies,
# and the republisher costs ~3 CPU cores. Turn it back on with
#   JPEG_REPUBLISHER=true bash start-cameras.sh
#
# Safe to run more than once: anything already running is left alone, so a
# second launch never starts duplicate drivers.
# Run at login via ~/.config/autostart/mcity-cameras.desktop (see install-autostart.sh).

##### CONFIG #####
camera_iface="enp10s0f1"   # interface the cameras/network are on
iface_timeout=60           # seconds to wait for it to come up
driver_settle=10           # seconds between drivers starting and the republisher
jpeg_republisher="${JPEG_REPUBLISHER:-false}"   # true: also start arenacam_jpeg_republisher.py
##################

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Same environment as an interactive shell (gnome-terminal tabs inherit it).
source /opt/ros/humble/setup.bash
source /home/mcity/Downloads/arena_camera_ros2/ros2_ws/install/setup.bash
# Wait for the camera network and the car (DDS) network.
for ((i = 0; i < iface_timeout; i++)); do
    ip -4 addr show "$camera_iface" 2>/dev/null | grep -q "inet " &&
        ip -4 -o addr show | grep -q ' 192\.168\.50\.' && break
    sleep 1
done
if ! ip -4 addr show "$camera_iface" 2>/dev/null | grep -q "inet "; then
    echo "WARNING: $camera_iface has no IPv4 address after ${iface_timeout}s, starting anyway"
fi
# DDS: CycloneDDS on the car NIC only (never campus WiFi), same config as the
# Autoware stack. Without a 192.168.50.x NIC the drivers would run isolated on
# loopback, invisible to the stitcher and Thor: refuse, loudly.
if ! ip -4 -o addr show | grep -q ' 192\.168\.50\.'; then
    logger -p user.err -t start-cameras "no 192.168.50.x NIC after ${iface_timeout}s: camera drivers NOT started"
    echo "ERROR: no 192.168.50.x NIC after ${iface_timeout}s, not starting camera drivers (rerun this script once the car NIC is up)" >&2
    exit 1
fi
source /home/mcity/Mcity/mcity-physical-ai-autostack/examples/lib_dds.sh

# Camera drivers
if pgrep -f "lib/arena_camera_node/start" > /dev/null; then
    echo "Camera drivers already running, not starting them again"
else
    bash "$script_dir/lucid-drivers-sync-trigger.sh"
    sleep "$driver_settle"
fi

# JPEG republisher (opt-in, see the header)
if [ "$jpeg_republisher" != "true" ]; then
    echo "JPEG republisher not started (JPEG_REPUBLISHER=true to enable)"
elif pgrep -f "arenacam_jpeg_republisher.py" > /dev/null; then
    echo "JPEG republisher already running, not starting it again"
else
    gnome-terminal --tab --title=jpeg-republisher -- bash -c "python3 $script_dir/arenacam_jpeg_republisher.py; /bin/bash"
fi
