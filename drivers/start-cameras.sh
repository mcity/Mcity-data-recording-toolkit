#!/bin/bash

# Startup script: LUCID sync-trigger drivers + JPEG republisher for Autoware.
#
# Launches lucid-drivers-sync-trigger.sh (gnome-terminal tabs, one per camera),
# then arenacam_jpeg_republisher.py in its own tab. The republisher publishes
# /arenacamN/images/compressed (10 Hz JPEG) for the stitcher on .27 so the raw
# 1.7 Gbit/s streams never leave this machine.
#
# Safe to run more than once: anything already running is left alone, so a
# second launch never starts duplicate drivers.
# Run at login via ~/.config/autostart/mcity-cameras.desktop (see install-autostart.sh).

##### CONFIG #####
camera_iface="enp10s0f1"   # interface the cameras/network are on
iface_timeout=60           # seconds to wait for it to come up
driver_settle=10           # seconds between drivers starting and the republisher
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

# JPEG republisher
if pgrep -f "arenacam_jpeg_republisher.py" > /dev/null; then
    echo "JPEG republisher already running, not starting it again"
else
    gnome-terminal --tab --title=jpeg-republisher -- bash -c "python3 $script_dir/arenacam_jpeg_republisher.py; /bin/bash"
fi
