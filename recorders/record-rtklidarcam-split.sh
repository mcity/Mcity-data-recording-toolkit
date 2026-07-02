#!/bin/bash

# Split recorder: records the OXTS INS in its OWN rosbag2 process, separate from
# the heavy cameras+lidar bag, so the camera recording load can't congest the
# pipeline and drop the 100 Hz INS. The two bags share the RTK GPS timebase
# (header.stamp) and align downstream. See SYNC_RECORDING.md.
#
# Usage: ./record-rtklidarcam-split.sh <run_name>
#
# Creates:
#   <base_dir>/<run_name>/ins/         -> /ins/imu, /ins/nav_sat_fix  (reliable)
#   <base_dir>/<run_name>/cam_lidar/   -> cameras + lidar + tf  (everything else)
#
# Ctrl-C stops both recorders cleanly.

# Check if a command line argument is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <run_name>"
    exit 1
fi

run_name="$1"

# Path to the base directory where the data will be saved, depending on which drive you will be saving to
# SENSOR DRIVE
#base_dir="/media/mcity/SENSOR_DATA_B/june4-2026"
base_dir="/media/mcity/New Volume/june29-2026/track_recordings"

# MAIN OS DRIVE
#base_dir="/home/mcity/mcity-engineering/xujie/data-capture/may8-2026"

# EXTERNAL USB DRIVE
#base_dir="/media/mcity/SANDISK/mache-data-capture/feb26-2026"

out_dir="$base_dir/$run_name"

# rosbag2 RAM cache for the heavy cam+lidar bag (~10 s of slack at 30 Hz full-res;
# bounded so it can't OOM the PC). The INS bag is tiny and uses the default.
cam_cache=4000000000

# Record /ins/* reliably -- the OXTS publisher is reliable but rosbag2 otherwise
# subscribes best-effort and drops INS messages under load.
qos_overrides="$(dirname "$0")/ins_reliable_qos.yaml"

if [ -e "$out_dir/ins" ] || [ -e "$out_dir/cam_lidar" ]; then
    echo "Output already exists under $out_dir -- pick a new <run_name>."
    exit 1
fi
mkdir -p "$out_dir"

# --- cam + lidar (everything except the INS) ---
ros2 bag record -s mcap \
  --max-cache-size "$cam_cache" \
  /arenacam1/images \
  /arenacam2/images \
  /arenacam3/images \
  /arenacam4/images \
  /arenacam5/images \
  /arenacam6/images \
  /rslidar_points \
  /tf \
  /tf_static \
  -o "$out_dir/cam_lidar" &
cam_pid=$!

# --- INS only (its own lightweight process, reliable) ---
ros2 bag record -s mcap \
  --qos-profile-overrides-path "$qos_overrides" \
  /ins/imu \
  /ins/nav_sat_fix \
  -o "$out_dir/ins" &
ins_pid=$!

# Stop both recorders cleanly on Ctrl-C (SIGINT lets rosbag2 finalize the mcap).
trap 'echo; echo "Stopping recorders..."; kill -INT "$cam_pid" "$ins_pid" 2>/dev/null' INT TERM

echo "Recording (Ctrl-C to stop both):"
echo "  cam+lidar -> $out_dir/cam_lidar   (pid $cam_pid)"
echo "  ins       -> $out_dir/ins         (pid $ins_pid)"

wait
