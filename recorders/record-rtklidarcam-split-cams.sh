#!/bin/bash

# Split-camera recorder: records the six cameras as TWO rosbag2 processes so the
# writer CPU is spread across cores. A single writer saturates a core around
# 30 Hz (~160% CPU, dropping frames); two writers at ~half the data rate each
# stay well under one core and stop dropping.
#
# Both bags keep the normal chunked mcap profile, so each opens in Foxglove
# (unlike --storage-preset-profile fastwrite, which drops the chunk index). Merge
# the two afterwards with merge-split-cams.sh for a single combined bag.
#
# Usage: ./record-rtklidarcam-split-cams.sh <run_name>
#
# Creates:
#   <base_dir>/<run_name>/camA/   -> cam1,cam2,cam3 + ins + lidar + tf  (INS reliable)
#   <base_dir>/<run_name>/camB/   -> cam4,cam5,cam6
#
# Ctrl-C stops both recorders cleanly (SIGINT lets mcap finalize its index).

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

out_dir="$base_dir/$run_name"

# rosbag2 RAM cache per recorder (bounded so it can't OOM; each group is ~half the
# data rate, so this is generous slack).
cam_cache=4000000000

# Record /ins/* reliably -- the OXTS publisher is reliable but rosbag2 otherwise
# subscribes best-effort and drops INS messages under load. (Group A carries INS.)
qos_overrides="$(dirname "$0")/ins_reliable_qos.yaml"

if [ -e "$out_dir/camA" ] || [ -e "$out_dir/camB" ]; then
    echo "Output already exists under $out_dir -- pick a new <run_name>."
    exit 1
fi
mkdir -p "$out_dir"

# --- Group A: cam1-3 + INS + lidar + tf (a self-contained set for verification) ---
ros2 bag record -s mcap \
  --max-cache-size "$cam_cache" \
  --qos-profile-overrides-path "$qos_overrides" \
  /arenacam1/images \
  /arenacam2/images \
  /arenacam3/images \
  /ins/imu \
  /ins/nav_sat_fix \
  /rslidar_points \
  /tf \
  /tf_static \
  -o "$out_dir/camA" &
a_pid=$!

# --- Group B: cam4-6 ---
ros2 bag record -s mcap \
  --max-cache-size "$cam_cache" \
  /arenacam4/images \
  /arenacam5/images \
  /arenacam6/images \
  -o "$out_dir/camB" &
b_pid=$!

# Stop both recorders cleanly on Ctrl-C.
trap 'echo; echo "Stopping recorders..."; kill -INT "$a_pid" "$b_pid" 2>/dev/null' INT TERM

echo "Recording (Ctrl-C to stop both):"
echo "  camA (cam1-3 + ins + lidar + tf) -> $out_dir/camA   (pid $a_pid)"
echo "  camB (cam4-6)                    -> $out_dir/camB   (pid $b_pid)"
echo
echo "Verify a run in Foxglove by opening $out_dir/camA (has INS + lidar + tf + 3 cams)."
echo "Combine both into one bag with:  ./merge-split-cams.sh \"$out_dir\""

wait
