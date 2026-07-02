#!/bin/bash

# Split recorder: records as THREE rosbag2 processes so the writer CPU is spread
# across cores. A single writer saturates a core around 30 Hz (~160% CPU,
# dropping frames); splitting keeps each writer well under one core so nothing
# drops.
#
#   camA          -> cam1,cam2,cam3
#   camB          -> cam4,cam5,cam6
#   lidar_ins_tf  -> /rslidar_points, /ins/imu, /ins/nav_sat_fix, /tf, /tf_static  (INS reliable)
#
# The two camera groups are balanced equally (3 each, no extra topics) and the
# sensors get their own process -- an earlier 2-way split lost frames on the group
# that also carried the sensors, because that writer was heavier than the other.
#
# All three bags keep the normal chunked mcap profile, so each opens in Foxglove
# (unlike --storage-preset-profile fastwrite, which drops the chunk index). Merge
# them with merge-split-cams.sh for a single combined bag.
#
# Usage: ./record-rtklidarcam-split-cams.sh <run_name>
#
# Creates:
#   <base_dir>/<run_name>/camA/
#   <base_dir>/<run_name>/camB/
#   <base_dir>/<run_name>/lidar_ins_tf/
#
# Ctrl-C stops all three recorders cleanly (SIGINT lets mcap finalize its index).

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

# rosbag2 RAM cache per recorder (bounded so it can't OOM).
cam_cache=4000000000

# Record /ins/* reliably -- the OXTS publisher is reliable but rosbag2 otherwise
# subscribes best-effort and drops INS messages under load. (lidar_ins_tf carries INS.)
qos_overrides="$(dirname "$0")/ins_reliable_qos.yaml"

if [ -e "$out_dir/camA" ] || [ -e "$out_dir/camB" ] || [ -e "$out_dir/lidar_ins_tf" ]; then
    echo "Output already exists under $out_dir -- pick a new <run_name>."
    exit 1
fi
mkdir -p "$out_dir"

# --- Group A: cam1-3 ---
ros2 bag record -s mcap \
  --max-cache-size "$cam_cache" \
  /arenacam1/images \
  /arenacam2/images \
  /arenacam3/images \
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

# --- Sensors: lidar + INS + tf ---
ros2 bag record -s mcap \
  --max-cache-size "$cam_cache" \
  --qos-profile-overrides-path "$qos_overrides" \
  /rslidar_points \
  /ins/imu \
  /ins/nav_sat_fix \
  /tf \
  /tf_static \
  -o "$out_dir/lidar_ins_tf" &
s_pid=$!

# Stop all three recorders cleanly on Ctrl-C.
trap 'echo; echo "Stopping recorders..."; kill -INT "$a_pid" "$b_pid" "$s_pid" 2>/dev/null' INT TERM

echo "Recording (Ctrl-C to stop all):"
echo "  camA (cam1-3)   -> $out_dir/camA           (pid $a_pid)"
echo "  camB (cam4-6)   -> $out_dir/camB           (pid $b_pid)"
echo "  lidar_ins_tf    -> $out_dir/lidar_ins_tf   (pid $s_pid)"
echo
echo "Combine all three into one bag with:  ./merge-split-cams.sh \"$out_dir\""

wait
