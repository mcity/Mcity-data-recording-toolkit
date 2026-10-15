#!/bin/bash
# Single-camera recorder: one rosbag2 process capturing the selected camera plus
# lidar, INS and tf into a single mcap bag.
#
# One writer comfortably handles a single camera stream. (The multi-camera
# variant splits across processes because a lone writer saturates a core around
# 30 Hz -- ~160% CPU, dropping frames -- once several image topics pile on.)
#
# The bag keeps the normal chunked mcap profile so it opens in Foxglove, unlike
# --storage-preset-profile fastwrite, which drops the chunk index.
#
# Usage: ./record-rtklidarcam.sh --cam <1-6> <run_name>
#
# Creates: <base_dir>/<run_name>/
#
# Ctrl-C stops the recorder cleanly (SIGINT lets mcap finalize its index).

set -u

usage() {
    echo "Usage: $0 --cam <1-6> <run_name>"
    echo
    echo "  --cam N     Which arenacam to record (1-6)"
    echo "  run_name    Name of the output bag directory"
}

cam=""
run_name=""

while [ $# -gt 0 ]; do
    case "$1" in
        --cam)
            if [ $# -lt 2 ]; then
                echo "--cam requires a value (1-6)."
                exit 1
            fi
            cam="$2"
            shift 2
            ;;
        --cam=*)
            cam="${1#*=}"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            if [ -n "$run_name" ]; then
                echo "Unexpected extra argument: $1"
                usage
                exit 1
            fi
            run_name="$1"
            shift
            ;;
    esac
done

if [ -z "$cam" ] || [ -z "$run_name" ]; then
    usage
    exit 1
fi

case "$cam" in
    [1-6]) ;;
    *)
        echo "--cam must be an integer from 1 to 6 (got '$cam')."
        exit 1
        ;;
esac

# Base directory for the data, depending on which drive you're saving to.
# SENSOR DRIVE
#base_dir="/media/mcity/SENSOR_DATA_B/june4-2026"
base_dir="/media/mcity/New Volume/july10-2026/calibration"
# MAIN OS DRIVE
#base_dir="/home/mcity/mcity-engineering/xujie/data-capture/may8-2026"

out_dir="$base_dir/$run_name"

# rosbag2 RAM cache (bounded so it can't OOM).
cam_cache=4000000000

# Record /ins/* reliably -- the OXTS publisher is reliable but rosbag2 otherwise
# subscribes best-effort and drops INS messages under load.
qos_overrides="$(dirname "$0")/ins_reliable_qos.yaml"

topics=(
  "/arenacam${cam}/images"
  "/arenacam${cam}/camera_info"
  /rslidar_points
  /ins/imu
  /ins/nav_sat_fix
  /tf
  /tf_static
)

if [ ! -f "$qos_overrides" ]; then
    echo "QoS overrides file not found: $qos_overrides"
    exit 1
fi

if [ -e "$out_dir" ]; then
    echo "Output already exists at $out_dir -- pick a new <run_name>."
    exit 1
fi

# Let rosbag2 create $out_dir itself; it refuses to write into an existing dir.
mkdir -p "$base_dir"

ros2 bag record -s mcap \
  --max-cache-size "$cam_cache" \
  --qos-profile-overrides-path "$qos_overrides" \
  -o "$out_dir" \
  "${topics[@]}" &
pid=$!

trap 'echo; echo "Stopping recorder..."; kill -INT "$pid" 2>/dev/null' INT TERM

echo "Recording (Ctrl-C to stop):"
echo "  camera: /arenacam${cam}/images"
echo "  output: $out_dir  (pid $pid)"
echo

# The first wait returns as soon as the trap fires, so keep waiting until the
# recorder has actually exited -- it needs a moment to write the mcap index.
while kill -0 "$pid" 2>/dev/null; do
    wait "$pid" 2>/dev/null
done
