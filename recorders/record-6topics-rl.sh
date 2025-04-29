#!/bin/bash
# rl = Rate Limited

# Check if a command line argument is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <output_filename>"
    exit 1
fi

# Set the output filename based on the argument
output_filename="$1"

# Path to the base directory where the data will be saved
base_dir="/media/mcity/SENSOR_DATA/april8"
#base_dir="/home/mcity/mcity-engineering/vince/data-capture/march24"

# Run the ros2 bag record command with the provided output filename
#ros2 bag record -s mcap \
ros2 bag record \
  /cam1_rl/images \
  /cam2_rl/images \
  /cam3_rl/images \
  /cam4_rl/images \
  /cam5_rl/images \
  /cam6_rl/images \
  -o "$base_dir/$output_filename"


