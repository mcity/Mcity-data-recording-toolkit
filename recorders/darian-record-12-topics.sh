#!/bin/bash

# Check if a command line argument is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <output_filename>"
    exit 1
fi

# Set the output filename based on the argument
output_filename="$1"

# Path to the base directory where the data will be saved
#base_dir="/media/mcity/SENSOR_DATA_B/april30"
#base_dir="/home/mcity/mcity-engineering/vince/data-capture/"
base_dir="/media/mcity/SENSOR_DATA_B/darian/data-capture/nov_11_2025"

# Run the ros2 bag record command with the provided output filename
 # /arenacam2/images \
  # /arenacam3/images \
  # /arenacam4/images \
  # /arenacam5/images \
  # /arenacam6/images \
  # /arenacam6/images \
ros2 bag record -s mcap \
  /rslidar_back_points \
  /rslidar_front_points \
  /rslidar_left_points \
  /rslidar_right_points \
  /oxts/imu \
  /oxts/fix \
  /tf \
  -o "$base_dir/$output_filename" \
  --max-cache-size 55000000000


