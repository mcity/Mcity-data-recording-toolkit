#!/bin/bash

# Check if a command line argument is provided
if [ -z "$1" ]; then
    echo "Usage: $0 <output_filename>"
    exit 1
fi

# Set the output filename based on the argument
output_filename="$1"

# Path to the base directory where the data will be saved, depending on which drive you will be saving to
# SENSOR DRIVE
#base_dir="/media/mcity/SENSOR_DATA_B/june4-2026"
base_dir="/media/mcity/New Volume/june12-2026"

# MAIN OS DRIVE
#base_dir="/home/mcity/mcity-engineering/xujie/data-capture/may8-2026"

# EXTERNAL USB DRIVE
#base_dir="/media/mcity/SANDISK/mache-data-capture/feb26-2026"


# Run the ros2 bag record command with the provided output filename, and a 55 GB cache
ros2 bag record -s mcap \
  /arenacam3/images \
  /ins/imu \
  /ins/nav_sat_fix \
  /tf \
  /tf_static \
  -o "$base_dir/$output_filename" \
  --max-cache-size 55000000000

  #/oxts/imu \
  #/oxts/fix \
  #/rslidar_back_points \
  #/rslidar_front_points \
  #/rslidar_left_points \
  #/rslidar_right_points \

