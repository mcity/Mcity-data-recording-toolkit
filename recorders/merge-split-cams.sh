#!/bin/bash

# Merge the camA + camB + lidar_ins_tf bags from record-rtklidarcam-split-cams.sh
# into a single Foxglove-viewable mcap bag. Uses `ros2 bag convert`, which ships
# with rosbag2 -- no extra tools needed. Messages from all inputs are merged by
# timestamp.
#
# Usage: ./merge-split-cams.sh <run_dir>
#   where <run_dir> contains camA/, camB/, lidar_ins_tf/  ->  writes <run_dir>/merged/

if [ -z "$1" ]; then
    echo "Usage: $0 <run_dir>   (the directory containing camA/, camB/, lidar_ins_tf/)"
    exit 1
fi

run_dir="${1%/}"

for d in camA camB lidar_ins_tf; do
    if [ ! -d "$run_dir/$d" ]; then
        echo "Expected $run_dir/$d to exist."
        exit 1
    fi
done
if [ -e "$run_dir/merged" ]; then
    echo "$run_dir/merged already exists -- remove it first."
    exit 1
fi

# ros2 bag convert takes the output settings from a yaml file.
cfg="$(mktemp --suffix=.yaml)"
cat > "$cfg" <<EOF
output_bags:
  - uri: $run_dir/merged
    storage_id: mcap
    all: true
EOF

echo "Merging camA + camB + lidar_ins_tf -> $run_dir/merged ..."
ros2 bag convert -i "$run_dir/camA" -i "$run_dir/camB" -i "$run_dir/lidar_ins_tf" -o "$cfg"
status=$?
rm -f "$cfg"

if [ $status -eq 0 ]; then
    echo "Done: $run_dir/merged  (open this one in Foxglove for all six cameras + sensors)"
else
    echo "ros2 bag convert failed (exit $status)."
    exit $status
fi
