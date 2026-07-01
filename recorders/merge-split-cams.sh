#!/bin/bash

# Merge the camA + camB bags from record-rtklidarcam-split-cams.sh into a single
# Foxglove-viewable mcap bag. Uses `ros2 bag convert`, which ships with rosbag2 --
# no extra tools needed. Messages from both inputs are merged by timestamp.
#
# Usage: ./merge-split-cams.sh <run_dir>
#   where <run_dir> contains camA/ and camB/  ->  writes <run_dir>/merged/

if [ -z "$1" ]; then
    echo "Usage: $0 <run_dir>   (the directory containing camA/ and camB/)"
    exit 1
fi

run_dir="${1%/}"

if [ ! -d "$run_dir/camA" ] || [ ! -d "$run_dir/camB" ]; then
    echo "Expected $run_dir/camA and $run_dir/camB to exist."
    exit 1
fi
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

echo "Merging $run_dir/camA + $run_dir/camB -> $run_dir/merged ..."
ros2 bag convert -i "$run_dir/camA" -i "$run_dir/camB" -o "$cfg"
status=$?
rm -f "$cfg"

if [ $status -eq 0 ]; then
    echo "Done: $run_dir/merged  (open this one in Foxglove for all six cameras + sensors)"
else
    echo "ros2 bag convert failed (exit $status)."
    exit $status
fi
