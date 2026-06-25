#!/usr/bin/env python3
"""
Check synchronization of the six LUCID cameras in a recorded bag.

Compares the sensor_msgs/Image header.stamp (the camera PTP capture time, in the
RTK GPS timebase) across all six /arenacamN/images topics, frame by frame, and
reports the cross-camera spread per shot. With PTP + scheduled action-command
triggering the spread should be tiny (microseconds to sub-millisecond); a
millisecond-or-larger spread means a camera is not firing on the shared trigger
or PTP is not well locked.

NOTE: header.stamp is RTK GPS time, not UTC/Unix, so the absolute seconds value
will not look like wall-clock time. Only the cross-camera DIFFERENCE matters here.

Usage:
    source /opt/ros/humble/setup.bash
    python3 check-cam-sync.py "/media/mcity/New Volume/<bagdir>"  [storage_id]

storage_id defaults to "mcap" (matches the recorders' `ros2 bag record -s mcap`).
"""
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CAMS = [f"/arenacam{i}/images" for i in range(1, 7)]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    bag_uri = sys.argv[1]
    storage_id = sys.argv[2] if len(sys.argv) > 2 else "mcap"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    stamps = {c: [] for c in CAMS}

    while reader.has_next():
        topic, data, _bag_t = reader.read_next()
        if topic in CAMS:
            msg = deserialize_message(data, get_message(type_map[topic]))
            ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            stamps[topic].append(ns)

    counts = {c: len(stamps[c]) for c in CAMS}
    print("per-camera message counts:")
    for c in CAMS:
        print(f"  {c:24s} {counts[c]}")

    present = [c for c in CAMS if counts[c] > 0]
    if len(present) < 2:
        print("\nNeed at least two camera topics with messages to compare.")
        return

    n = min(counts[c] for c in present)
    if any(counts[c] != counts[present[0]] for c in present):
        print(f"\nWARNING: unequal counts; aligning by index over first {n} frames.")

    print(f"\nfirst 10 frames (offset from {present[0]}, microseconds):")
    header = "  idx  spread_us  " + "  ".join(f"{c.split('/')[1]:>9}" for c in present)
    print(header)

    max_spread = 0
    sum_spread = 0
    for i in range(n):
        vals = [stamps[c][i] for c in present]
        spread = max(vals) - min(vals)
        max_spread = max(max_spread, spread)
        sum_spread += spread
        if i < 10:
            offs = [(v - vals[0]) / 1000.0 for v in vals]
            print(f"  {i:>3}  {spread/1000.0:8.1f}  "
                  + "  ".join(f"{o:9.1f}" for o in offs))

    print(f"\nframes compared:          {n}")
    print(f"mean cross-camera spread: {sum_spread/n/1000.0:.2f} us")
    print(f"max  cross-camera spread: {max_spread/1000.0:.2f} us  ({max_spread} ns)")
    print("\nInterpretation: < ~1 ms = well synchronized (ideally tens of us);"
          " ms-or-larger means a camera is not on the shared trigger / PTP not locked.")


if __name__ == "__main__":
    main()
