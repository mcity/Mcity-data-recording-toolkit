#!/usr/bin/env python3
"""
Check synchronization of the six LUCID cameras in a recorded bag.

Compares the sensor_msgs/Image header.stamp (the camera PTP capture time, in the
RTK GPS timebase) across all six /arenacamN/images topics and reports the
cross-camera spread per shot. With PTP + scheduled action-command triggering the
spread should be tiny (microseconds to sub-millisecond); a millisecond-or-larger
spread means a camera is not firing on the shared trigger or PTP is not locked.

Frames are matched ACROSS cameras by timestamp (nearest within a tolerance), NOT
by index. This is important: the cameras usually have slightly different message
counts (a few boundary frames at record start/stop, or an occasional mid-stream
drop), so index i of one camera is not the same shot as index i of another. An
index-based comparison would report a bogus ~one-frame-period offset; matching by
timestamp aligns the actual shots and reports the true sub-ms spread, and also
tells you how many shots are complete vs. which cameras missed which triggers.

NOTE: header.stamp is RTK GPS time, not UTC/Unix, so the absolute seconds value
will not look like wall-clock time. Only the cross-camera DIFFERENCE matters here.

Usage:
    source /opt/ros/humble/setup.bash
    python3 check-cam-sync.py "/media/mcity/New Volume/<bagdir>"  [storage_id]

storage_id defaults to "mcap" (matches the recorders' `ros2 bag record -s mcap`).
"""
import bisect
import statistics
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CAMS = [f"/arenacam{i}/images" for i in range(1, 7)]


def nearest(sorted_list, t):
    """Return the value in sorted_list closest to t (or None if empty)."""
    if not sorted_list:
        return None
    j = bisect.bisect_left(sorted_list, t)
    cand = []
    if j < len(sorted_list):
        cand.append(sorted_list[j])
    if j > 0:
        cand.append(sorted_list[j - 1])
    return min(cand, key=lambda x: abs(x - t))


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

    for c in CAMS:
        stamps[c].sort()

    counts = {c: len(stamps[c]) for c in CAMS}
    print("per-camera message counts:")
    for c in CAMS:
        print(f"  {c:24s} {counts[c]}")

    present = [c for c in CAMS if counts[c] > 0]
    if len(present) < 2:
        print("\nNeed at least two camera topics with messages to compare.")
        return

    # Use the camera with the most frames as the reference timeline, and estimate
    # the trigger period from its median inter-frame interval. Match tolerance is
    # ~half a period so each reference shot maps to the right frame on the other
    # cameras (not an adjacent one).
    ref = max(present, key=lambda c: counts[c])
    ref_stamps = stamps[ref]
    diffs = [ref_stamps[i + 1] - ref_stamps[i] for i in range(len(ref_stamps) - 1)]
    period = statistics.median(diffs) if diffs else 33_333_333
    tol = int(period * 0.45)

    print(f"\nreference timeline: {ref}  ({counts[ref]} frames, "
          f"period ~{period/1e6:.2f} ms, match tol ~{tol/1e6:.2f} ms)")

    complete = 0
    spreads = []
    missing = {c: 0 for c in present}
    sample_rows = []

    for t in ref_stamps:
        matched = {}
        for c in present:
            cand = nearest(stamps[c], t)
            if cand is not None and abs(cand - t) <= tol:
                matched[c] = cand
            else:
                missing[c] += 1
        if len(matched) == len(present):
            complete += 1
            vals = [matched[c] for c in present]
            spread = max(vals) - min(vals)
            spreads.append(spread)
            if len(sample_rows) < 10:
                base = matched[present[0]]
                offs = [(matched[c] - base) / 1000.0 for c in present]
                sample_rows.append((spread, offs))

    print(f"\nfirst {len(sample_rows)} COMPLETE shots "
          f"(offset from {present[0]}, microseconds):")
    print("  spread_us  " + "  ".join(f"{c.split('/')[1]:>9}" for c in present))
    for spread, offs in sample_rows:
        print(f"  {spread/1000.0:8.1f}  " + "  ".join(f"{o:9.1f}" for o in offs))

    print(f"\nshots on reference timeline:   {len(ref_stamps)}")
    print(f"complete shots (all cameras):  {complete}")
    print("per-camera triggers missed vs reference (boundary frames + real drops):")
    for c in present:
        print(f"  {c:24s} {missing[c]}")
    if spreads:
        print(f"\nmean cross-camera spread: {statistics.mean(spreads)/1000.0:.2f} us")
        print(f"max  cross-camera spread: {max(spreads)/1000.0:.2f} us  "
              f"({max(spreads)} ns)")
    print("\nInterpretation: spread over COMPLETE shots is the sync metric --"
          " < ~1 ms = well synchronized (ideally tens of us). 'triggers missed'"
          " counts boundary frames + any real drops; a handful is normal.")


if __name__ == "__main__":
    main()
