#!/usr/bin/env python3
"""
Live (or from-bag) brightness / clipping check for the six LUCID cameras.

Made for tuning a STATIC exposure on the vehicle: launch
lucid-drivers-sync-trigger-static-exposure.sh, run this in another terminal,
and it prints, per camera, once a second:

    cam   frames   mean   p1   p50   p99   clip%  black%   verdict

  mean/p1/p50/p99  pixel levels on a 0-255 scale (raw Bayer, no debayer)
  clip%            fraction of pixels at >= 98% of full scale -> blown, unrecoverable
  black%           fraction of pixels at <= 3% of full scale  -> crushed, mostly noise
  verdict          what to do with exposure_time

The goal for "expose dark, lift in post" is clip% ~0 (well under 0.5%) with a
mean that is dark but not black (roughly 25-100 on the 0-255 scale). A bright,
washed-out image here with a static launch usually means ExposureAuto/GainAuto
are still on -- check each camera's "exposure state (STATIC)" log line.

Stats are computed on a subsampled grid of the raw sensor values (Bayer or mono,
8- or 16-bit), so they are cheap and encoding-agnostic. Colors are irrelevant
for exposure; what matters is whether any channel is clipping.

Usage:
    source /opt/ros/humble/setup.bash
    python3 check-exposure.py                      # live, all six cameras
    python3 check-exposure.py --topics /arenacam1/images
    python3 check-exposure.py --bag "<bagdir>"     # summarize a recording instead
    python3 check-exposure.py --bag "<bagdir>" --every 30 --max-frames 200

Live mode subscribes best-effort with depth 1, so it only ever looks at the
newest frame and does not back up the (reliable) publisher. Still: don't leave
it running during a real capture; every extra subscriber costs the recorder.
"""
import argparse
import sys
import time

import numpy as np

CAMS = [f"/arenacam{i}/images" for i in range(1, 7)]
SUBSAMPLE = 4          # look at every 4th row and column (1/16 of the pixels)
CLIP_FRAC = 0.98       # >= this fraction of full scale counts as clipped
BLACK_FRAC = 0.03      # <= this fraction of full scale counts as crushed


def image_to_array(msg):
    """sensor_msgs/Image -> 2-D numpy array of raw sensor values (first plane).

    Handles the driver's Bayer/mono 8- and 16-bit encodings. For 3-channel
    encodings (rgb8/bgr8) it returns the max over channels, which is what
    matters for clipping.
    """
    enc = msg.encoding.lower()
    if enc.endswith("16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    else:
        dtype = np.dtype(np.uint8)
    buf = np.frombuffer(msg.data, dtype=dtype)
    row_len = msg.step // dtype.itemsize
    if buf.size < msg.height * row_len:
        raise ValueError(
            f"short image data: {buf.size} values for {msg.height}x{row_len}")
    a = buf[: msg.height * row_len].reshape(msg.height, row_len)
    channels = 1
    for prefix, n in (("rgba", 4), ("bgra", 4), ("rgb", 3), ("bgr", 3)):
        if enc.startswith(prefix):
            channels = n
            break
    if channels > 1:
        a = a[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        a = a.max(axis=2)
    else:
        a = a[:, : msg.width]
    return a


def full_scale_for(arr):
    """255 for 8-bit; for 16-bit containers, 4095 if the data is right-justified
    12-bit (max seen <= 4095) else 65535."""
    if arr.dtype.itemsize == 1:
        return 255.0
    return 4095.0 if int(arr.max()) <= 4095 else 65535.0


def frame_stats(arr):
    """Return dict of brightness stats on a 0-255 scale plus clip/black %."""
    sub = arr[::SUBSAMPLE, ::SUBSAMPLE].astype(np.float32)
    fs = full_scale_for(arr)
    norm = sub / fs
    p1, p50, p99 = np.percentile(norm, [1, 50, 99])
    return {
        "mean": float(norm.mean() * 255.0),
        "p1": float(p1 * 255.0),
        "p50": float(p50 * 255.0),
        "p99": float(p99 * 255.0),
        "clip": float((norm >= CLIP_FRAC).mean() * 100.0),
        "black": float((norm <= BLACK_FRAC).mean() * 100.0),
        "full_scale": fs,
    }


def verdict(st):
    if st["clip"] > 2.0:
        return "CLIPPING hard -> lower exposure_time"
    if st["clip"] > 0.5:
        return "clipping -> lower exposure_time a bit"
    if st["mean"] > 140:
        return "bright: check ExposureAuto/GainAuto are Off; else lower exposure"
    if st["mean"] < 8 or st["black"] > 60:
        return "black -> raise exposure_time (then gain)"
    if st["mean"] < 25:
        return "very dark: OK for post-lift if noise acceptable, else raise"
    return "OK (dark-ish, no clipping)"


HEADER = (f"{'cam':10s} {'frames':>6s} {'mean':>6s} {'p1':>5s} {'p50':>5s} "
          f"{'p99':>5s} {'clip%':>6s} {'black%':>6s}  verdict")


def fmt_row(name, n, st):
    return (f"{name:10s} {n:6d} {st['mean']:6.1f} {st['p1']:5.0f} "
            f"{st['p50']:5.0f} {st['p99']:5.0f} {st['clip']:6.2f} "
            f"{st['black']:6.1f}  {verdict(st)}")


def run_live(topics, interval):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy)
    from sensor_msgs.msg import Image

    class Watcher(Node):
        def __init__(self):
            super().__init__("check_exposure")
            self.latest = {t: None for t in topics}
            self.counts = {t: 0 for t in topics}
            self.errors = {t: None for t in topics}
            qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
            for t in topics:
                self.create_subscription(
                    Image, t, lambda m, t=t: self.on_image(t, m), qos)
            self.create_timer(interval, self.report)

        def on_image(self, topic, msg):
            self.counts[topic] += 1
            try:
                self.latest[topic] = (frame_stats(image_to_array(msg)),
                                      msg.encoding)
                self.errors[topic] = None
            except Exception as e:  # keep going on a bad frame
                self.errors[topic] = str(e)

        def report(self):
            print(time.strftime("%H:%M:%S"))
            print(HEADER)
            for t in topics:
                name = t.split("/")[1] if t.count("/") >= 2 else t
                if self.errors[t]:
                    print(f"{name:10s} {self.counts[t]:6d}  error: {self.errors[t]}")
                elif self.latest[t] is None:
                    print(f"{name:10s} {self.counts[t]:6d}  (no frames yet)")
                else:
                    st, enc = self.latest[t]
                    print(fmt_row(name, self.counts[t], st) + f"  [{enc}]")
            print()

    rclpy.init()
    node = Watcher()
    print(f"watching {len(topics)} topic(s); Ctrl-C to stop\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def expand_bags(path):
    """A bag dir (has metadata.yaml) -> [path]; a split-recorder run folder
    -> its child bags (camA, camB, ...)."""
    import os
    if os.path.isfile(os.path.join(path, "metadata.yaml")):
        return [path]
    subs = sorted(d for d in (os.path.join(path, n) for n in os.listdir(path))
                  if os.path.isfile(os.path.join(d, "metadata.yaml")))
    if not subs:
        sys.exit(f"{path} is neither a bag (no metadata.yaml) nor a run folder containing bags")
    return subs


def run_bag(bag_uri, storage_id, topics, every, max_frames):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    present = []
    acc = {}
    for bag in expand_bags(bag_uri):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag, storage_id=storage_id),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                        output_serialization_format="cdr"),
        )
        type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
        here = [t for t in topics if t in type_map]
        if not here:
            continue
        present += [t for t in topics if t in here and t not in present]
        seen = {t: 0 for t in here}
        for t in here:
            acc.setdefault(t, [])
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic not in seen:
                continue
            seen[topic] += 1
            if (seen[topic] - 1) % every != 0:
                continue
            if max_frames and len(acc[topic]) >= max_frames:
                if all(len(acc[t]) >= max_frames for t in here):
                    break
                continue
            msg = deserialize_message(data, get_message(type_map[topic]))
            acc[topic].append(frame_stats(image_to_array(msg)))
    if not present:
        print(f"none of {topics} found under {bag_uri}")
        return

    print(f"bag: {bag_uri}  (every {every}th frame"
          f"{', max ' + str(max_frames) + ' sampled per camera' if max_frames else ''})")
    print(HEADER.replace("frames", "sampld"))
    for t in present:
        name = t.split("/")[1] if t.count("/") >= 2 else t
        rows = acc[t]
        if not rows:
            print(f"{name:10s}      0  (no frames)")
            continue
        agg = {k: float(np.mean([r[k] for r in rows]))
               for k in ("mean", "p1", "p50", "p99", "black")}
        # Clipping is judged on the WORST sampled frame: one blown frame is
        # one blown frame, averaging would hide it.
        agg["clip"] = float(max(r["clip"] for r in rows))
        print(fmt_row(name, len(rows), agg))
    print("\n(mean/p1/p50/p99/black% are averaged over sampled frames; clip% is"
          " the worst sampled frame)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topics", nargs="+", default=CAMS)
    ap.add_argument("--interval", type=float, default=1.0,
                    help="live report period, seconds (default 1.0)")
    ap.add_argument("--bag", help="rosbag2 directory, or a split-recorder run folder "
                                  "(camA/camB read together), to summarize instead of live")
    ap.add_argument("--storage", default="mcap", help="bag storage id (default mcap)")
    ap.add_argument("--every", type=int, default=10,
                    help="bag mode: sample every Nth frame per camera (default 10)")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="bag mode: stop after this many sampled frames per camera")
    args = ap.parse_args()

    if args.bag:
        run_bag(args.bag, args.storage, args.topics, max(1, args.every),
                args.max_frames)
    else:
        run_live(args.topics, args.interval)


if __name__ == "__main__":
    main()
