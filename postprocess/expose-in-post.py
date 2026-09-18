#!/usr/bin/env python3
"""
Lift deliberately under-exposed (static exposure) camera frames from a rosbag
into viewable images.

Companion to drivers/lucid-drivers-sync-trigger-static-exposure.sh: the
cameras are run open-loop and DARK so highlights never clip, and brightness is
restored here. Because the capture was static, ONE exposure adjustment per
camera is correct for the whole run, which is what this does.

Pipeline per frame:
  1. decode the sensor_msgs/Image (bayer_*8 / bayer_*16 / mono / rgb / bgr)
  2. debayer (OpenCV, pattern taken from the encoding string)
  3. undo the in-camera gamma (the launcher uses gamma=0.5) -> linear light
  4. multiply by 2**EV  (EV = stops of lift; fixed, or chosen automatically)
  5. clip, apply a display gamma (default 2.2) and write an 8-bit JPEG/PNG

EV selection:
  --ev 3.0        fixed: same lift for every camera and frame (fully deterministic)
  --auto          (default) per camera: sample frames from the start of the run
                  and pick the EV that puts the --auto-percentile (99.5th) of
                  linear luminance at --auto-target (0.90) -> bright but
                  highlights intact. The chosen EVs are printed; re-run with
                  --ev to pin them, or use --auto-global for one EV across all
                  six cameras (keeps cross-camera brightness comparable).
                  If the sun/sky covers more than ~0.5% of the frame the auto
                  EV will protect it and leave the rest dark; lower
                  --auto-percentile (e.g. 97) to let the sky clip instead.

Output: <out>/<camera>/<header.stamp ns>.jpg  (stamp = PTP/RTK capture time,
so frames from different cameras with the same stamp are the same shot.)

Usage:
    source /opt/ros/humble/setup.bash
    python3 expose-in-post.py "<bagdir>" -o /tmp/lifted                  # auto EV
    python3 expose-in-post.py "<bagdir>" -o /tmp/lifted --ev 3           # +3 stops
    python3 expose-in-post.py "<bagdir>" -o /tmp/lifted --every 30 --max-frames 20   # quick look
    python3 expose-in-post.py "<bagdir>" -o /tmp/lifted --camera-gamma 1.0   # if launched with gamma=1.0

Requires numpy + opencv (python3-opencv on the vehicle) and the ROS 2 python
packages (rosbag2_py, rclpy) for reading the bag.
"""
import argparse
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

CAMS = [f"/arenacam{i}/images" for i in range(1, 7)]

# ROS encoding -> OpenCV demosaic code. OpenCV names Bayer patterns by the
# 2x2 tile's second row, so the names look swapped vs ROS; this is the same
# mapping cv_bridge uses.
BAYER_TO_CV = {
    "bayer_rggb": "COLOR_BayerBG2BGR",
    "bayer_bggr": "COLOR_BayerRG2BGR",
    "bayer_gbrg": "COLOR_BayerGR2BGR",
    "bayer_grbg": "COLOR_BayerGB2BGR",
}


def decode_image(encoding, data, height, width, step, is_bigendian):
    """Return (array HxW or HxWxC, bits) with the raw integer sensor values."""
    enc = encoding.lower()
    if enc.endswith("16"):
        dtype = np.dtype(">u2" if is_bigendian else "<u2")
        bits = 16
    else:
        dtype = np.dtype(np.uint8)
        bits = 8
    buf = np.frombuffer(data, dtype=dtype)
    row_len = step // dtype.itemsize
    a = buf[: height * row_len].reshape(height, row_len)
    channels = 1
    for prefix, n in (("rgba", 4), ("bgra", 4), ("rgb", 3), ("bgr", 3)):
        if enc.startswith(prefix):
            channels = n
            break
    if channels > 1:
        a = a[:, : width * channels].reshape(height, width, channels)
    else:
        a = a[:, :width]
    return a, bits


def to_bgr(raw, encoding):
    """Debayer / reorder to a BGR (or mono) integer image."""
    enc = encoding.lower()
    for key, code in BAYER_TO_CV.items():
        if enc.startswith(key):
            if cv2 is None:
                raise RuntimeError("opencv is required to debayer")
            # Lucid 16-bit Bayer is 12-bit data; OpenCV demosaics 16U fine.
            return cv2.cvtColor(np.ascontiguousarray(raw), getattr(cv2, code))
    if enc.startswith("rgb"):
        return raw[..., [2, 1, 0]]
    if enc.startswith("bgr"):
        return raw[..., :3]
    return raw  # mono


def full_scale(img, bits):
    if bits == 8:
        return 255.0
    # 16-bit container: right-justified 12-bit data reads <= 4095.
    return 4095.0 if int(img.max()) <= 4095 else 65535.0


def linearize(img, bits, camera_gamma):
    """Integer image -> float32 linear light in [0, 1]."""
    x = img.astype(np.float32) / full_scale(img, bits)
    if camera_gamma and abs(camera_gamma - 1.0) > 1e-6:
        # Camera applied out = in ** gamma; invert it.
        x = np.power(np.clip(x, 0.0, 1.0), 1.0 / camera_gamma)
    return x


def luminance(lin):
    if lin.ndim == 2:
        return lin
    b, g, r = lin[..., 0], lin[..., 1], lin[..., 2]
    return 0.0722 * b + 0.7152 * g + 0.2126 * r


def choose_ev(linear_frames, target=0.90, percentile=99.5, max_ev=8.0):
    """EV that maps the given percentile of linear luminance (pooled over the
    sampled frames) to `target`. Never negative: the point is to lift."""
    vals = np.concatenate([luminance(f)[::4, ::4].ravel() for f in linear_frames])
    p = float(np.percentile(vals, percentile))
    if p <= 1e-6:
        return max_ev
    ev = float(np.log2(target / p))
    return float(np.clip(ev, 0.0, max_ev))


def apply_lift(lin, ev, display_gamma=2.2):
    """Linear [0,1] -> 8-bit display image after +ev stops."""
    y = np.clip(lin * (2.0 ** ev), 0.0, 1.0)
    if display_gamma and abs(display_gamma - 1.0) > 1e-6:
        y = np.power(y, 1.0 / display_gamma)
    return (y * 255.0 + 0.5).astype(np.uint8)


def process_frame(raw, bits, encoding, camera_gamma, ev, display_gamma):
    """Full per-frame pipeline: raw sensor ints -> lifted 8-bit BGR/mono."""
    bgr = to_bgr(raw, encoding)
    lin = linearize(bgr, bits, camera_gamma)
    return apply_lift(lin, ev, display_gamma)


def open_reader(bag_uri, storage_id):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    return reader


def iter_images(bag_uri, storage_id, topics, every, max_frames):
    """Yield (topic, msg) for every Nth frame per topic, up to max_frames each."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(bag_uri, storage_id)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in topics if t in type_map]
    if not present:
        raise SystemExit(f"none of {topics} in {bag_uri}; topics: {sorted(type_map)}")
    seen = {t: 0 for t in present}
    yielded = {t: 0 for t in present}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in seen:
            continue
        seen[topic] += 1
        if (seen[topic] - 1) % every != 0:
            continue
        if max_frames and yielded[topic] >= max_frames:
            if all(yielded[t] >= max_frames for t in present):
                return
            continue
        yielded[topic] += 1
        yield topic, deserialize_message(data, get_message(type_map[topic]))


def cam_name(topic):
    parts = [p for p in topic.split("/") if p]
    return parts[0] if parts else "cam"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 directory (mcap)")
    ap.add_argument("-o", "--out", required=True, help="output directory")
    ap.add_argument("--topics", nargs="+", default=CAMS)
    ap.add_argument("--storage", default="mcap")
    ap.add_argument("--ev", type=float, default=None,
                    help="fixed lift in stops for all cameras (disables --auto)")
    ap.add_argument("--auto", action="store_true", default=True,
                    help="pick EV per camera from sample frames (default)")
    ap.add_argument("--auto-global", action="store_true",
                    help="one auto EV shared by all cameras")
    ap.add_argument("--auto-target", type=float, default=0.90,
                    help="linear luminance the 99.5th percentile is lifted to (default 0.90)")
    ap.add_argument("--auto-percentile", type=float, default=99.5,
                    help="luminance percentile that is lifted to --auto-target (default 99.5)")
    ap.add_argument("--auto-samples", type=int, default=10,
                    help="frames per camera used to choose the auto EV (default 10)")
    ap.add_argument("--camera-gamma", type=float, default=0.5,
                    help="Gamma the cameras were launched with (default 0.5; use 1.0 if linear)")
    ap.add_argument("--display-gamma", type=float, default=2.2,
                    help="output encoding gamma (default 2.2; 1.0 writes linear)")
    ap.add_argument("--every", type=int, default=1, help="process every Nth frame")
    ap.add_argument("--max-frames", type=int, default=0, help="per camera; 0 = all")
    ap.add_argument("--format", choices=["jpg", "png"], default="jpg")
    ap.add_argument("--jpeg-quality", type=int, default=92)
    args = ap.parse_args()

    if cv2 is None:
        raise SystemExit("python3-opencv (cv2) is required")

    every = max(1, args.every)

    # --- pass 1: choose EV ----------------------------------------------------
    if args.ev is not None:
        ev_for = {t: float(args.ev) for t in args.topics}
        print(f"fixed EV +{args.ev:.2f} for all cameras")
    else:
        samples = {}
        for topic, msg in iter_images(args.bag, args.storage, args.topics,
                                      every, args.auto_samples):
            raw, bits = decode_image(msg.encoding, msg.data, msg.height,
                                     msg.width, msg.step, msg.is_bigendian)
            lin = linearize(to_bgr(raw, msg.encoding), bits, args.camera_gamma)
            samples.setdefault(topic, []).append(lin)
        if args.auto_global:
            pooled = [f for fr in samples.values() for f in fr]
            ev = choose_ev(pooled, args.auto_target, args.auto_percentile)
            ev_for = {t: ev for t in samples}
            print(f"auto EV (global, {len(pooled)} frames): +{ev:.2f} stops")
        else:
            ev_for = {t: choose_ev(fr, args.auto_target, args.auto_percentile)
                      for t, fr in samples.items()}
            for t in sorted(ev_for):
                print(f"auto EV {cam_name(t):10s} +{ev_for[t]:.2f} stops "
                      f"({len(samples[t])} sample frames)")
        print("(pass --ev <stops> to pin this for a reproducible re-run)")

    # --- pass 2: write frames -------------------------------------------------
    written = {}
    enc_params = ([cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
                  if args.format == "jpg" else [])
    for topic, msg in iter_images(args.bag, args.storage, args.topics,
                                  every, args.max_frames):
        ev = ev_for.get(topic)
        if ev is None:
            continue
        raw, bits = decode_image(msg.encoding, msg.data, msg.height,
                                 msg.width, msg.step, msg.is_bigendian)
        out = process_frame(raw, bits, msg.encoding, args.camera_gamma, ev,
                            args.display_gamma)
        d = os.path.join(args.out, cam_name(topic))
        os.makedirs(d, exist_ok=True)
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        path = os.path.join(d, f"{stamp}.{args.format}")
        if not cv2.imwrite(path, out, enc_params):
            print(f"failed to write {path}", file=sys.stderr)
            continue
        written[topic] = written.get(topic, 0) + 1
        n = written[topic]
        if n % 100 == 0:
            print(f"{cam_name(topic)}: {n} frames")

    print("\nwritten:")
    for t in sorted(written):
        print(f"  {cam_name(t):10s} {written[t]:6d}  (EV +{ev_for[t]:.2f})  -> "
              f"{os.path.join(args.out, cam_name(t))}")


if __name__ == "__main__":
    main()
