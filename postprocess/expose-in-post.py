#!/usr/bin/env python3
"""
Lift deliberately under-exposed (static exposure) camera frames from a rosbag
into viewable images, with a per-run, per-frame or smoothed per-frame EV.

Companion to drivers/lucid-drivers-sync-trigger-static-exposure.sh: the
cameras run open-loop and DARK so highlights never clip, and brightness is
restored here. Because the camera never adapts, the raw data in the bag is
photometrically consistent; this tool decides how much to lift AFTER the fact,
with the whole run available, so unlike in-camera autoexposure there is no
convergence lag and no flicker.

Pipeline per frame:
  1. decode the sensor_msgs/Image (bayer_*8 / bayer_*16 / mono / rgb / bgr)
  2. debayer (OpenCV, pattern taken from the encoding string)
  3. undo the in-camera gamma (the launcher uses gamma=0.5) -> linear light
  4. multiply by 2**EV
  5. clip, apply a display gamma (default 2.2) and write an 8-bit JPEG/PNG

EV = stops of lift. The "highlight level" of a frame is the --auto-percentile
(99.5th) of its linear pixel values; EV is whatever puts that level at
--auto-target (0.90): as bright as possible with highlights intact.

--ev-mode
  run     one EV per camera for the whole run, from the first --auto-samples
          frames. Fully consistent; wrong if lighting changes mid-run.
  frame   EV chosen from each frame on its own. Tracks lighting instantly but
          can flicker frame to frame (passing cars, sun glints).
  smooth  (default) per-frame EV, then a CENTERED rolling median + mean over
          --ev-window seconds (default 2 s). Follows clouds / dusk / tunnels
          with no lag (it sees the future) and no flicker. Recommended for
          long runs.
--ev <stops>  fixed EV for every frame (overrides --ev-mode).
--ev-global   share one EV across all cameras at each instant (the brightest
              camera sets it). Keeps cross-camera brightness comparable, which
              matters when frames from several cameras feed one reconstruction.
              Without it each camera gets its own EV track.

Every write is recorded in <out>/exposure_ev.csv (camera, stamp_ns, file,
ev_applied, highlight_level_linear). A reconstruction pipeline that models
per-image exposure can consume ev_applied directly; lifting is a pure gain in
linear light, so multiplying back by 2**-ev recovers the original relative
radiance (up to clipping at 1.0).

Output: <out>/<camera>/<header.stamp ns>.jpg  (stamp = PTP/RTK capture time,
so frames from different cameras with the same stamp are the same shot.)

Usage:
    source /opt/ros/humble/setup.bash
    python3 expose-in-post.py "<run_dir>" -o <outdir>                     # all bags under a split-recorder run folder
    python3 expose-in-post.py "<bagdir>" -o <outdir>                      # a single bag
    python3 expose-in-post.py "<bagdir>" -o <outdir> --ev-global          # smooth, one EV per shot
    python3 expose-in-post.py "<bagdir>" -o <outdir> --ev-mode run        # one EV per camera
    python3 expose-in-post.py "<bagdir>" -o <outdir> --ev 3               # fixed +3 stops
    python3 expose-in-post.py "<bagdir>" -o <outdir> --every 30 --max-frames 20   # quick look
    python3 expose-in-post.py "<bagdir>" -o <outdir> --camera-gamma 1.0   # if launched with gamma=1.0

The bag is read twice in the auto modes (stats pass, then write pass). The
stats pass is cheap (subsampled raw values, no debayer); --ev-sample-every N
computes stats on every Nth frame and interpolates in between.

Requires numpy + opencv (python3-opencv on the vehicle) and the ROS 2 python
packages (rosbag2_py, rclpy) for reading the bag.
"""
import argparse
import csv
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

EV_MIN = -1.0   # allow a slight darkening if a frame is brighter than target
EV_MAX = 8.0


# --------------------------------------------------------------------------
# image math
# --------------------------------------------------------------------------

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


def highlight_level(raw, bits, camera_gamma, percentile=99.5, sub=8):
    """Linear-light level of the given percentile of a frame's raw sensor
    values (all channels pooled, subsampled). Cheap: no debayer."""
    s = raw[::sub, ::sub]
    lin = linearize(s, bits, camera_gamma)
    return float(np.percentile(lin, percentile))


def ev_from_level(level, target=0.90):
    """EV (stops) that maps `level` to `target`, clipped to [EV_MIN, EV_MAX]."""
    if level <= 1e-6:
        return EV_MAX
    return float(np.clip(np.log2(target / level), EV_MIN, EV_MAX))


def choose_ev(linear_frames, target=0.90, percentile=99.5, max_ev=EV_MAX):
    """Run-mode EV from debayered linear frames (pooled percentile)."""
    vals = np.concatenate([luminance(f)[::4, ::4].ravel() for f in linear_frames])
    p = float(np.percentile(vals, percentile))
    if p <= 1e-6:
        return max_ev
    return float(np.clip(np.log2(target / p), 0.0, max_ev))


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


# --------------------------------------------------------------------------
# EV tracks over time
# --------------------------------------------------------------------------

def _centered_window_filter(t, x, window_s, fn):
    """Apply fn over a centered window of +-window_s/2 seconds around each
    sample. t is sorted (seconds), x same length."""
    if window_s <= 0 or len(x) < 2:
        return np.asarray(x, dtype=np.float64)
    half = window_s / 2.0
    lo = np.searchsorted(t, t - half, side="left")
    hi = np.searchsorted(t, t + half, side="right")
    out = np.empty(len(x), dtype=np.float64)
    for i in range(len(x)):
        out[i] = fn(x[lo[i]:hi[i]])
    return out


def smooth_ev(t, ev, window_s):
    """Centered rolling median (kills one-frame glints) then centered rolling
    mean (removes steps) over window_s seconds. Non-causal on purpose: post has
    the whole run, so the lift can anticipate a change instead of lagging it."""
    ev = np.asarray(ev, dtype=np.float64)
    med = _centered_window_filter(t, ev, window_s, np.median)
    return _centered_window_filter(t, med, window_s, np.mean)


class EVTrack:
    """Per-camera (stamp -> EV) lookup built from sampled highlight levels."""

    def __init__(self, stamps_ns, levels, mode, target, window_s):
        order = np.argsort(stamps_ns)
        self.t = np.asarray(stamps_ns, dtype=np.float64)[order] / 1e9
        self.level = np.asarray(levels, dtype=np.float64)[order]
        ev = np.array([ev_from_level(p, target) for p in self.level])
        if mode == "smooth":
            ev = smooth_ev(self.t, ev, window_s)
        elif mode == "run":
            # One value for the run, set by the 90th percentile of the sampled
            # highlight levels: protects highlights in nearly every frame
            # without letting a single glint frame darken the whole run.
            ev = np.full_like(ev, ev_from_level(float(np.percentile(self.level, 90)),
                                                target))
        self.ev = ev

    def at(self, stamp_ns):
        """EV at a stamp; linear interpolation between sampled frames."""
        if len(self.t) == 1:
            return float(self.ev[0])
        return float(np.interp(stamp_ns / 1e9, self.t, self.ev))

    def level_at(self, stamp_ns):
        if len(self.t) == 1:
            return float(self.level[0])
        return float(np.interp(stamp_ns / 1e9, self.t, self.level))


def build_tracks(samples, mode, target, window_s, ev_global):
    """samples: {topic: (stamps_ns list, levels list)} -> {topic: EVTrack}.

    With ev_global, all cameras share one track: at each sampled instant the
    highlight level is the MAX over cameras (the brightest camera must not
    clip), evaluated on the union of all sample stamps.
    """
    if not ev_global:
        return {t: EVTrack(s, l, mode, target, window_s)
                for t, (s, l) in samples.items()}
    per_cam = {t: EVTrack(s, l, "frame", target, 0.0) for t, (s, l) in samples.items()}
    all_stamps = np.unique(np.concatenate(
        [np.asarray(s, dtype=np.int64) for s, _ in samples.values()]))
    all_t = all_stamps.astype(np.float64) / 1e9
    pooled = np.max([np.interp(all_t, tr.t, tr.level) for tr in per_cam.values()],
                    axis=0)
    shared = EVTrack(all_stamps, pooled, mode, target, window_s)
    return {t: shared for t in samples}


# --------------------------------------------------------------------------
# bag I/O
# --------------------------------------------------------------------------

def open_reader(bag_uri, storage_id):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"),
    )
    return reader


def expand_bags(paths):
    """Accept bag directories and/or run folders. A run folder from the split
    recorder holds several bags (camA, camB, lidar_ins_tf), each with its own
    metadata.yaml; expand it to those so one invocation covers all cameras."""
    bags = []
    for p in paths:
        if os.path.isfile(os.path.join(p, "metadata.yaml")):
            bags.append(p)
            continue
        subs = sorted(d for d in (os.path.join(p, n) for n in os.listdir(p))
                      if os.path.isfile(os.path.join(d, "metadata.yaml")))
        if not subs:
            raise SystemExit(f"{p} is neither a bag (no metadata.yaml) nor a run "
                             f"folder containing bags")
        bags.extend(subs)
    return bags


def iter_images(bag_uris, storage_id, topics, every, max_frames):
    """Yield (topic, msg) for every Nth frame per topic, up to max_frames
    each, across one or more bags (each bag holds a subset of the cameras)."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    any_present = False
    for bag_uri in bag_uris:
        reader = open_reader(bag_uri, storage_id)
        type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
        present = [t for t in topics if t in type_map]
        if not present:
            continue  # e.g. the lidar/INS bag
        any_present = True
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
                    break
                continue
            yielded[topic] += 1
            yield topic, deserialize_message(data, get_message(type_map[topic]))
    if not any_present:
        raise SystemExit(f"none of {topics} found in {bag_uris}")


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def cam_name(topic):
    parts = [p for p in topic.split("/") if p]
    return parts[0] if parts else "cam"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", nargs="+",
                    help="rosbag2 directory (mcap), several of them, or a run folder "
                         "from the split recorder (camA/camB/... are read together)")
    ap.add_argument("-o", "--out", required=True, help="output directory")
    ap.add_argument("--topics", nargs="+", default=CAMS)
    ap.add_argument("--storage", default="mcap")
    ap.add_argument("--ev", type=float, default=None,
                    help="fixed lift in stops for every frame (overrides --ev-mode)")
    ap.add_argument("--ev-mode", choices=["run", "frame", "smooth"], default="smooth",
                    help="how EV varies over the run (default smooth)")
    ap.add_argument("--ev-window", type=float, default=2.0,
                    help="smooth mode: centered window in seconds (default 2.0)")
    ap.add_argument("--ev-global", action="store_true",
                    help="one shared EV across all cameras at each instant")
    ap.add_argument("--ev-sample-every", type=int, default=1,
                    help="stats pass: measure every Nth frame, interpolate between (default 1)")
    ap.add_argument("--auto-global", action="store_true",
                    help="alias for --ev-mode run --ev-global")
    ap.add_argument("--auto-target", type=float, default=0.90,
                    help="linear level the highlight percentile is lifted to (default 0.90)")
    ap.add_argument("--auto-percentile", type=float, default=99.5,
                    help="percentile of linear pixel values treated as the highlight (default 99.5)")
    ap.add_argument("--auto-samples", type=int, default=10,
                    help="run mode: frames per camera used to choose the EV (default 10)")
    ap.add_argument("--camera-gamma", type=float, default=0.5,
                    help="Gamma the cameras were launched with (default 0.5; use 1.0 if linear)")
    ap.add_argument("--display-gamma", type=float, default=2.2,
                    help="output encoding gamma (default 2.2; 1.0 writes linear)")
    ap.add_argument("--every", type=int, default=1, help="write every Nth frame")
    ap.add_argument("--max-frames", type=int, default=0, help="per camera; 0 = all")
    ap.add_argument("--format", choices=["jpg", "png"], default="jpg")
    ap.add_argument("--jpeg-quality", type=int, default=92)
    args = ap.parse_args()

    if cv2 is None:
        raise SystemExit("python3-opencv (cv2) is required")
    if args.auto_global:
        args.ev_mode, args.ev_global = "run", True

    every = max(1, args.every)
    args.bag = expand_bags(args.bag)
    print("bags: " + ", ".join(args.bag))

    # --- pass 1: EV tracks --------------------------------------------------
    tracks = None
    if args.ev is not None:
        print(f"fixed EV +{args.ev:.2f} for every frame")
    else:
        if args.ev_mode == "run":
            s_every, s_max = every, args.auto_samples
        else:
            # cover the same span the write pass will, measured every Nth frame
            s_every = max(1, args.ev_sample_every)
            s_max = int(np.ceil(args.max_frames * every / s_every)) if args.max_frames else 0
        samples = {}
        n = 0
        for topic, msg in iter_images(args.bag, args.storage, args.topics,
                                      s_every, s_max):
            raw, bits = decode_image(msg.encoding, msg.data, msg.height,
                                     msg.width, msg.step, msg.is_bigendian)
            lvl = highlight_level(raw, bits, args.camera_gamma, args.auto_percentile)
            st, lv = samples.setdefault(topic, ([], []))
            st.append(stamp_ns(msg))
            lv.append(lvl)
            n += 1
            if n % 2000 == 0:
                print(f"stats pass: {n} frames measured")
        if not samples:
            raise SystemExit("no frames found")
        tracks = build_tracks(samples, args.ev_mode, args.auto_target,
                              args.ev_window, args.ev_global)
        print(f"EV mode: {args.ev_mode}"
              f"{' (global across cameras)' if args.ev_global else ' (per camera)'}"
              f"{f', window {args.ev_window:.1f} s' if args.ev_mode == 'smooth' else ''}")
        for t in sorted(tracks):
            tr = tracks[t]
            print(f"  {cam_name(t):10s} EV min/median/max = "
                  f"{tr.ev.min():+.2f} / {np.median(tr.ev):+.2f} / {tr.ev.max():+.2f} stops "
                  f"({len(tr.ev)} samples)")

    # --- pass 2: write frames + sidecar ---------------------------------------
    os.makedirs(args.out, exist_ok=True)
    csv_path = os.path.join(args.out, "exposure_ev.csv")
    written = {}
    enc_params = ([cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
                  if args.format == "jpg" else [])
    with open(csv_path, "w", newline="") as fh:
        wr = csv.writer(fh)
        wr.writerow(["camera", "stamp_ns", "file", "ev_applied",
                     "highlight_level_linear", "ev_mode"])
        for topic, msg in iter_images(args.bag, args.storage, args.topics,
                                      every, args.max_frames):
            st = stamp_ns(msg)
            if args.ev is not None:
                ev, lvl = float(args.ev), ""
            else:
                tr = tracks.get(topic)
                if tr is None:
                    continue
                ev, lvl = tr.at(st), f"{tr.level_at(st):.5f}"
            raw, bits = decode_image(msg.encoding, msg.data, msg.height,
                                     msg.width, msg.step, msg.is_bigendian)
            out = process_frame(raw, bits, msg.encoding, args.camera_gamma, ev,
                                args.display_gamma)
            d = os.path.join(args.out, cam_name(topic))
            os.makedirs(d, exist_ok=True)
            fname = f"{st}.{args.format}"
            path = os.path.join(d, fname)
            if not cv2.imwrite(path, out, enc_params):
                print(f"failed to write {path}", file=sys.stderr)
                continue
            wr.writerow([cam_name(topic), st, os.path.join(cam_name(topic), fname),
                         f"{ev:.4f}", lvl,
                         "fixed" if args.ev is not None else args.ev_mode])
            written[topic] = written.get(topic, 0) + 1
            if written[topic] % 500 == 0:
                print(f"{cam_name(topic)}: {written[topic]} frames")

    print("\nwritten:")
    for t in sorted(written):
        print(f"  {cam_name(t):10s} {written[t]:6d}  -> {os.path.join(args.out, cam_name(t))}")
    print(f"per-frame EV sidecar: {csv_path}")


if __name__ == "__main__":
    main()
