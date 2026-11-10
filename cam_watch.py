#!/usr/bin/env python3
"""
cam_watch.py - watch several camera topics and alert the moment one stops publishing.

Prints one status line per second, beeps on a drop, and logs every event
(first frame, drop, recovery) to a CSV so you can see exactly when and how
long into a run each camera failed.

Usage:
  python3 cam_watch.py [--timeout 2.0] [--log cam_events.csv]

Topics are hardcoded in TOPICS below (/arenacam1..6/image_raw).

Notes:
  - Subscribes "raw" (no image deserialization), so it adds almost no CPU load.
  - Uses best-effort QoS, which works with both reliable and best-effort publishers.
  - Ctrl-C prints a per-camera summary.
"""
import argparse
import csv
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


TOPICS = [f"/arenacam{i}/image_raw" for i in range(1, 7)]


def fmt_dur(sec):
    sec = int(sec)
    return f"{sec // 3600:02d}:{(sec % 3600) // 60:02d}:{sec % 60:02d}"


class Stream:
    def __init__(self, label, topic):
        self.label = label
        self.topic = topic
        self.count = 0
        self.window = 0          # messages since last status tick
        self.first = None        # monotonic time of first frame
        self.last = None         # monotonic time of latest frame
        self.up_since = None     # start of current healthy stretch
        self.down = False
        self.drops = 0
        self.longest_up = 0.0


class CamWatch(Node):
    def __init__(self, topics, timeout, log_path, msg_type):
        super().__init__("cam_watch")
        self.timeout = timeout
        self.t0 = time.monotonic()
        self.last_tick = self.t0
        self.streams = [Stream(f"cam{i}", t) for i, t in enumerate(topics, start=1)]

        for s in self.streams:
            self.create_subscription(
                msg_type, s.topic,
                lambda _msg, s=s: self.on_msg(s),
                qos_profile_sensor_data, raw=True)

        self.log_file = open(log_path, "a", newline="")
        self.csv = csv.writer(self.log_file)
        self.event(None, "watch_start", f"timeout={timeout}s")

        print("Watching:")
        for s in self.streams:
            print(f"  {s.label} = {s.topic}")
        print(f"Drop = no frame for {timeout}s. Events logged to {log_path}\n")

        self.create_timer(1.0, self.tick)

    def event(self, s, name, detail=""):
        now = time.monotonic()
        self.csv.writerow([
            datetime.now().isoformat(timespec="seconds"),
            f"{now - self.t0:.1f}",
            s.topic if s else "-",
            name,
            detail,
        ])
        self.log_file.flush()

    def on_msg(self, s):
        now = time.monotonic()
        s.count += 1
        s.window += 1
        if s.first is None:
            s.first = now
            s.up_since = now
            self.event(s, "first_frame", f"{now - self.t0:.1f}s after watch start")
        elif s.down:
            gap = now - s.last
            s.down = False
            s.up_since = now
            self.event(s, "recovered", f"gap={gap:.1f}s")
            print(f"\n  >>> {s.label} RECOVERED after {gap:.1f}s gap")
        s.last = now

    def tick(self):
        now = time.monotonic()
        dt = max(now - self.last_tick, 1e-3)
        self.last_tick = now
        parts = []

        for s in self.streams:
            hz = s.window / dt
            s.window = 0

            if s.first is None:
                parts.append(f"{s.label} WAITING")
                continue

            if s.up_since is not None and not s.down:
                s.longest_up = max(s.longest_up, now - s.up_since)

            gap = now - s.last
            if gap > self.timeout:
                if not s.down:
                    s.down = True
                    s.drops += 1
                    ran = s.last - s.up_since
                    self.event(s, "DROP", f"ran {fmt_dur(ran)} before dropping")
                    print(f"\a\n  !!! {s.label} DROPPED ({s.topic}) "
                          f"after running {fmt_dur(ran)}")
                parts.append(f"{s.label} DOWN {gap:.0f}s")
            else:
                parts.append(f"{s.label} {hz:4.1f}Hz")

        print(f"[{fmt_dur(now - self.t0)}] " + " | ".join(parts), flush=True)

    def summary(self):
        now = time.monotonic()
        print("\n\nSummary")
        for s in self.streams:
            if s.first is None:
                print(f"  {s.label} {s.topic}: NEVER STARTED")
                continue
            if not s.down and s.up_since is not None:
                s.longest_up = max(s.longest_up, now - s.up_since)
            active = (s.last - s.first) or 1e-3
            print(f"  {s.label} {s.topic}: {s.count} frames, "
                  f"avg {s.count / active:.1f}Hz, drops={s.drops}, "
                  f"longest run {fmt_dur(s.longest_up)}")
        self.event(None, "watch_stop")
        self.log_file.close()


def main():
    ap = argparse.ArgumentParser(description="Alert when camera topics stop publishing.")
    ap.add_argument("--timeout", type=float, default=2.0,
                    help="seconds without a frame before declaring a drop (default 2)")
    ap.add_argument("--log", default="cam_events.csv", help="CSV event log path")
    ap.add_argument("--compressed", action="store_true",
                    help="topics are sensor_msgs/CompressedImage")
    args = ap.parse_args()

    rclpy.init()
    node = CamWatch(TOPICS, args.timeout, args.log,
                    CompressedImage if args.compressed else Image)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
