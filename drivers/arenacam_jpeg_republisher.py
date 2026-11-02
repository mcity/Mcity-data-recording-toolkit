#!/usr/bin/env python3
"""arenacam_jpeg_republisher: JPEG copies of the LUCID camera streams. Run on .28.

Keeps 10 Hz by stamp slot, debayers, JPEG q80, publishes <topic>/compressed
(sensor_msgs/CompressedImage, original header). See tools/ in
mcity-physical-ai-autostack on .27 for the full header comment.
"""
import array
import time

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

_BAYER = {"bayer_rggb8": cv2.COLOR_BayerBG2BGR, "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
          "bayer_gbrg8": cv2.COLOR_BayerGR2BGR, "bayer_grbg8": cv2.COLOR_BayerGB2BGR}


class ArenacamJpegRepublisher(Node):
    def __init__(self):
        super().__init__("arenacam_jpeg_republisher")
        p = self.declare_parameter
        self.topics = list(p("topics", ["/arenacam1/images", "/arenacam2/images",
                                        "/arenacam3/images"]).value)
        self.rate = float(p("rate_hz", 10.0).value)
        self.quality = int(p("jpeg_quality", 80).value)
        suffix = str(p("suffix", "/compressed").value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.last_slot = [None] * len(self.topics)
        self.n_out = [0] * len(self.topics)
        self.bytes_out = [0] * len(self.topics)
        self.warned = set()
        self.pubs = []
        for i, t in enumerate(self.topics):
            self.pubs.append(self.create_publisher(CompressedImage, t.rstrip("/") + suffix, qos))
            self.create_subscription(Image, t, lambda m, i=i: self.on_image(i, m), qos,
                                     callback_group=MutuallyExclusiveCallbackGroup())
        self.t_stats = time.monotonic()
        self.create_timer(5.0, self.log_stats)
        self.get_logger().info(f"{self.topics} -> <topic>{suffix}, {self.rate} Hz, JPEG q{self.quality}")

    def on_image(self, i, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        slot = int(stamp * self.rate) if self.rate > 0 else None
        if slot is not None and slot == self.last_slot[i]:
            return
        self.last_slot[i] = slot
        enc = msg.encoding.lower()
        buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        if enc in _BAYER:
            bgr = cv2.cvtColor(buf[:, :msg.width], _BAYER[enc])
        elif enc == "bgr8":
            bgr = buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        elif enc == "rgb8":
            bgr = cv2.cvtColor(buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3),
                               cv2.COLOR_RGB2BGR)
        elif enc == "mono8":
            bgr = buf[:, :msg.width]
        else:
            if enc not in self.warned:
                self.get_logger().error(f"{self.topics[i]}: unsupported encoding '{msg.encoding}'")
                self.warned.add(enc)
            return
        ok, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = ("mono8" if enc == "mono8" else "bgr8") + "; jpeg compressed bgr8"
        out.data = array.array("B", jpg.tobytes())
        self.pubs[i].publish(out)
        self.n_out[i] += 1
        self.bytes_out[i] += len(jpg)

    def log_stats(self):
        dt = time.monotonic() - self.t_stats
        parts = []
        for i, t in enumerate(self.topics):
            n = self.n_out[i]
            kb = self.bytes_out[i] / n / 1000 if n else 0
            parts.append(f"{t.split('/')[1]} {n / dt:.1f} Hz {kb:.0f} kB")
        mbit = sum(self.bytes_out) * 8 / dt / 1e6
        self.get_logger().info(" | ".join(parts) + f" | {mbit:.0f} Mbit/s")
        self.n_out = [0] * len(self.topics)
        self.bytes_out = [0] * len(self.topics)
        self.t_stats = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = ArenacamJpegRepublisher()
    ex = MultiThreadedExecutor(num_threads=len(node.topics) + 1)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
