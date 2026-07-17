#!/usr/bin/env python3
"""
grab_frame.py -- salveaza UN cadru de pe /perception/image_raw pe disc,
apoi iese. Pentru calibrare (calibrate_depth.py) FARA sa opresti
camera_publisher.py sau sa intri in conflict cu rpicam-still pe camera
(Picamera2 tine camera ocupata exclusiv -- doua procese nu pot s-o
foloseasca simultan).

Nu are nevoie de YOLO/perception_container pornit -- doar de
camera_publisher.py (pe host) + imx500_bridge (in container), care
publica pe /perception/image_raw.

RULARE (in container, cu imx500_bridge deja pornit in alt terminal):
    python3 /ws/scripts/grab_frame.py frame_1m.jpg
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class _GrabOnce(Node):
    def __init__(self, out_path: str):
        super().__init__("grab_frame")
        self.out_path = out_path
        self.bridge = CvBridge()
        self.got_frame = False
        self.sub = self.create_subscription(
            Image, "/perception/image_raw", self._on_image, 1
        )

    def _on_image(self, msg: Image) -> None:
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(self.out_path, bgr)
        self.get_logger().info(f"Salvat {self.out_path} ({bgr.shape[1]}x{bgr.shape[0]})")
        self.got_frame = True


def main() -> None:
    if len(sys.argv) != 2:
        print("Uzaj: python3 grab_frame.py <cale_iesire.jpg>")
        sys.exit(1)
    out_path = sys.argv[1]

    rclpy.init()
    node = _GrabOnce(out_path)
    try:
        while rclpy.ok() and not node.got_frame:
            rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
