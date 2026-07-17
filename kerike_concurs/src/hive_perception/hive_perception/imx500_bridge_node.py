#!/usr/bin/env python3
"""
IMX500 Bridge — primește frame-uri de la host publisher
(host/camera_publisher.py, prin TCP socket localhost:9999) și le
publică în ROS 2 pe /perception/image_raw. Detecția reală se face
separat, în yolo_detection_node.py (yolov8n.pt, pe CPU-ul Pi-ului) --
IMX500-ul e folosit doar ca senzor de imagine, nu pentru NPU-ul
on-chip (ar necesita conversia modelului prin toolchain-ul Sony).
"""
import socket
import struct
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

HOST = "localhost"
PORT = 9999


class IMX500BridgeNode(Node):
    def __init__(self):
        super().__init__("imx500_bridge")
        self.bridge = CvBridge()
        self.img_pub = self.create_publisher(Image, "/perception/image_raw", 10)

        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(f"📷 IMX500 bridge connecting to {HOST}:{PORT}")

    def _connect(self):
        while not self._stop:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((HOST, PORT))
                self.get_logger().info("✅ Connected to host publisher")
                return s
            except (ConnectionRefusedError, OSError) as e:
                self.get_logger().warn(
                    f"Host publisher not ready ({e}), retry în 2s..."
                )
                import time
                time.sleep(2.0)

    def _recv_exact(self, sock, n):
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("Host closed")
            buf += chunk
        return buf

    def _run(self):
        sock = self._connect()
        if sock is None:
            return

        while not self._stop:
            try:
                # Protocol: [4 bytes BE: meta_len][meta JSON][4 bytes BE: jpg_len][jpg bytes]
                meta_len = struct.unpack(">I", self._recv_exact(sock, 4))[0]
                self._recv_exact(sock, meta_len)  # meta neutilizat -- vezi docstring

                jpg_len = struct.unpack(">I", self._recv_exact(sock, 4))[0]
                jpg_data = self._recv_exact(sock, jpg_len)

                # Decode imagine
                np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # Publish Image
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "imx500"
                self.img_pub.publish(img_msg)

            except (ConnectionError, OSError) as e:
                self.get_logger().warn(f"Connection lost: {e}, reconnecting...")
                try: sock.close()
                except Exception: pass
                sock = self._connect()
                if sock is None: return
            except Exception as e:
                self.get_logger().error(f"Bridge error: {e}")

    def stop(self):
        self._stop = True


def main(args=None):
    rclpy.init(args=args)
    node = IMX500BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
