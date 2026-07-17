#!/usr/bin/env python3
"""
camera_publisher.py — rulează pe HOST (în afara Docker).

Captează video de la camera IMX500 ca senzor de imagine obișnuit
(Picamera2), FĂRĂ să folosească NPU-ul on-chip al cipului. NPU-ul
IMX500 rulează doar modele precompilate in format .rpk, printr-un
toolchain separat al Sony (imx500-converter) -- ca sa incarci un YOLO
antrenat de tine acolo ai avea nevoie de conversia aia, efort care nu
se justifica acum. Detectia reala se face separat, in Python, pe
CPU-ul Pi 5 (yolo_detection_node.py / yolov8n.pt din models/).

Servește frame-uri via TCP socket localhost:9999 către container.
"""
import json
import socket
import struct
import sys
import threading
import time

import cv2
from picamera2 import Picamera2

PORT = 9999
WIDTH, HEIGHT = 640, 480

_lock = threading.Lock()
_latest_frame = None


def main():
    global _latest_frame

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
        controls={"FrameRate": 30},
        buffer_count=4,
    )
    picam2.configure(config)
    picam2.start(config)
    print("[host] Camera pornita (fara AI on-chip).")

    def camera_loop():
        global _latest_frame
        while True:
            try:
                frame = picam2.capture_array()
                with _lock:
                    _latest_frame = frame
            except Exception as e:
                print(f"[host] camera_loop err: {e}")
                time.sleep(0.1)

    threading.Thread(target=camera_loop, daemon=True).start()

    # TCP server pe localhost:9999
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", PORT))
    server.listen(2)
    print(f"[host] TCP server listening on 127.0.0.1:{PORT}")

    while True:
        client, addr = server.accept()
        print(f"[host] Client connected from {addr}")
        try:
            while True:
                with _lock:
                    frame = _latest_frame
                if frame is None:
                    time.sleep(0.02)
                    continue

                ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue
                jpg_bytes = jpg.tobytes()

                # Trimite [meta_len][meta_json][jpg_len][jpg]. meta ramane
                # in protocol (imx500_bridge_node il consuma neconditionat)
                # dar fara detections -- alea vin acum de la yolo_detection_ros.
                meta = json.dumps({}).encode("utf-8")
                client.sendall(struct.pack(">I", len(meta)))
                client.sendall(meta)
                client.sendall(struct.pack(">I", len(jpg_bytes)))
                client.sendall(jpg_bytes)

                time.sleep(0.033)  # ~30 FPS
        except (ConnectionResetError, BrokenPipeError) as e:
            print(f"[host] Client disconnected: {e}")
        except KeyboardInterrupt:
            print("[host] Shutting down")
            break
        finally:
            try: client.close()
            except: pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[host] Bye!")
        sys.exit(0)
