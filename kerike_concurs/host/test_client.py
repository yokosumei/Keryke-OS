#!/usr/bin/env python3
"""Test mic — citește 10 pachete de la camera_publisher și confirmă structura."""
import json
import socket
import struct
import sys

def recv_exact(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Socket closed")
        data += chunk
    return data

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("127.0.0.1", 9999))
    print("Connected. Receiving 10 packets...")

    for i in range(10):
        meta_len = struct.unpack(">I", recv_exact(sock, 4))[0]
        metadata = json.loads(recv_exact(sock, meta_len))
        frame_len = struct.unpack(">I", recv_exact(sock, 4))[0]
        frame = recv_exact(sock, frame_len)

        n_det = len(metadata["detections"])
        classes = [d["class_name"] for d in metadata["detections"]]
        print(f"Packet {i+1}: frame#{metadata['frame_id']} "
              f"{metadata['width']}x{metadata['height']} | "
              f"jpg={frame_len}B | {n_det} det → {classes}")

    sock.close()
    print("✅ All 10 packets received correctly. Publisher works!")

if __name__ == "__main__":
    sys.exit(main())
