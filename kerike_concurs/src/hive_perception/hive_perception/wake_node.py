#!/usr/bin/env python3
"""
Wake Word Node - asculta microfonul USB.
La detectare wake, INCHIDE complet stream-ul ca brain sa poata folosi
acelasi microfon, apoi REDESCHIDE dupa pauza.
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import pyaudio
from openwakeword.model import Model

NATIVE_RATE = 48000
TARGET_RATE = 16000
CHUNK_NATIVE = 3840
DOWNSAMPLE = NATIVE_RATE // TARGET_RATE

WAKE_THRESHOLD = 0.5
COOLDOWN_SEC = 12.0
MIC_RELEASE_SEC = 9.0


class WakeNode(Node):
    def __init__(self):
        super().__init__("hive_wake")
        self.wake_pub = self.create_publisher(String, "/audio/wake_detected", 10)
        self.status_pub = self.create_publisher(String, "/audio/status", 10)

        self.get_logger().info("Loading OpenWakeWord model (hey_jarvis)...")
        self.oww = Model(
            wakeword_models=["hey_jarvis"],
            inference_framework="onnx",
        )
        self.get_logger().info("Model loaded")

        self.pa = pyaudio.PyAudio()
        self.device_index = self._find_input_device()
        if self.device_index is None:
            self.get_logger().error("No USB audio input device found!")
            raise RuntimeError("No microphone")

        self.stream = None
        self._open_stream()

        self.last_trigger = 0.0
        self._paused = False
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

        self.get_logger().info("Listening for 'Hey Jarvis'...")

    def _find_input_device(self):
        # Logam tot pentru debug
        self.get_logger().info(
            f"Scanning {self.pa.get_device_count()} audio devices..."
        )
        candidates = []
        for i in range(self.pa.get_device_count()):
            d = self.pa.get_device_info_by_index(i)
            name = d.get("name", "")
            ch_in = d.get("maxInputChannels", 0)
            ch_out = d.get("maxOutputChannels", 0)
            self.get_logger().info(
                f"  [{i}] {name} (in={ch_in}, out={ch_out})"
            )
            # Heuristic: orice device USB-named e potential microfon,
            # chiar daca maxInputChannels e 0 (PyAudio bug pe Pi)
            if "USB" in name.upper():
                candidates.append(i)
            if ch_in > 0 and "USB" in name.upper():
                self.get_logger().info(
                    f"  --> Selected (real input): index {i}"
                )
                return i

        # Daca nu am gasit input "real", incercam orice device USB
        if candidates:
            chosen = candidates[0]
            self.get_logger().warn(
                f"  --> No device with maxInputChannels>0, "
                f"trying USB device anyway: index {chosen}"
            )
            return chosen

        self.get_logger().error("No USB device found!")
        return None

    def _open_stream(self):
        if self.stream is not None:
            return
        self.get_logger().info(
            f"Opening stream on device {self.device_index} @ {NATIVE_RATE} Hz"
        )
        self.stream = self.pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=NATIVE_RATE,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=CHUNK_NATIVE,
        )

    def _close_stream(self):
        if self.stream is None:
            return
        try:
            self.stream.stop_stream()
            self.stream.close()
        except Exception as e:
            self.get_logger().warn(f"Close stream warn: {e}")
        self.stream = None

    def _loop(self):
        chunks = 0
        max_score = 0.0
        while not self._stop:
            if self._paused or self.stream is None:
                time.sleep(0.1)
                continue
            try:
                data = self.stream.read(CHUNK_NATIVE, exception_on_overflow=False)
                audio_48k = np.frombuffer(data, dtype=np.int16)
                audio_16k = audio_48k[::DOWNSAMPLE]

                prediction = self.oww.predict(audio_16k)
                score = prediction.get("hey_jarvis", 0.0)
                if score > max_score:
                    max_score = score

                chunks += 1
                if chunks % 50 == 0:
                    self.get_logger().info(
                        f"chunks={chunks} max_score_recent={max_score:.3f}"
                    )
                    max_score = 0.0

                if score > WAKE_THRESHOLD:
                    now = time.time()
                    if now - self.last_trigger < COOLDOWN_SEC:
                        continue
                    self.last_trigger = now

                    self.get_logger().info(
                        f"WAKE WORD detected (score={score:.2f})"
                    )

                    msg = String()
                    msg.data = "hey_jarvis"
                    self.wake_pub.publish(msg)

                    s = String()
                    s.data = "listening"
                    self.status_pub.publish(s)

                    # CLOSE complet stream-ul ca brain sa poata deschide
                    self.get_logger().info(
                        f"Closing stream completely for {MIC_RELEASE_SEC}s"
                    )
                    self._paused = True
                    self._close_stream()
                    time.sleep(MIC_RELEASE_SEC)
                    self._open_stream()
                    self._paused = False
                    self.get_logger().info("Resumed listening for wake word")

            except Exception as e:
                self.get_logger().error(f"Audio error: {e}")
                time.sleep(0.5)

    def stop(self):
        self._stop = True
        self._close_stream()
        try:
            self.pa.terminate()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WakeNode()
    except Exception as e:
        print(f"Failed to start: {e}")
        return

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
