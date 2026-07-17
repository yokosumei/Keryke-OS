#!/usr/bin/env python3
"""
tts_node.py  --  TTS real, pe boxa de pe vesta

Pana acum, /audio/speak (publicat de narrator_node.py si brain_node.py)
ajungea DOAR in browser-ul dashboard-ului (speechSynthesis, JS). Daca
nimeni nu are dashboard-ul deschis intr-un browser, utilizatorul care
poarta vesta nu auzea NIMIC -- nici narratiunea, nici raspunsurile
Gemini. Nodul asta e cel care lipsea: transforma /audio/speak in sunet
REAL pe boxa USB de pe vesta.

Sinteza: espeak-ng (voce romana "ro"). Ales in locul unui TTS neural
(ex. Piper) fiindca nu are nevoie de niciun model descarcat -- sintetic,
instant, functioneaza garantat din prima. Daca vocea suna prea robotic
dupa ce merge de baza, Piper e un upgrade usor de facut mai tarziu
(model Piper romana + inlocuiesti doar _speak()).

Redare: aplay, targetat pe boxa USB dupa NUME (vezi audio_devices.py) --
NU index/numar de card, care se schimba la fiecare hotplug.

INSTALARE (in Dockerfile, containerul hive):
    apt-get install espeak-ng alsa-utils

RULARE:
    ros2 run hive_perception tts
    -> asculta /audio/speak, vorbeste pe boxa USB (cauta "USB" implicit,
       suprascrie cu KERYKE_SPEAKER_NAME daca ai mai multe device-uri
       audio USB si prinde pe cel gresit -- verifica intai cu `aplay -l`).
"""
from __future__ import annotations

import os
import queue
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from .audio_devices import find_aplay_device
except ImportError:
    from audio_devices import find_aplay_device

# Numele boxei cautat in `aplay -l` -- suprascrie cu KERYKE_SPEAKER_NAME
# daca ai mai multe device-uri audio USB conectate (verifica `aplay -l`
# pe Pi si pune substring-ul corect).
SPEAKER_NAME = os.environ.get("KERYKE_SPEAKER_NAME", "USB")
ESPEAK_VOICE = "ro"
ESPEAK_SPEED = 165  # cuvinte/minut -- usor sub implicit (175), pt claritate


class TtsNode(Node):
    def __init__(self):
        super().__init__("hive_tts")

        self.device = find_aplay_device(SPEAKER_NAME)
        if self.device:
            self.get_logger().info(f"Boxa gasita: {self.device}")
        else:
            self.get_logger().warn(
                f"Nu gasesc boxa '{SPEAKER_NAME}' in `aplay -l` -- folosesc "
                f"device-ul implicit ALSA, risc sa iasa sunet pe boxa gresita "
                f"(sau deloc). Verifica `aplay -l` si seteaza KERYKE_SPEAKER_NAME."
            )

        self._queue: "queue.Queue[str]" = queue.Queue(maxsize=20)
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

        self.create_subscription(String, "/audio/speak", self._on_speak, 10)
        self.get_logger().info("TtsNode pornit, ascult /audio/speak.")

    def _on_speak(self, msg: String) -> None:
        try:
            self._queue.put_nowait(msg.data)
        except queue.Full:
            # coada plina -- arunca cel mai vechi mesaj nerostit inca,
            # mai bine sa spuna ceva actual decat sa se acumuleze intarziere
            self.get_logger().warn("Coada TTS plina, arunc mesajul cel mai vechi.")
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(msg.data)
            except queue.Full:
                pass

    def _worker(self) -> None:
        while rclpy.ok():
            text = self._queue.get()
            self._speak(text)

    def _speak(self, text: str) -> None:
        if not text:
            return
        espeak_cmd = ["espeak-ng", "-v", ESPEAK_VOICE, "-s", str(ESPEAK_SPEED),
                      "--stdout", text]
        aplay_cmd = ["aplay", "-q"]
        if self.device:
            aplay_cmd += ["-D", self.device]

        try:
            espeak_proc = subprocess.Popen(espeak_cmd, stdout=subprocess.PIPE)
            subprocess.run(aplay_cmd, stdin=espeak_proc.stdout, check=True)
            espeak_proc.stdout.close()
            espeak_proc.wait()
        except FileNotFoundError as e:
            self.get_logger().error(
                f"Lipseste executabilul ({e}) -- verifica ca espeak-ng si "
                f"alsa-utils sunt instalate in container."
            )
        except Exception as e:
            self.get_logger().error(f"Eroare TTS: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
