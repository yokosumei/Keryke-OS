#!/usr/bin/env python3
"""
HIVE Narrator - citeste /perception/detections_yolo, decide ce sa spuna in romana.
Se muteaza INSTANT la /audio/wake_detected si ramane mut pana cand
status devine "idle" + 8 secunde pauza.

Alertele PRECISE (distanta + directie, ex. "persoana la 350 milimetri in
dreapta, ne miscam in stanga") vin din /keryke/risk (RiskDescriptor de
la spatial_risk_node.py) -- DETERMINIST, fara niciun apel Gemini. Un
obstacol trebuie anuntat instant, nu dupa 1-3s de latenta API (si nu
costa un apel API de fiecare data cand cineva trece prin fata camerei).
Descrierea generica ("Vad o persoana in fata") ramane fallback-ul din
det_cb, folosit doar cand nu e nimic urgent de raportat (risk_level none).
"""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

LATERAL_RO = {"left": "stanga", "right": "dreapta", "center": "fata"}


RO_LABELS = {
    "person": "o persoana", "bicycle": "o bicicleta", "car": "o masina",
    "motorcycle": "o motocicleta", "bus": "un autobuz", "truck": "un camion",
    "traffic light": "un semafor", "stop sign": "un indicator stop",
    "bench": "o banca", "bird": "o pasare", "cat": "o pisica",
    "dog": "un caine", "backpack": "un rucsac", "umbrella": "o umbrela",
    "bottle": "o sticla", "cup": "o cana", "chair": "un scaun",
    "couch": "o canapea", "potted plant": "o planta", "tv": "un televizor",
    "laptop": "un laptop", "mouse": "un mouse", "keyboard": "o tastatura",
    "cell phone": "un telefon", "book": "o carte", "clock": "un ceas",
}
ALERT_CLASSES = {"car", "motorcycle", "bus", "truck", "bicycle", "person"}

SILENCE_AFTER_IDLE_SEC = 8.0
MAX_MUTE_SEC = 30.0


class NarratorNode(Node):
    def __init__(self):
        super().__init__("hive_narrator")
        self.create_subscription(Detection2DArray, "/perception/detections_yolo",
                                  self.det_cb, 10)
        self.create_subscription(String, "/keryke/risk", self.risk_cb, 10)
        self.create_subscription(String, "/audio/status", self.status_cb, 10)
        self.create_subscription(String, "/audio/wake_detected",
                                  self.wake_cb, 10)
        self.speak_pub = self.create_publisher(String, "/audio/speak", 10)

        self.cooldown_normal = 4.0
        self.cooldown_alert = 2.0
        self.cooldown_risk = 3.0
        self.last_speak_time = 0.0
        self.last_announced = set()
        self.last_risk_level = "none"
        self.last_risk_speak_time = 0.0

        self.muted = False
        self.muted_at = 0.0
        self.unmute_at = 0.0

        self.get_logger().info("Narrator started (mutes on wake)")

    def wake_cb(self, msg):
        """INSTANT mute la wake word."""
        self.muted = True
        self.muted_at = time.time()
        self.unmute_at = 0.0
        self.get_logger().info("Narrator MUTED (wake detected)")

    def status_cb(self, msg):
        status = msg.data.strip().lower()
        now = time.time()
        if status in ("listening", "thinking"):
            self.muted = True
            self.muted_at = now
            self.unmute_at = 0.0
        elif status == "idle":
            self.unmute_at = now + SILENCE_AFTER_IDLE_SEC
            self.get_logger().info(
                f"Narrator will unmute in {SILENCE_AFTER_IDLE_SEC}s"
            )

    def _is_muted(self):
        now = time.time()
        if self.muted and (now - self.muted_at > MAX_MUTE_SEC):
            self.get_logger().warn("Mute timeout, force unmute")
            self.muted = False
            self.unmute_at = 0.0
            return False
        if self.unmute_at > 0 and now >= self.unmute_at:
            self.muted = False
            self.unmute_at = 0.0
            self.get_logger().info("Narrator resumed")
        return self.muted

    def det_cb(self, msg):
        if self._is_muted():
            return

        # Ceva mai urgent (obstacol/audio/zona nesigura) e deja raportat
        # precis de risk_cb -- descrierea generica ramane doar fallback
        # ambiental, ca sa nu se calce pe picioare cu alerta precisa.
        if self.last_risk_level != "none":
            return

        now = time.time()
        current_classes = set()
        for d in msg.detections:
            if not d.results:
                continue
            cls = d.results[0].hypothesis.class_id
            score = d.results[0].hypothesis.score
            if score >= 0.55:
                current_classes.add(cls)

        if not current_classes:
            self.last_announced = set()
            return

        new_classes = current_classes - self.last_announced
        has_alert = bool(current_classes & ALERT_CLASSES)
        cooldown = self.cooldown_alert if has_alert else self.cooldown_normal
        if now - self.last_speak_time < cooldown:
            return
        if not new_classes and current_classes == self.last_announced:
            return

        items = [RO_LABELS.get(c, c) for c in sorted(current_classes)]
        if len(items) == 1:
            text = f"Vad {items[0]} in fata."
        elif len(items) == 2:
            text = f"Vad {items[0]} si {items[1]}."
        else:
            text = "Vad " + ", ".join(items[:-1]) + f" si {items[-1]}."
        if has_alert:
            alert_items = [RO_LABELS.get(c, c)
                           for c in sorted(current_classes & ALERT_CLASSES)]
            text = f"Atentie, {alert_items[0]} aproape. " + text

        self._speak(text)
        self.last_speak_time = now
        self.last_announced = current_classes

    def risk_cb(self, msg):
        """
        Alerte PRECISE (distanta + directie), din spatial_risk_node.py --
        determinist, fara Gemini. Ex: "Persoana la 350 milimetri in
        dreapta, ne miscam in stanga."
        """
        try:
            risk = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return

        self.last_risk_level = risk.get("risk_level", "none")

        if self._is_muted():
            return

        action = risk.get("action", "none")
        if action == "none":
            return

        now = time.time()
        if now - self.last_risk_speak_time < self.cooldown_risk:
            return

        text = self._build_risk_text(risk, action)
        if not text:
            return

        self._speak(text)
        self.last_risk_speak_time = now
        self.last_speak_time = now

    def _build_risk_text(self, risk, action):
        primary = risk.get("primary_obstacle")
        audio_alert = risk.get("audio_alert")
        walkable = risk.get("walkable_status")

        if primary and action in ("servo_left", "servo_right", "servo_center_stop"):
            label_ro = RO_LABELS.get(primary["label"], primary["label"])
            dist = primary["distance_mm"]
            lateral_ro = LATERAL_RO.get(primary["lateral"], primary["lateral"])
            if action == "servo_center_stop":
                return f"{label_ro.capitalize()} la {dist} milimetri in fata. Opreste-te."
            move_ro = "stanga" if action == "servo_left" else "dreapta"
            return (f"{label_ro.capitalize()} la {dist} milimetri in {lateral_ro}, "
                    f"ne miscam in {move_ro}.")

        if audio_alert and action == "vibrate_alert":
            return f"Atentie, {audio_alert.get('tip', 'sunet de pericol')}."

        if walkable and not walkable.get("on_path", True) and action == "vibrate_short":
            return "Atentie, nu ai pe unde sa mergi."

        return None

    def _speak(self, text):
        msg_out = String()
        msg_out.data = text
        self.speak_pub.publish(msg_out)
        self.get_logger().info(f"Speak: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = NarratorNode()
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
