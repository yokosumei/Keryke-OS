#!/usr/bin/env python3
"""
Brain Node v3 - VLM via Gemini cu STOP sign detection si servo commands.
"""
import json
import os
import subprocess
import tempfile
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

import google.generativeai as genai

try:
    from .audio_devices import find_arecord_device
except ImportError:
    from audio_devices import find_arecord_device

# Numele microfonului cautat in `arecord -l` -- NU hardcoda un numar de
# card (ex. "plughw:3,0"): indexul se schimba de fiecare data cand
# plugi/scoti device-uri USB (dovedit repetat in teste). Suprascrie cu
# KERYKE_MIC_NAME daca ai alt microfon decat AB13X.
MIC_NAME = os.environ.get("KERYKE_MIC_NAME", "AB13X")
SAMPLE_RATE = 16000
RECORD_SEC = 5

GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "")


class BrainNode(Node):
    def __init__(self):
        super().__init__("hive_brain")
        self.cv_bridge = CvBridge()
        self.last_image = None
        self.last_detections = []
        self.last_sensor = {}
        self.last_risk = {}           # RiskDescriptor de la spatial_risk_node
        self.last_stop_alert_time = 0.0

        self.create_subscription(Image, "/perception/image_raw",
                                  self.image_cb, 10)
        self.create_subscription(Detection2DArray, "/perception/detections_yolo",
                                  self.det_cb, 10)
        self.create_subscription(String, "/keryke/sensors",
                                  self.sensor_cb, 10)
        self.create_subscription(String, "/audio/wake_detected",
                                  self.wake_cb, 10)
        self.create_subscription(String, "/keryke/risk",
                                  self.risk_cb, 10)

        self.speak_pub = self.create_publisher(String, "/audio/speak", 10)
        self.status_pub = self.create_publisher(String, "/audio/status", 10)
        self.servo_cmd_pub = self.create_publisher(String, "/servo/command", 10)
        self.vibrate_pub = self.create_publisher(String, "/actuator/vibrate", 10)

        if not GEMINI_API_KEY:
            self.get_logger().error("GEMINI_API_KEY not set!")
            raise RuntimeError("No Gemini API key")

        genai.configure(api_key=GEMINI_API_KEY)
        self.model = genai.GenerativeModel("gemini-2.5-flash")
        self.get_logger().info("Brain v3 ready (STOP detection + servo control)")

        self._busy = False

    def image_cb(self, msg):
        try:
            self.last_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image decode: {e}")

    def det_cb(self, msg):
        items = []
        has_stop_sign = False
        stop_position = None
        stop_size_ratio = 0.0

        for d in msg.detections:
            if not d.results:
                continue
            cls = d.results[0].hypothesis.class_id
            score = d.results[0].hypothesis.score
            if score < 0.45:
                continue

            cx = d.bbox.center.position.x
            cy = d.bbox.center.position.y
            w = d.bbox.size_x
            h = d.bbox.size_y

            if cx < 213:
                pos_x = "stanga"
            elif cx > 426:
                pos_x = "dreapta"
            else:
                pos_x = "centru"

            size_ratio = (w * h) / (640 * 480)
            if size_ratio > 0.25:
                dist_est = "foarte aproape"
            elif size_ratio > 0.08:
                dist_est = "aproape"
            else:
                dist_est = "departe"

            items.append({
                "obiect": cls,
                "pozitie": pos_x,
                "distanta_estimata": dist_est,
                "confidence": round(score, 2),
            })

            # Detect STOP sign
            if cls == "stop sign" and score > 0.55:
                has_stop_sign = True
                stop_position = pos_x
                stop_size_ratio = size_ratio

        self.last_detections = items

        # Trigger STOP sign auto-alert (fara wake, automat)
        if has_stop_sign:
            self._handle_stop_sign(stop_position, stop_size_ratio)

    def _handle_stop_sign(self, position, size_ratio):
        """Auto-handle STOP sign: speak warning + vibrate + servo command."""
        now = time.time()
        if now - self.last_stop_alert_time < 10.0:
            return  # cooldown 10s
        self.last_stop_alert_time = now

        # Decide directia opusa indicatorului
        if position == "stanga":
            servo_dir = "right"
            spoken_dir = "dreapta"
        elif position == "dreapta":
            servo_dir = "left"
            spoken_dir = "stanga"
        else:  # centru
            servo_dir = "left"  # default merge la stanga la STOP central
            spoken_dir = "stanga"

        # Distanta estimata
        if size_ratio > 0.20:
            distanta_text = "foarte aproape, sub un metru"
        elif size_ratio > 0.08:
            distanta_text = "aproape, la cativa metri"
        else:
            distanta_text = "in zare"

        text = (
            f"Atentie! Indicator stop detectat in {position}, {distanta_text}. "
            f"Recomand sa rotesti {spoken_dir}."
        )

        self.get_logger().info(f"STOP SIGN AUTO-ALERT: {text}")

        # Publish actiuni in paralel. STOP = pericol -> servoul se aplica
        # imediat pe ESP32, nu asteapta urmatoarea faza de balans.
        self._speak(text)
        self._send_servo(servo_dir, mode="immediate")
        self._send_vibrate("vibrate_alert")

    def sensor_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        # Doar dict-uri: un JSON valid non-dict ar crapa _build_context.
        if isinstance(data, dict):
            self.last_sensor = data

    def risk_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            self.last_risk = data

    def wake_cb(self, msg):
        if self._busy:
            self.get_logger().warn("Already busy")
            return
        # _busy se seteaza AICI, pe thread-ul executorului (callback-urile
        # sunt serializate), nu in thread-ul pornit -- altfel doua wake-uri
        # in rafala ar trece amandoua de verificare si ar porni doua
        # interogari concurente (doua arecord pe acelasi microfon).
        self._busy = True
        threading.Thread(target=self._handle_query, daemon=True).start()

    def _build_context(self):
        ctx = {"obiecte_detectate_yolo": self.last_detections}
        if self.last_sensor:
            dist_mm = self.last_sensor.get("distance", 0)
            if dist_mm > 0:
                ctx["distanta_obstacol_in_fata_mm"] = dist_mm
                if dist_mm < 500:
                    ctx["alerta_obstacol"] = "FOARTE APROAPE - sub 50cm"
                elif dist_mm < 1000:
                    ctx["alerta_obstacol"] = "aproape - sub 1m"
            yaw = self.last_sensor.get("yaw", 0)
            if abs(yaw) > 0.3:
                ctx["orientare_baston_grade"] = int(yaw * 180 / 3.14159)
            swing = self.last_sensor.get("swing", 0)
            if swing:
                ctx["faza_mers"] = "swing (inertie)"

        # Adaugă RiskDescriptor de la spatial_risk_node (fuziune YOLO+ToF+IMU)
        if self.last_risk:
            risk_level = self.last_risk.get("risk_level", "none")
            ctx["evaluare_risc_spatial"] = {
                "nivel_risc": risk_level,
                "utilizator_in_miscare": self.last_risk.get("user_moving", False),
            }
            primary = self.last_risk.get("primary_obstacle")
            if primary and risk_level in ("medium", "high", "critical"):
                ctx["evaluare_risc_spatial"]["obstacol_principal"] = {
                    "obiect": primary.get("label"),
                    "zona_corp": primary.get("body_zone"),
                    "directie": primary.get("lateral"),
                    "distanta_mm": primary.get("distance_mm"),
                }
            if risk_level in ("high", "critical"):
                ctx["ALERTA_RISC"] = f"NIVEL {risk_level.upper()} - actiune necesara"

        return ctx

    def _handle_query(self):
        # _busy e deja True (setat in wake_cb, inainte de pornirea thread-ului).
        try:
            self._speak("Te ascult.")
            self._set_status("listening")
            time.sleep(1.5)

            self.get_logger().info("Recording 5 sec via arecord...")
            try:
                audio_bytes = self._record_audio(RECORD_SEC)
            except Exception as e:
                self.get_logger().error(f"Record failed: {e}")
                self._speak("Nu am putut inregistra audio.")
                return

            self._set_status("thinking")

            if self.last_image is None:
                self._speak("Nu am imagine. Verifica camera.")
                return

            context = self._build_context()
            self.get_logger().info(
                f"Context: {json.dumps(context, ensure_ascii=False)}"
            )

            try:
                response_text = self._query_gemini(
                    audio_bytes, self.last_image, context
                )
                self.get_logger().info(f"Gemini: {response_text}")

                self._maybe_send_servo_command(response_text)
                self._speak(response_text)
            except Exception as e:
                self.get_logger().error(f"Gemini error: {e}")
                self._speak("Nu am putut procesa cererea.")
        finally:
            # Orice iesire -- inclusiv o exceptie neprevazuta (ex. context
            # malformat) -- readuce status-ul pe idle, altfel bannerul
            # "Ma gandesc..." ramane blocat pe dashboard.
            self._set_status("idle")
            self._busy = False

    def _maybe_send_servo_command(self, text):
        t = text.lower()
        cmd = None
        if any(k in t for k in ["roteste dreapta", "spre dreapta",
                                  "viraj dreapta", "vireaza dreapta"]):
            cmd = "right"
        elif any(k in t for k in ["roteste stanga", "spre stanga",
                                    "viraj stanga", "vireaza stanga"]):
            cmd = "left"
        elif any(k in t for k in ["drept inainte", "drept in fata"]):
            cmd = "center"

        if cmd:
            self._send_servo(cmd)

    def _send_servo(self, cmd, mode="gait_sync"):
        # Ghidarea din raspunsul Gemini nu e pericol iminent -> gait_sync
        # (ESP32 aplica poza la urmatoarea faza de balans, nu instant).
        self.get_logger().info(f"Servo command published: {cmd} ({mode})")
        msg = String()
        msg.data = json.dumps({"pose": cmd, "mode": mode})
        self.servo_cmd_pub.publish(msg)

    def _send_vibrate(self, cmd):
        msg = String()
        msg.data = cmd
        self.vibrate_pub.publish(msg)

    def _record_audio(self, duration_sec):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            device = find_arecord_device(MIC_NAME)
            if not device:
                self.get_logger().warn(
                    f"Nu gasesc cardul '{MIC_NAME}' in `arecord -l` -- "
                    f"folosesc device-ul implicit ALSA."
                )
            cmd = ["arecord"]
            if device:
                cmd += ["-D", device]
            cmd += [
                "-c", "1",
                "-r", str(SAMPLE_RATE),
                "-f", "S16_LE",
                "-d", str(duration_sec),
                "-q",
                tmp_path,
            ]
            self.get_logger().info(f"Running: {' '.join(cmd)}")
            result = subprocess.run(
                cmd, capture_output=True, timeout=duration_sec + 3
            )
            if result.returncode != 0:
                err = result.stderr.decode("utf-8", errors="ignore")
                raise RuntimeError(f"arecord failed: {err}")

            with open(tmp_path, "rb") as f:
                wav_bytes = f.read()

            self.get_logger().info(f"Recorded {len(wav_bytes)} bytes")
            return wav_bytes
        finally:
            try:
                os.remove(tmp_path)
            except Exception:
                pass

    def _query_gemini(self, audio_wav, image_bgr, context):
        ok, jpg = cv2.imencode(".jpg", image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        jpg_bytes = jpg.tobytes()

        system_prompt = f"""Esti Keryke, asistentul vocal AI pentru un baston inteligent destinat persoanelor cu deficiente grave de vedere.

REGULI STRICTE:
- Raspunzi STRICT in limba romana, in 1-2 propozitii scurte si clare.
- Folosesti contextul senzorial de mai jos pentru a fi precis.
- Mentionezi pozitia (stanga/dreapta/centru) si distanta cand e relevant.
- Daca utilizatorul intreaba "ce vezi" sau "ce este in fata", descrii ce vezi in imagine + folosesti date YOLO.
- Daca vezi indicator STOP sau pericol, sugerezi rotire ("roteste stanga" / "roteste dreapta" / "drept inainte").
- NU spui "ca asistent AI" sau "imi cer scuze". Vorbesti direct, scurt, util.
- NU repeti intrebarea.

CONTEXT SENZORIAL CURENT:
{json.dumps(context, ensure_ascii=False, indent=2)}

Acum asculta intrebarea audio + analizeaza imaginea + raspunde."""

        response = self.model.generate_content([
            system_prompt,
            {"mime_type": "audio/wav", "data": audio_wav},
            {"mime_type": "image/jpeg", "data": jpg_bytes},
        ])
        return response.text.strip()

    def _speak(self, text):
        msg = String()
        msg.data = text
        self.speak_pub.publish(msg)

    def _set_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BrainNode()
    except RuntimeError as e:
        print(f"Brain init failed: {e}")
        return

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
