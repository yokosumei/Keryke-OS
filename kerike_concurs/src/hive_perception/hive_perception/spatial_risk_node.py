#!/usr/bin/env python3
"""
Spatial Risk Node — fuziune YOLO + ToF + IMU pentru Keryke-OS.

Consumă:
  /perception/detections_yolo (Detection2DArray — YOLO de la yolo_detection_node)
  /keryke/sensors            (String JSON        — ToF + IMU de la sensor_bridge)
  /audio/alerts              (String JSON        — alerte YAMNet de la audio_event_node)
  /perception/walkable_status (String JSON       — stare zonă sigură de la yolo_segmentation_node)

Publică:
  /keryke/risk            (String JSON        — RiskDescriptor structurat)
  /servo/command          (String JSON        — {"pose": left/right/center, "mode": gait_sync/immediate})
  /actuator/vibrate       (String             — vibrate_alert / vibrate_short)

O SINGURĂ decizie pe ciclu, DOUĂ canale de ieşire cu ritmuri diferite
(vezi ARHITECTURA_SISTEM.md §3.4):
  - bastonul (/servo/command): CE poză decide Pi-ul aici; CÂND se aplică
    decide ESP32 local, din propriul IMU — "gait_sync" aşteaptă următoarea
    fază de balans, "immediate" (critical/high) aplică instant;
  - vesta (/keryke/risk → haptic_vest_node): feedback direcţional instant,
    din compass_azimuth_deg + risk_level, fără constrângere biomecanică.

Logica de fuziune (InfoEducaţie — fără EKF):
  1. Clasifică fiecare detecţie YOLO în zona de corp: ankle / knee / torso / head
     pe baza poziţiei Y relative în frame (640×480).
  2. Combină distanţa ToF cu mărimea bounding box pentru a estima riscul real.
  3. Foloseşte IMU (accelMag, gyroZ) pentru a detecta dacă utilizatorul e în mers
     sau staționar și modulează pragurile corespunzător.
  4. Emite o singură comandă de acţiune per ciclu, cu PRIORITATE EXPLICITĂ:
     obstacol critic > alertă audio de pericol > obstacol high > zonă
     nesigură (segmentare) > obstacol medium > alertă audio de atenţie >
     obstacol low. Un pericol real (obstacol sau sunet) anulează orice
     ghidare spre zonă sigură sau ţintă — nu te duce spre pericol doar
     fiindcă acolo ai vrea să mergi.

RiskDescriptor JSON:
{
  "risk_level": "none" | "low" | "medium" | "high" | "critical",
  "primary_obstacle": {
    "label": str,
    "body_zone": "ankle" | "knee" | "torso" | "head",
    "lateral": "left" | "center" | "right",
    "distance_mm": int,       # ToF dacă e disponibil, altfel estimat din bbox
    "distance_source": "tof" | "bbox_estimate",
    "confidence": float
  } | null,
  "all_obstacles": [...],
  "user_moving": bool,
  "imu_swing": bool,
  "audio_alert": {"tip": str, "nivel": str, "scor": float} | null,
  "walkable_status": {"on_path": bool, "azimuth_deg": float | null, "message": str | null} | null,
  "action": "none" | "vibrate_short" | "vibrate_alert" | "servo_left" | "servo_right" | "servo_center_stop",
  "servo_mode": "gait_sync" | "immediate",       # CÂND aplică bastonul poza (ESP32 local)
  "compass_azimuth_deg": float | null,           # azimut continuu pt vesta-busolă (relativ, nu cardinal);
                                                 #   din obstacol DOAR la risc critical/high ("pericol acolo"),
                                                 #   altfel din drum ("mergi încolo") -- un singur sens per semnal
  "compass_source": "obstacle" | "path" | null,  # de unde vine azimutul
  "timestamp": float
}
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

try:
    from .perception_geometry import azimuth_from_bbox_center
except ImportError:
    from perception_geometry import azimuth_from_bbox_center

# ── Parametri cadru ────────────────────────────────────────────────────────────
FRAME_W = 640
FRAME_H = 480

# ── Zone de corp: limitele Y relative (0.0 = sus, 1.0 = jos) ──────────────────
# Camera e montată la nivelul pieptului, orientată înainte-jos uşor.
# Obiectele din partea de jos a frame-ului sunt la nivelul gleznelor.
ZONE_ANKLE  = (0.70, 1.00)   # y_rel > 0.70
ZONE_KNEE   = (0.45, 0.70)
ZONE_TORSO  = (0.20, 0.45)
ZONE_HEAD   = (0.00, 0.20)

# ── Praguri distanţă ToF (mm) ──────────────────────────────────────────────────
DIST_CRITICAL = 400    # < 40 cm → critical
DIST_HIGH     = 700    # < 70 cm → high
DIST_MEDIUM   = 1200   # < 120 cm → medium
DIST_LOW      = 2000   # < 200 cm → low

# ── Praguri bbox size ratio pentru estimare distanţă când ToF lipseste ─────────
BBOX_VERY_CLOSE = 0.20   # > 20% din frame → ~<50cm
BBOX_CLOSE      = 0.08   # > 8%  → ~<100cm
BBOX_NEAR       = 0.03   # > 3%  → ~<200cm

# ── IMU: detecţie mers ─────────────────────────────────────────────────────────
ACCEL_MOVING_THRESHOLD = 0.8   # m/s² — sub asta, utilizatorul e staționar
GYRO_SWING_THRESHOLD   = 0.3   # rad/s

# ── Cooldown-uri acţiuni (secunde) ────────────────────────────────────────────
COOLDOWN_CRITICAL = 2.0
COOLDOWN_HIGH     = 4.0
COOLDOWN_MEDIUM   = 6.0
COOLDOWN_LOW      = 10.0

# ── Clase care nu generează alertă (fundal) ───────────────────────────────────
BACKGROUND_CLASSES = {"sky", "road", "sidewalk", "wall", "floor", "ceiling"}

# ── Clase cu prioritate ridicată ──────────────────────────────────────────────
HIGH_PRIORITY_CLASSES = {"person", "car", "motorcycle", "bus", "truck",
                          "bicycle", "stop sign", "traffic light"}

# ── Alerte audio (YAMNet) / zonă sigură (segmentare): staleness ───────────────
# Ambele topic-uri publică periodic (audio ~1s/fereastră) -- daca ultimul
# mesaj primit e mai vechi decat pragul, il tratam ca inactiv (nu blocam
# risc-ul la nesfarsit pe un semnal care nu mai vine).
AUDIO_ALERT_STALE_S = 2.5
WALKABLE_STALE_S = 2.5

# ── Senzori baston (/keryke/sensors, BLE): staleness ──────────────────────────
# Deconectarile BLE sunt mod normal de functionare (reconectare cu backoff in
# ble_bridge) -- fara prag, ultima distanta ToF ar ramane inghetata la
# nesfarsit si ar tine un "critical" fals sau ar masca un obstacol real.
# "No data ≠ semnal": dupa prag, distanta = 0 (necunoscut) si IMU absent,
# identic cu situatia in care bastonul nu a fost conectat deloc.
SENSOR_STALE_S = 2.5

# Nivel de risc echivalent pentru alertele audio YAMNet, pentru raportare
# in RiskDescriptor.risk_level (decizia de ACTIUNE foloseste prioritatea
# explicita din _decide_action, nu doar acest maxim).
AUDIO_RISK_BY_LEVEL = {"pericol": "critical", "atentie": "medium", "info": "none"}

# Nivel de risc echivalent cand utilizatorul nu e pe zona sigura (segmentare).
OFF_PATH_RISK_LEVEL = "medium"


def _lateral_zone(cx: float) -> str:
    """Împarte frame-ul în 3 zone laterale."""
    if cx < FRAME_W / 3:
        return "left"
    elif cx > 2 * FRAME_W / 3:
        return "right"
    return "center"


def _body_zone(cy: float) -> str:
    """Determină zona de corp pe baza poziţiei Y în frame."""
    y_rel = cy / FRAME_H
    if ZONE_ANKLE[0] <= y_rel:
        return "ankle"
    elif ZONE_KNEE[0] <= y_rel:
        return "knee"
    elif ZONE_TORSO[0] <= y_rel:
        return "torso"
    return "head"


def _bbox_distance_estimate(size_ratio: float) -> int:
    """Estimare rudimentară distanţă din mărimea bbox (mm)."""
    if size_ratio > BBOX_VERY_CLOSE:
        return 350
    elif size_ratio > BBOX_CLOSE:
        return 750
    elif size_ratio > BBOX_NEAR:
        return 1500
    return 2500


def _risk_from_distance(dist_mm: int) -> str:
    if dist_mm < DIST_CRITICAL:
        return "critical"
    elif dist_mm < DIST_HIGH:
        return "high"
    elif dist_mm < DIST_MEDIUM:
        return "medium"
    elif dist_mm < DIST_LOW:
        return "low"
    return "none"


def _risk_level_value(level: str) -> int:
    return {"none": 0, "low": 1, "medium": 2, "high": 3, "critical": 4}.get(level, 0)


class SpatialRiskNode(Node):
    def __init__(self):
        super().__init__("spatial_risk")

        # ── State ──────────────────────────────────────────────────────────────
        self.last_detections: list = []
        self.last_sensor: dict = {}
        self.last_sensor_time: float = 0.0
        self._sensor_fresh_prev: bool = False   # pt. log DOAR la tranzitii
        self.last_action_time: float = 0.0
        self.last_risk_level: str = "none"
        self.last_audio_alert: dict | None = None
        self.last_audio_alert_time: float = 0.0
        self.last_walkable: dict | None = None
        self.last_walkable_time: float = 0.0

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(
            Detection2DArray, "/perception/detections_yolo", self.det_cb, 10
        )
        self.create_subscription(
            String, "/keryke/sensors", self.sensor_cb, 10
        )
        self.create_subscription(
            String, "/audio/alerts", self.audio_alert_cb, 10
        )
        self.create_subscription(
            String, "/perception/walkable_status", self.walkable_cb, 10
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        self.risk_pub    = self.create_publisher(String, "/keryke/risk", 10)
        self.servo_pub   = self.create_publisher(String, "/servo/command", 10)
        self.vibrate_pub = self.create_publisher(String, "/actuator/vibrate", 10)

        # ── Timer principal 10 Hz ──────────────────────────────────────────────
        self.create_timer(0.10, self.evaluate_risk)

        self.get_logger().info("Spatial Risk Node pornit (YOLO + ToF + IMU)")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def det_cb(self, msg: Detection2DArray):
        items = []
        for d in msg.detections:
            if not d.results:
                continue
            label = d.results[0].hypothesis.class_id
            score = d.results[0].hypothesis.score
            if score < 0.40:
                continue
            if label in BACKGROUND_CLASSES:
                continue

            cx = d.bbox.center.position.x
            cy = d.bbox.center.position.y
            w  = d.bbox.size_x
            h  = d.bbox.size_y

            size_ratio = (w * h) / (FRAME_W * FRAME_H)

            items.append({
                "label":      label,
                "score":      round(score, 3),
                "cx":         cx,
                "cy":         cy,
                "size_ratio": size_ratio,
            })
        self.last_detections = items

    def sensor_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return
        if isinstance(data, dict):
            self.last_sensor = data
            self.last_sensor_time = time.time()

    def audio_alert_cb(self, msg: String):
        try:
            self.last_audio_alert = json.loads(msg.data)
            self.last_audio_alert_time = time.time()
        except (json.JSONDecodeError, ValueError):
            pass

    def walkable_cb(self, msg: String):
        try:
            self.last_walkable = json.loads(msg.data)
            self.last_walkable_time = time.time()
        except (json.JSONDecodeError, ValueError):
            pass

    # ── Logica principală ──────────────────────────────────────────────────────

    def evaluate_risk(self):
        # Senzorii bastonului sunt valizi doar daca sunt proaspeti (BLE se
        # poate deconecta oricand) -- altfel ii tratam ca absenti, nu inghetati.
        sensor_fresh = (time.time() - self.last_sensor_time) < SENSOR_STALE_S
        if sensor_fresh != self._sensor_fresh_prev:
            # O data pe tranzitie, nu periodic -- degradarea nu ramane tacuta.
            if sensor_fresh:
                self.get_logger().info("Senzorii bastonului: date proaspete (BLE)")
            else:
                self.get_logger().warn(
                    f"Senzorii bastonului stale (>{SENSOR_STALE_S}s) -- "
                    "distanta ToF si IMU tratate ca absente"
                )
            self._sensor_fresh_prev = sensor_fresh
        sensor = self.last_sensor if sensor_fresh else {}
        tof_dist_mm  = sensor.get("distance", 0)
        accel_mag    = sensor.get("accel_mag", 0.0)
        gyro_z       = sensor.get("gyroZ", 0.0)

        # Detecţie mers din IMU
        user_moving = accel_mag > ACCEL_MOVING_THRESHOLD
        imu_swing   = abs(gyro_z) > GYRO_SWING_THRESHOLD

        # ── Construieşte lista de obstacole ───────────────────────────────────
        obstacles = []
        for det in self.last_detections:
            label      = det["label"]
            score      = det["score"]
            cx         = det["cx"]
            cy         = det["cy"]
            size_ratio = det["size_ratio"]

            # Estimare distanţă
            if tof_dist_mm > 0 and det["label"] in HIGH_PRIORITY_CLASSES:
                # Folosim ToF pentru obiectele importante din centru
                lateral = _lateral_zone(cx)
                if lateral == "center":
                    dist_mm = tof_dist_mm
                    dist_src = "tof"
                else:
                    # ToF indică ce e în centru, bbox pentru lateral
                    dist_mm  = _bbox_distance_estimate(size_ratio)
                    dist_src = "bbox_estimate"
            else:
                dist_mm  = _bbox_distance_estimate(size_ratio)
                dist_src = "bbox_estimate"

            # Modulare cu IMU: dacă utilizatorul nu se mişcă, reducem urgenţa
            if not user_moving and dist_mm > 500:
                dist_mm = int(dist_mm * 1.3)  # scade riscul dacă e staționar

            risk_level = _risk_from_distance(dist_mm)

            # Boost risc pentru clase cu prioritate ridicată
            if label in HIGH_PRIORITY_CLASSES and risk_level == "low":
                risk_level = "medium"

            obstacles.append({
                "label":           label,
                "body_zone":       _body_zone(cy),
                "lateral":         _lateral_zone(cx),
                "cx":              cx,   # păstrat pt azimutul continuu al vestei
                "distance_mm":     dist_mm,
                "distance_source": dist_src,
                "confidence":      score,
                "risk_level":      risk_level,
            })

        # ── Selectează obstacol primar (cel mai periculos) ────────────────────
        if obstacles:
            primary = max(
                obstacles,
                key=lambda o: (
                    _risk_level_value(o["risk_level"]) * 10
                    + (2 if o["label"] in HIGH_PRIORITY_CLASSES else 0)
                    + o["confidence"]
                )
            )
            overall_risk = primary["risk_level"]
        else:
            primary      = None
            overall_risk = "none"

        # ── Alertă audio (YAMNet) -- ignorată dacă e mai veche decât pragul ───
        audio_alert = None
        if self.last_audio_alert and \
           (time.time() - self.last_audio_alert_time) < AUDIO_ALERT_STALE_S:
            audio_alert = self.last_audio_alert
        audio_risk = AUDIO_RISK_BY_LEVEL.get(
            (audio_alert or {}).get("nivel"), "none"
        )

        # ── Zonă sigură (segmentare) -- ignorată dacă e mai veche decât pragul ─
        walkable = None
        if self.last_walkable and \
           (time.time() - self.last_walkable_time) < WALKABLE_STALE_S:
            walkable = self.last_walkable
        off_path_risk = (
            OFF_PATH_RISK_LEVEL if walkable and not walkable.get("on_path", True)
            else "none"
        )

        # ── Risc raportat: maximul dintre obstacol / audio / zonă sigură ──────
        # (doar pentru RiskDescriptor.risk_level si cooldown -- decizia de
        # ACTIUNE foloseste prioritatea explicita din _decide_action, nu un
        # simplu maxim numeric intre semnale de natura diferita)
        reported_risk = max(
            [overall_risk, audio_risk, off_path_risk], key=_risk_level_value
        )

        # ── Decide acţiunea ───────────────────────────────────────────────────
        action, reason = self._decide_action(
            primary, overall_risk, user_moving, audio_alert, walkable
        )

        # ── Modul de aplicare pe baston (ARHITECTURA_SISTEM.md §3.4.1) ────────
        # Pericol iminent (critical/high): siguranţa bate confortul ritmic —
        # ESP32 aplică poza instant. Altfel: gait_sync — poza se aplică la
        # următoarea fază de balans detectată local din IMU-ul bastonului.
        servo_mode = (
            "immediate" if reported_risk in ("critical", "high") else "gait_sync"
        )

        # ── Azimut continuu pentru vesta-busolă ───────────────────────────────
        # Obstacolul dă azimutul DOAR când e pericol real (critical/high) --
        # atunci semnalul vestei înseamnă "pericol acolo". Sub high, azimutul
        # vine din drum ("mergi încolo"): un singur sens per semnal, altfel
        # vesta ar ghida utilizatorul SPRE un obstacol benign. Relativ la
        # cameră (stânga negativ / dreapta pozitiv), NU cardinal.
        if primary is not None and overall_risk in ("critical", "high"):
            compass_azimuth = round(
                azimuth_from_bbox_center(primary["cx"], FRAME_W), 1
            )
            compass_source = "obstacle"
        elif walkable and walkable.get("on_path") and \
                walkable.get("azimuth_deg") is not None:
            compass_azimuth = round(walkable["azimuth_deg"], 1)
            compass_source = "path"
        else:
            compass_azimuth = None
            compass_source = None

        # ── Publică RiskDescriptor ─────────────────────────────────────────────
        descriptor = {
            "risk_level":          reported_risk,
            "primary_obstacle":    primary,
            "all_obstacles":       obstacles,
            "user_moving":         user_moving,
            "imu_swing":           imu_swing,
            "audio_alert":         audio_alert,
            "walkable_status":     walkable,
            "action":              action,
            "reason":              reason,
            "servo_mode":          servo_mode,
            "compass_azimuth_deg": compass_azimuth,
            "compass_source":      compass_source,
            "tof_raw_mm":          tof_dist_mm,
            "timestamp":           time.time(),
        }

        msg = String()
        msg.data = json.dumps(descriptor, ensure_ascii=False)
        self.risk_pub.publish(msg)

        # ── Execută acţiunea dacă e cazul ─────────────────────────────────────
        self._execute_action(action, reported_risk, servo_mode)
        self.last_risk_level = reported_risk

    def _decide_action(self, primary, risk_level: str, user_moving: bool,
                       audio_alert: dict | None = None,
                       walkable: dict | None = None) -> tuple[str, str]:
        """
        Decide o singură acţiune per ciclu, cu PRIORITATE EXPLICITĂ (nu un
        simplu maxim numeric între semnale de natură diferită):

          1. Obstacol vizual CRITIC (coliziune iminentă)
          2. Alertă audio PERICOL (sirenă/claxon/urgenţă -- unghi mort ~280°)
          3. Obstacol vizual HIGH
          4. Nu eşti pe zonă sigură (segmentare) -- ghidare spre zona sigură
          5. Obstacol vizual MEDIUM
          6. Alertă audio ATENŢIE (vehicul obişnuit, sonerie bicicletă)
          7. Obstacol vizual LOW, dacă eşti în mers -- preventiv

        Un pericol real (obstacol critic sau alertă audio de pericol) bate
        orice ghidare spre zonă sigură sau ţintă -- nu te duce spre pericol
        doar fiindcă acolo ai vrea să mergi.

        Intoarce (actiune, motiv) -- motivul e text explicativ, pentru
        dashboard/depanare, gandit sa raspunda direct la "de ce a decis asta".
        """
        lateral = primary["lateral"] if primary is not None else None

        # 1) Obstacol critic
        if primary is not None and risk_level == "critical":
            label, dist = primary["label"], primary["distance_mm"]
            if lateral == "left":
                return "servo_right", f"Obstacol CRITIC: {label} la {dist}mm in stanga -> servo dreapta"
            elif lateral == "right":
                return "servo_left", f"Obstacol CRITIC: {label} la {dist}mm in dreapta -> servo stanga"
            return "servo_center_stop", f"Obstacol CRITIC: {label} la {dist}mm in fata -> STOP"

        # 2) Alertă audio de pericol
        if audio_alert and audio_alert.get("nivel") == "pericol":
            tip = audio_alert.get("tip", "necunoscut")
            return "vibrate_alert", f"Alerta sunet PERICOL: {tip} -> vibratie alerta"

        # 3) Obstacol high
        if primary is not None and risk_level == "high":
            label, dist = primary["label"], primary["distance_mm"]
            if lateral == "center":
                return "vibrate_alert", f"Obstacol apropiat: {label} la {dist}mm in fata -> vibratie alerta"
            elif lateral == "left":
                return "servo_right", f"Obstacol apropiat: {label} la {dist}mm in stanga -> servo dreapta"
            return "servo_left", f"Obstacol apropiat: {label} la {dist}mm in dreapta -> servo stanga"

        # 4) Nu eşti pe zonă sigură -- segmentarea e semnal binar (siguranta),
        # nu ghidare de directie (aia vine din YOLO object detection).
        if walkable and not walkable.get("on_path", True):
            return "vibrate_short", "Nu esti pe zona sigura -- vibratie scurta"

        # 5) Obstacol medium
        if primary is not None and risk_level == "medium":
            label, dist = primary["label"], primary["distance_mm"]
            return "vibrate_short", f"Obstacol la distanta medie: {label} la {dist}mm -> vibratie scurta"

        # 6) Alertă audio de atenţie
        if audio_alert and audio_alert.get("nivel") == "atentie":
            tip = audio_alert.get("tip", "necunoscut")
            return "vibrate_short", f"Alerta sunet: {tip} -> vibratie scurta"

        # 7) Obstacol low, doar dacă utilizatorul e în mers
        if primary is not None and risk_level == "low" and user_moving:
            label = primary["label"]
            return "vibrate_short", f"Obstacol indepartat: {label}, esti in mers -> vibratie scurta preventiva"

        return "none", "Fara pericol detectat -- nicio actiune"

    def _execute_action(self, action: str, risk_level: str, servo_mode: str):
        """Execută acţiunea cu cooldown corespunzător nivelului de risc."""
        if action == "none":
            return

        now = time.time()
        cooldown = {
            "critical": COOLDOWN_CRITICAL,
            "high":     COOLDOWN_HIGH,
            "medium":   COOLDOWN_MEDIUM,
            "low":      COOLDOWN_LOW,
        }.get(risk_level, COOLDOWN_MEDIUM)

        if now - self.last_action_time < cooldown:
            return  # în cooldown

        self.last_action_time = now

        if action == "servo_left":
            self._pub_servo("left", servo_mode)
            self._pub_vibrate("vibrate_short")
            self.get_logger().info(f"RISK {risk_level}: servo LEFT ({servo_mode})")

        elif action == "servo_right":
            self._pub_servo("right", servo_mode)
            self._pub_vibrate("vibrate_short")
            self.get_logger().info(f"RISK {risk_level}: servo RIGHT ({servo_mode})")

        elif action == "servo_center_stop":
            self._pub_servo("center", servo_mode)
            self._pub_vibrate("vibrate_alert")
            self.get_logger().warn(f"RISK {risk_level}: STOP + vibrate_alert")

        elif action == "vibrate_alert":
            self._pub_vibrate("vibrate_alert")
            self.get_logger().warn(f"RISK {risk_level}: vibrate_alert")

        elif action == "vibrate_short":
            self._pub_vibrate("vibrate_short")
            self.get_logger().info(f"RISK {risk_level}: vibrate_short")

    def _pub_servo(self, pose: str, mode: str):
        # Payload JSON — puntea BLE îl traduce în CMD_SERVO_POSE(poză, mod);
        # modul spune ESP32-ului CÂND aplică (gait_sync = la următorul swing).
        msg = String()
        msg.data = json.dumps({"pose": pose, "mode": mode})
        self.servo_pub.publish(msg)

    def _pub_vibrate(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.vibrate_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpatialRiskNode()
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
