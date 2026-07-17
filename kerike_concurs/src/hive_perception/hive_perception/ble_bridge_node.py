#!/usr/bin/env python3
"""
BLE Bridge Node — puntea ROS2 <-> ESP32 (baston) peste BLE binar.

Înlocuieşte perechea istorică sensor_bridge (UDP in) + servo_command (UDP out):
un singur nod care vorbeşte protocolul binar final din PROTOCOL_BLE.md,
prin clientul de referinţă `firmware/esp32_keryke_ble/pi_client_ble.py`
(clasa KerykeBLEClient). Clientul NU e copiat aici — e încărcat dinamic din
locul lui (sursa de adevăr a protocolului rămâne în cele 3 fişiere lockstep:
esp32_keryke_ble.ino, pi_client_ble.py, PROTOCOL_BLE.md; varianta TCP e
păstrată doar ca istorie).

Consumă:
  /servo/command    (String JSON {"pose": "left"|"right"|"center",
                                  "mode": "gait_sync"|"immediate"};
                     acceptă tolerant şi string simplu "left" -> gait_sync)
  /actuator/vibrate (String — vibrate_alert / vibrate_short)

Publică:
  /keryke/sensors   (String JSON — telemetria decodată; schema definită AICI,
                     câmpurile plate pe care le citesc consumatorii:
                     spatial_risk: distance, accel_mag, gyroZ;
                     brain: distance, yaw, swing)

Semantica accel_mag: |sqrt(ax²+ay²+az²) − 9.81| — deviaţia faţă de gravitaţie,
astfel încât pragul ACCEL_MOVING_THRESHOLD=0.8 m/s² din spatial_risk să separe
real staţionar (≈0) de mers (>0.8). Magnitudinea brută ar fi ≈9.8 şi în repaus.

Punct central Gait-Sync: acest nod transmite doar CE poză şi CÂND-ul ca mod
(gait_sync/immediate) — momentul efectiv al aplicării în modul gait_sync îl
decide ESP32 local, din propriul IMU (detectSwingPhase), tocmai ca jitterul
de transport să nu conteze biomecanic.

Comenzile sosite cât timp bastonul e deconectat se ARUNCĂ (cu warning):
decizia de risc se republică oricum la 10 Hz, iar fail-safe-ul e pe baston
(firmware autonom: TOF_STALE_MS, heartbeat local).

Rulare (necesită ESP32 împerecheat în prealabil cu bluetoothctl — vezi
antetul din pi_client_ble.py):
    ros2 run hive_perception ble_bridge
    ros2 run hive_perception ble_bridge --ros-args -p device:=AA:BB:CC:DD:EE:FF
"""

import asyncio
import importlib.util
import json
import math
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── Parametri impuls vibraţie pe baston (CMD_VIBRATE: intensitate, durată ms) ──
# De calibrat pe dispozitiv; intensitatea e PWM 0..255.
VIBRATE_ALERT_PARAMS = (255, 800)
VIBRATE_SHORT_PARAMS = (140, 250)

GRAVITY_MS2 = 9.81

# Reconectare cu backoff: pornim mic, plafonăm ca să nu spamăm radio-ul.
RECONNECT_INITIAL_S = 3.0
RECONNECT_MAX_S = 15.0

# Căi implicite către clientul BLE de referinţă (locuieşte lângă firmware-ul
# lui, în firmware/esp32_keryke_ble/): întâi cea din container (../firmware
# montat în /ws/firmware), apoi relativ la repo (dezvoltare).
_DEFAULT_CLIENT_PATHS = [
    "/ws/firmware/esp32_keryke_ble/pi_client_ble.py",
    str(Path(__file__).resolve().parents[3]
        / "firmware" / "esp32_keryke_ble" / "pi_client_ble.py"),
]


def _load_ble_client_module(explicit_path: str):
    """Încarcă pi_client_ble.py ca modul, din prima cale existentă."""
    candidates = ([explicit_path] if explicit_path else []) + _DEFAULT_CLIENT_PATHS
    for cand in candidates:
        p = Path(cand)
        if p.is_file():
            spec = importlib.util.spec_from_file_location("pi_client_ble", p)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod, str(p)
    raise FileNotFoundError(
        f"pi_client_ble.py negasit; cai incercate: {candidates}. "
        "Seteaza parametrul ROS 'ble_client_path'."
    )


class BleBridgeNode(Node):
    def __init__(self):
        super().__init__("ble_bridge")

        self.declare_parameter("device", "")            # adresă MAC sau gol -> scan după nume
        self.declare_parameter("ble_client_path", "")   # suprascrie calea către pi_client_ble.py

        explicit_path = self.get_parameter("ble_client_path").value
        self._ble, used_path = _load_ble_client_module(explicit_path)
        self.get_logger().info(f"Client BLE incarcat din {used_path}")

        self._pose_map = {
            "center": self._ble.POSE_CENTER,
            "left":   self._ble.POSE_LEFT,
            "right":  self._ble.POSE_RIGHT,
        }

        # ── ROS I/O ────────────────────────────────────────────────────────────
        self.sensors_pub = self.create_publisher(String, "/keryke/sensors", 10)
        self.create_subscription(String, "/servo/command", self.servo_cb, 10)
        self.create_subscription(String, "/actuator/vibrate", self.vibrate_cb, 10)

        # ── Bucla asyncio/bleak pe thread propriu ─────────────────────────────
        # rclpy îşi are executorul lui; bleak cere un event loop asyncio.
        # Sesiunea (self._session) e non-None DOAR cât timp suntem conectaţi.
        self._session = None
        # KerykeBLEClient._command goleşte coada comună de ACK înainte de
        # fiecare trimitere -- două comenzi concurente (spatial_risk publică
        # servo + vibrate spate-în-spate) şi-ar putea arunca reciproc ACK-ul
        # (timeout fals). Lock-ul serializează comenzile; clientul de
        # referinţă rămâne neatins (e fişier lockstep de protocol).
        self._cmd_lock = asyncio.Lock()
        self._loop = asyncio.new_event_loop()
        self._ble_thread = threading.Thread(
            target=self._run_loop, name="ble-loop", daemon=True
        )
        self._ble_thread.start()
        asyncio.run_coroutine_threadsafe(self._ble_task(), self._loop)

        self.get_logger().info("BLE Bridge pornit — caut bastonul...")

    # ── Thread-ul BLE ──────────────────────────────────────────────────────────

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    async def _ble_task(self):
        """Conectare + reconectare cu backoff, la nesfârşit."""
        backoff = RECONNECT_INITIAL_S
        target = self.get_parameter("device").value or None
        while True:
            try:
                dev = await self._ble.find_device(target)
                if dev is None:
                    raise ConnectionError(
                        "dispozitiv negasit (advertising oprit sau in afara razei)"
                    )
                disconnected = asyncio.Event()
                client = self._ble.BleakClient(
                    dev,
                    disconnected_callback=lambda _c: disconnected.set(),
                )
                await client.connect()
                try:
                    # Împerechere best-effort; pe BlueZ bonding-ul real se face
                    # o singură dată din bluetoothctl (passkey 123456).
                    try:
                        await client.pair()
                    except Exception:
                        pass

                    session = self._ble.KerykeBLEClient(
                        client, on_telemetry=self._on_telemetry
                    )
                    await session.start()
                    self._session = session
                    backoff = RECONNECT_INITIAL_S
                    self.get_logger().info(
                        f"Conectat la baston: {dev.name} [{dev.address}]"
                    )
                    await disconnected.wait()
                finally:
                    self._session = None
                    try:
                        await client.disconnect()
                    except Exception:
                        pass
                self.get_logger().warn("Baston deconectat — reincerc...")
            except Exception as e:
                self.get_logger().warn(
                    f"BLE indisponibil ({e}) — reincerc in {backoff:.0f}s"
                )
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, RECONNECT_MAX_S)

    # ── Telemetrie: BLE -> ROS ─────────────────────────────────────────────────

    def _on_telemetry(self, t, solicited: bool):
        """Rulează pe thread-ul BLE; publish-ul rclpy e sigur din alt thread."""
        accel_mag = abs(math.sqrt(t.ax**2 + t.ay**2 + t.az**2) - GRAVITY_MS2)
        payload = {
            "counter":   t.counter,
            "angle":     t.angle,
            "distance":  t.distance_mm,
            "pitch":     round(t.pitch, 3),
            "roll":      round(t.roll, 3),
            "yaw":       round(t.yaw, 3),
            "swing":     t.swing,
            "servo":     t.servo,
            "vibration": t.vibration,
            "temp":      round(t.temp_c, 1),
            "accel_mag": round(accel_mag, 3),
            "gyroZ":     round(t.gz, 3),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.sensors_pub.publish(msg)

    # ── Comenzi: ROS -> BLE ────────────────────────────────────────────────────

    def _submit(self, make_coro, what: str):
        """
        Trimite o comandă pe bucla BLE; aruncă dacă nu suntem conectaţi.
        make_coro primeşte sesiunea şi întoarce corutina comenzii -- aşa
        verificarea de conexiune stă într-un singur loc, iar corutina nu e
        creată degeaba când bastonul lipseşte.
        """
        session = self._session
        if session is None:
            self.get_logger().warn(
                f"{what} ignorat: bastonul nu e conectat", throttle_duration_sec=5.0
            )
            return
        fut = asyncio.run_coroutine_threadsafe(
            self._run_locked(make_coro(session)), self._loop
        )
        fut.add_done_callback(lambda f: self._log_result(f, what))

    async def _run_locked(self, coro):
        """Serializează comenzile (vezi comentariul de la _cmd_lock)."""
        async with self._cmd_lock:
            return await coro

    def _log_result(self, fut, what: str):
        try:
            fut.result()
        except Exception as e:
            self.get_logger().warn(f"{what} esuat: {e}")

    def servo_cb(self, msg: String):
        raw = msg.data.strip()
        try:
            cmd = json.loads(raw)
            pose_name = cmd.get("pose", "")
            mode = cmd.get("mode", "gait_sync")
        except (json.JSONDecodeError, AttributeError):
            # Fallback tolerant: string simplu ("left") -> gait_sync.
            pose_name, mode = raw, "gait_sync"

        pose = self._pose_map.get(pose_name)
        if pose is None:
            self.get_logger().warn(f"Poza servo necunoscuta: {raw!r}")
            return

        immediate = (mode == "immediate")
        self._submit(
            lambda s: s.servo_pose(pose, immediate=immediate),
            f"servo {pose_name} ({mode})",
        )

    def vibrate_cb(self, msg: String):
        cmd = msg.data.strip()
        if cmd == "vibrate_alert":
            intensity, duration = VIBRATE_ALERT_PARAMS
        elif cmd == "vibrate_short":
            intensity, duration = VIBRATE_SHORT_PARAMS
        else:
            self.get_logger().warn(f"Comanda vibratie necunoscuta: {cmd!r}")
            return

        self._submit(lambda s: s.vibrate(intensity, duration), cmd)

    def destroy_node(self):
        # Fără disconnect explicit: legătura BLE cade odată cu procesul, iar
        # fail-safe-urile autonome ale bastonului (TOF_STALE_MS, revenirea
        # servoului la centru după SERVO_HOLD_MS) acoperă starea rămasă.
        self._loop.call_soon_threadsafe(self._loop.stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BleBridgeNode()
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
