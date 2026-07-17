#!/usr/bin/env python3
"""
Haptic Vest Node — vesta ca busolă haptică (ARHITECTURA_SISTEM.md §3.4).

Al doilea "afişaj" al ACELEIAŞI decizii de percepţie: citeşte RiskDescriptor
de pe /keryke/risk (10 Hz, fără cooldown) şi îl traduce în vibraţie
direcţională pe piele — instant, la rata percepţiei, fără constrângerea
biomecanică a bastonului (care aşteaptă faza de balans). Nodul NU re-decide
nimic: prioritatea e citită din câmpurile deja decise de
spatial_risk_node._decide_action().

Hardware: 5 motoare LRA pe PCA9685 (I2C, comandate local de Pi — NU trec
prin ESP32 şi nici prin protocolul baston<->vestă). Canalele implicite
[10, 11, 14, 12, 13] = stânga-extremă -> dreapta-extremă, în ordinea fizică
de pe vestă (confirmate pe montajul real; configurabile prin parametru).

Zone de azimut (relativ la cameră, stânga negativ — NU cardinal):
    foarte-stânga  < -HFOV/4 | stânga -HFOV/4..-5° | centru ±5° |
    dreapta +5°..+HFOV/4 | foarte-dreapta > +HFOV/4      (HFOV=78.3°)

Prioritate (identică conceptual cu _decide_action, nu una nouă):
  1. risc CRITICAL        -> TOATE motoarele, intensitate maximă, puls rapid
  2. alertă audio PERICOL -> puls dublu distinct (sursa se distinge de obstacol)
  3. risc HIGH            -> motorul zonei obstacolului, intensitate mare
  4. off-path (segmentare)-> puls scurt generic, fără direcţie, intensitate joasă
  5. ghidare normală      -> UN motor de zonă, intensitate joasă
                             (centru: blip scurt de confirmare)
  6. nimic                -> totul oprit

Rate: /keryke/risk soseşte la 10 Hz, dar cadenţa percepută a pulsurilor e
2-4 Hz (anvelopă proprie, timer intern) — pielea nu tolerează buzz continuu.

Fail-safe ("no data ≠ semnal"): porneşte cu totul oprit; fără RiskDescriptor
mai vechi de stale_timeout -> totul oprit; orice excepţie sau shutdown ->
toate canalele la 0 + deinit. Fără hardware I2C (sau cu force_mock:=true)
rulează cu un backend mock care doar loghează — permite dezvoltare şi test
fără vestă.

Publică /vest/haptic_state (String JSON) — starea curentă a motoarelor, la
tranziţii + heartbeat 1 Hz (STATE_HEARTBEAT_S) — pentru depanare/test fără
hardware şi pentru panelul "Vesta haptica" din dashboard (care redă anvelopa
cu o copie a _pattern_on — lockstep la schimbări).
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from .perception_geometry import HFOV_DEG
except ImportError:
    from perception_geometry import HFOV_DEG

# ── Mapare zone de azimut (grade) ──────────────────────────────────────────────
AZ_CENTER_DEG = 5.0             # ±5° = "drept în faţă"
AZ_EXTREME_DEG = HFOV_DEG / 4   # ±19.6° = marginea zonei "moderate"

# ── Intensităţi per situaţie (fracţie din duty maxim 16-bit) ──────────────────
INTENSITY_ALERT   = 1.00   # critical: totul la maxim
INTENSITY_AUDIO   = 0.80   # alertă audio pericol
INTENSITY_HIGH    = 0.75   # obstacol high, motor de zonă
INTENSITY_OFFPATH = 0.35   # puls generic "nu eşti pe zona sigură"
INTENSITY_GUIDE   = 0.35   # ghidare normală de direcţie
INTENSITY_CONFIRM = 0.25   # blip de confirmare "eşti pe centru"

# ── Cadenţe pulsuri (anvelopa temporală; vezi _pattern_on) ────────────────────
ALERT_PULSE_HZ = 4.0       # critical: puls rapid
GUIDE_PULSE_HZ = 2.5       # ghidare: puls confortabil 2-4 Hz

PWM_FREQ_HZ = 60           # frecvenţa PWM a PCA9685 (identică cu testul hardware)
RENDER_PERIOD_S = 0.05     # 20 Hz — suficient de fin pt anvelope de 2-4 Hz

# Starea se publică la tranziţii + heartbeat: fără el, o decizie stabilă
# (acelaşi azimut minute în şir) nu ar produce niciun mesaj şi dashboard-ul
# n-ar putea distinge "nod mort" de "stare stabilă".
STATE_HEARTBEAT_S = 1.0


class MockBackend:
    """Backend fără hardware: loghează tranziţiile, pentru dezvoltare/test."""

    def __init__(self, logger, n_motors: int):
        self._logger = logger
        self._last = [0.0] * n_motors

    def set_levels(self, levels):
        if levels != self._last:
            desc = " ".join(f"{lv:.2f}" for lv in levels)
            self._logger.info(f"[mock vest] niveluri: [{desc}]")
            self._last = list(levels)

    def close(self):
        self._logger.info("[mock vest] toate motoarele oprite")


class Pca9685Backend:
    """PCA9685 real pe I2C — fidel scriptului de test validat pe hardware."""

    def __init__(self, channels, freq_hz: int):
        import board
        import busio
        from adafruit_pca9685 import PCA9685

        self._pca = PCA9685(busio.I2C(board.SCL, board.SDA))
        self._pca.frequency = freq_hz
        self._channels = channels
        self._last = [None] * len(channels)

    def set_levels(self, levels):
        for i, (ch, lv) in enumerate(zip(self._channels, levels)):
            if lv != self._last[i]:
                self._pca.channels[ch].duty_cycle = int(lv * 0xFFFF)
                self._last[i] = lv

    def close(self):
        # Oprire de siguranţă: niciun motor nu rămâne pornit după nod.
        for ch in self._channels:
            self._pca.channels[ch].duty_cycle = 0
        self._pca.deinit()


def _zone_index(azimuth_deg: float) -> int:
    """Azimut continuu -> indexul motorului (0=stânga-extremă .. 4=dreapta-extremă)."""
    if azimuth_deg < -AZ_EXTREME_DEG:
        return 0
    if azimuth_deg < -AZ_CENTER_DEG:
        return 1
    if azimuth_deg <= AZ_CENTER_DEG:
        return 2
    if azimuth_deg <= AZ_EXTREME_DEG:
        return 3
    return 4


def _pattern_on(pattern: str, now: float) -> bool:
    """Anvelopa temporală a pulsului: e motorul pornit în acest moment?"""
    if pattern == "alert":            # puls rapid 4 Hz, 50% duty
        return (now * ALERT_PULSE_HZ) % 1.0 < 0.5
    if pattern == "double":           # puls dublu pe secundă (distinct: sursă audio)
        phase = now % 1.0
        return phase < 0.12 or 0.24 <= phase < 0.36
    if pattern == "pulse":            # puls continuu confortabil 2-4 Hz
        return (now * GUIDE_PULSE_HZ) % 1.0 < 0.5
    if pattern == "blip":             # blip scurt de confirmare, la 2 s
        return now % 2.0 < 0.12
    if pattern == "short":            # puls scurt generic, la ~1.2 s
        return now % 1.2 < 0.15
    return False                      # "off" sau necunoscut


class HapticVestNode(Node):
    def __init__(self):
        super().__init__("haptic_vest")

        self.declare_parameter("channels", [10, 11, 14, 12, 13])
        self.declare_parameter("force_mock", False)
        self.declare_parameter("stale_timeout", 1.0)

        channels = list(self.get_parameter("channels").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout").value)
        self._n = len(channels)

        # Backend real dacă se poate, altfel mock — nodul porneşte oricum
        # (bringup-ul nu depinde de prezenţa fizică a vestei).
        self._backend = None
        if not self.get_parameter("force_mock").value:
            try:
                self._backend = Pca9685Backend(channels, PWM_FREQ_HZ)
                self.get_logger().info(
                    f"PCA9685 initializat (canale {channels}, {PWM_FREQ_HZ} Hz)"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"PCA9685 indisponibil ({e}) — trec pe backend mock"
                )
        if self._backend is None:
            self._backend = MockBackend(self.get_logger(), self._n)

        self._last_risk: dict = {}
        self._last_risk_time = 0.0
        self._last_state_json = ""
        self._last_state_pub_time = 0.0

        self.create_subscription(String, "/keryke/risk", self.risk_cb, 10)
        self.state_pub = self.create_publisher(String, "/vest/haptic_state", 10)
        self.create_timer(RENDER_PERIOD_S, self._render)

        self.get_logger().info(
            "Haptic Vest pornit — busolă haptică pe /keryke/risk "
            f"(zone ±{AZ_CENTER_DEG}° / ±{AZ_EXTREME_DEG:.1f}°)"
        )

    def risk_cb(self, msg: String):
        try:
            risk = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return
        # Doar dict-uri: un "null"/listă valid JSON ar crăpa _decide_target
        # la fiecare tick de redare (spam de erori la 20 Hz).
        if isinstance(risk, dict):
            self._last_risk = risk
            self._last_risk_time = time.time()

    # ── Traducerea deciziei în ţintă de vibraţie ──────────────────────────────

    def _decide_target(self, risk: dict):
        """
        Întoarce (motoare_active, intensitate, pattern, sursă) din
        RiskDescriptor — aceeaşi scară de priorităţi ca _decide_action().
        """
        level = risk.get("risk_level", "none")
        audio = risk.get("audio_alert") or {}
        walkable = risk.get("walkable_status") or {}
        azimuth = risk.get("compass_azimuth_deg")

        # 1) Obstacol critic: toate motoarele, indiferent de direcţie —
        #    pericolul bate orice ghidare (regula de baza a proiectului).
        if level == "critical":
            return list(range(self._n)), INTENSITY_ALERT, "alert", "critical"

        # 2) Alertă audio de pericol: puls dublu, distinct de obstacolul vizual.
        if audio.get("nivel") == "pericol":
            return list(range(self._n)), INTENSITY_AUDIO, "double", "audio_pericol"

        # 3) Obstacol high: motorul zonei obstacolului, intensitate mare.
        if level == "high" and azimuth is not None:
            return [_zone_index(azimuth)], INTENSITY_HIGH, "pulse", "obstacol_high"

        # 4) Nu eşti pe zona sigură: puls scurt generic, fără direcţie
        #    (segmentarea e semnal de siguranţă, nu de ghidare).
        if walkable and not walkable.get("on_path", True):
            return list(range(self._n)), INTENSITY_OFFPATH, "short", "off_path"

        # 5) Ghidare normală de direcţie (target/drum), fără pericol.
        if azimuth is not None:
            zone = _zone_index(azimuth)
            if zone == 2:
                # Pe centru: blip scurt de confirmare, nu puls continuu.
                return [2], INTENSITY_CONFIRM, "blip", "ghidare_centru"
            return [zone], INTENSITY_GUIDE, "pulse", "ghidare"

        return [], 0.0, "off", "nimic"

    # ── Redarea anvelopei (20 Hz) ─────────────────────────────────────────────

    def _render(self):
        now = time.time()
        try:
            if now - self._last_risk_time > self._stale_timeout:
                # Fail-safe: fără decizie proaspătă, niciun semnal pe piele.
                active, intensity, pattern, source = [], 0.0, "off", "stale"
            else:
                active, intensity, pattern, source = self._decide_target(
                    self._last_risk
                )

            on = _pattern_on(pattern, now)
            levels = [
                intensity if (on and i in active) else 0.0
                for i in range(self._n)
            ]
            self._backend.set_levels(levels)
            self._publish_state(active, intensity, pattern, source)
        except Exception as e:
            # Orice defect în redare NU lasă motoarele pornite. Log throttled:
            # un defect I2C persistent (cablu scos în mers) ar genera altfel
            # o eroare la fiecare tick de 20 Hz.
            self.get_logger().error(
                f"Redare vesta esuata: {e}", throttle_duration_sec=5.0
            )
            try:
                self._backend.set_levels([0.0] * self._n)
            except Exception:
                pass  # backend complet mort -- nu mai avem ce comanda

    def _publish_state(self, active, intensity, pattern, source):
        state = {
            "active_motors": active,
            "intensity": intensity,
            "pattern": pattern,
            "source": source,
        }
        data = json.dumps(state)
        now = time.time()
        # Tranziţii + heartbeat (STATE_HEARTBEAT_S) -- consumatorii pot
        # distinge "nod mort" de "stare stabilă".
        if data != self._last_state_json or \
                now - self._last_state_pub_time >= STATE_HEARTBEAT_S:
            self._last_state_json = data
            self._last_state_pub_time = now
            msg = String()
            msg.data = data
            self.state_pub.publish(msg)

    def destroy_node(self):
        try:
            self._backend.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HapticVestNode()
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
