#!/usr/bin/env python3
"""
KERYKE v3 — Client BLE de referință (Raspberry Pi / PC) pentru firmware-ul
`esp32_keryke_ble`
==========================================================================
Variantă Bluetooth a lui `pi_server_tcp.py`: ACELAȘI protocol binar, dar peste BLE
GATT în loc de TCP. Rolurile se inversează față de varianta WiFi:
  - ESP32-S3 = periferic BLE (advertising, GATT server) — vezi esp32_keryke_ble.ino;
  - Pi/PC    = central BLE (scanează, se conectează, se împerechează cu PIN).

SECURITATE: caracteristicile GATT cer link CRIPTAT + AUTENTIFICAT (MITM). De aceea,
înainte de a citi/scrie, dispozitivul TREBUIE împerecheat cu passkey-ul (PIN) fix din
firmware (implicit 123456). Pe Linux/BlueZ împerecherea se face O SINGURĂ DATĂ, de
regulă din `bluetoothctl` (vezi „Împerechere" mai jos); după bonding, reconectarea nu
mai cere PIN. Scriptul încearcă și `client.pair()` programatic (best effort).

GATT (trebuie să corespundă firmware-ului):
  Serviciu KERYKE                5f6d0001-9b5a-4c3d-8e2f-1a2b3c4d5e6f
    Caracteristică TX (NOTIFY)   5f6d0002-9b5a-4c3d-8e2f-1a2b3c4d5e6f  (ESP -> central)
    Caracteristică RX (WRITE)    5f6d0003-9b5a-4c3d-8e2f-1a2b3c4d5e6f  (central -> ESP)

Cadru (identic cu varianta TCP): STX(0x02) | OPCODE | LEN(uint16 LE) | PAYLOAD | ETX(0x03)

Dependență:  pip install bleak
Rulare:      python pi_client_ble.py            # caută după nume "KERYKE-ESP32"
             python pi_client_ble.py AA:BB:CC:DD:EE:FF   # conectare la o adresă anume

--------------------------------------------------------------------------------
Împerechere (o singură dată, pe Raspberry Pi OS / BlueZ):
    bluetoothctl
      power on
      agent KeyboardOnly
      default-agent
      scan on                      # așteaptă să apară KERYKE-ESP32 + adresa
      scan off
      pair AA:BB:CC:DD:EE:FF        # va cere passkey-ul => introdu 123456
      trust AA:BB:CC:DD:EE:FF
      quit
După `pair` + `trust`, dispozitivul e bonded; scriptul se conectează direct.
--------------------------------------------------------------------------------
"""

from __future__ import annotations

import asyncio
import struct
import sys
from dataclasses import dataclass

from bleak import BleakClient, BleakScanner

# ---------------- Constante protocol (identice cu pi_server_tcp.py) ----------------
STX, ETX = 0x02, 0x03

CMD_GET_TELEMETRY  = 0x10
RSP_TELEMETRY      = 0x11   # răspuns SOLICITAT la CMD_GET_TELEMETRY
RSP_TELEMETRY_PUSH = 0x12   # emisie NESOLICITATĂ periodică (payload identic)
CMD_MOTOR_ROTATE   = 0x20   # int16 LE grade relative servo
CMD_SERVO_POSE     = 0x22   # poză servo (centru/stânga/dreapta) + mod gait/imediat
CMD_VIBRATE        = 0x23   # impuls motor vibrații (intensitate + durată)
RSP_ACK            = 0x06
RSP_NACK           = 0x15

POSE_CENTER, POSE_LEFT, POSE_RIGHT = 0, 1, 2

NACK_CODES = {0x01: "lungime invalida", 0x02: "opcode necunoscut", 0x03: "coada actuator plina"}

TELEMETRY = struct.Struct('<IBh10f3B')   # 50 octeți
assert TELEMETRY.size == 50

# ---------------- Config BLE (trebuie să corespundă firmware-ului) ----------------
DEVICE_NAME  = "KERYKE-ESP32"
SERVICE_UUID = "5f6d0001-9b5a-4c3d-8e2f-1a2b3c4d5e6f"
TX_UUID      = "5f6d0002-9b5a-4c3d-8e2f-1a2b3c4d5e6f"   # NOTIFY: ESP -> central
RX_UUID      = "5f6d0003-9b5a-4c3d-8e2f-1a2b3c4d5e6f"   # WRITE : central -> ESP


@dataclass
class Telemetry:
    counter: int
    angle: int
    distance_mm: int
    pitch: float
    roll: float
    yaw: float
    ax: float; ay: float; az: float
    gx: float; gy: float; gz: float
    temp_c: float
    swing: int          # 0/1 fază de balans detectată
    servo: int          # unghi servo curent [0..180]
    vibration: int      # nivel PWM vibrații [0..255]


def decode_telemetry(payload: bytes) -> Telemetry:
    return Telemetry(*TELEMETRY.unpack(payload))


def build_frame(opcode: int, payload: bytes = b'') -> bytes:
    return bytes([STX, opcode]) + struct.pack('<H', len(payload)) + payload + bytes([ETX])


MAX_PAYLOAD = 256               # identic cu P_MAX_PAYLOAD din firmware


class FrameAssembler:
    """Automat finit care reasamblează cadre din fluxul de octeți al notificărilor BLE.
    Deși un cadru încape de regulă într-o singură notificare (MTU 517), tratarea la
    nivel de octet e robustă și la fragmentare/concatenare."""

    def __init__(self):
        self._buf = bytearray()
        self._state = 0          # 0=STX,1=OPCODE,2=LEN_L,3=LEN_H,4=PAYLOAD,5=ETX
        self._opcode = 0
        self._len = 0
        self._need = 0

    def feed(self, data: bytes):
        """Generează (opcode, payload) pentru fiecare cadru complet și valid."""
        for b in data:
            if self._state == 0:
                if b == STX:
                    self._state = 1
            elif self._state == 1:
                self._opcode = b
                self._state = 2
            elif self._state == 2:
                self._len = b
                self._state = 3
            elif self._state == 3:
                self._len |= b << 8
                if self._len > MAX_PAYLOAD:     # LEN corupt => resincronizare imediată
                    self._state = 0             # (identic cu parserul din firmware)
                    continue
                self._buf.clear()
                self._need = self._len
                self._state = 4 if self._len > 0 else 5
            elif self._state == 4:
                self._buf.append(b)
                self._need -= 1
                if self._need <= 0:
                    self._state = 5
            elif self._state == 5:
                if b == ETX:
                    yield self._opcode, bytes(self._buf)
                self._state = 0     # ETX invalid => resincronizare


# ---------------- Client de sesiune BLE (dispecerizare pe opcode) ----------------
class KerykeBLEClient:
    """Gestionează o sesiune BLE cu ESP32. Notificările de pe TX sunt dispecerizate
    după opcode: telemetria (solicitată 0x11 / nesolicitată 0x12) actualizează starea
    + callback, iar ACK/NACK deblochează emițătorul comenzii — la fel ca varianta TCP."""

    def __init__(self, client: BleakClient, on_telemetry=None):
        self._client = client
        self.on_telemetry = on_telemetry
        self.latest: Telemetry | None = None
        self.telemetry_count = 0

        self._asm = FrameAssembler()
        self._resp_q: asyncio.Queue = asyncio.Queue()      # cadre ACK/NACK
        self._tele_event = asyncio.Event()                 # semnalizează RSP_TELEMETRY (solicitat)
        self._solicited_count = 0

    async def start(self):
        await self._client.start_notify(TX_UUID, self._on_notify)

    def _on_notify(self, _sender, data: bytearray):
        for opcode, payload in self._asm.feed(bytes(data)):
            if opcode in (RSP_TELEMETRY, RSP_TELEMETRY_PUSH):
                if len(payload) != TELEMETRY.size:
                    print(f"[RX] telemetrie malformata (len={len(payload)}) — ignorata")
                    continue
                solicited = (opcode == RSP_TELEMETRY)
                t = decode_telemetry(payload)
                self.latest = t
                self.telemetry_count += 1
                if solicited:
                    self._solicited_count += 1
                    self._tele_event.set()
                if self.on_telemetry:
                    self.on_telemetry(t, solicited)
            elif opcode in (RSP_ACK, RSP_NACK):
                self._resp_q.put_nowait((opcode, payload))
            else:
                print(f"[RX] opcode neasteptat {opcode:#04x} (len={len(payload)}) — ignorat")

    # ---- API comenzi ----
    async def _send(self, frame: bytes):
        # response=True forțează schimb autentificat; RX are permisiune ENC_MITM.
        await self._client.write_gatt_char(RX_UUID, frame, response=True)

    async def request_telemetry(self, timeout: float = 3.0) -> Telemetry:
        base = self._solicited_count
        self._tele_event.clear()
        await self._send(build_frame(CMD_GET_TELEMETRY))
        while self._solicited_count <= base:
            try:
                await asyncio.wait_for(self._tele_event.wait(), timeout)
            except asyncio.TimeoutError:
                raise TimeoutError("telemetrie neprimita in timp util")
            self._tele_event.clear()
        return self.latest

    async def _command(self, opcode: int, payload: bytes, timeout: float) -> int:
        # golește eventuale răspunsuri vechi
        while not self._resp_q.empty():
            self._resp_q.get_nowait()
        await self._send(build_frame(opcode, payload))
        try:
            rop, rpl = await asyncio.wait_for(self._resp_q.get(), timeout)
        except asyncio.TimeoutError:
            raise TimeoutError("niciun raspuns la comanda in timp util")
        if rop == RSP_ACK:
            return struct.unpack('<h', rpl[1:3])[0]
        raise RuntimeError(f"NACK {opcode:#04x}: {NACK_CODES.get(rpl[1], hex(rpl[1]))}")

    async def motor_rotate(self, degrees: int, timeout: float = 3.0) -> int:
        return await self._command(CMD_MOTOR_ROTATE, struct.pack('<h', degrees), timeout)

    async def servo_pose(self, pose: int, immediate: bool = False, timeout: float = 3.0) -> int:
        return await self._command(CMD_SERVO_POSE,
                                   bytes([pose & 0xFF, 1 if immediate else 0]), timeout)

    async def vibrate(self, intensity: int, duration_ms: int, timeout: float = 3.0) -> int:
        return await self._command(CMD_VIBRATE,
                                   bytes([intensity & 0xFF]) + struct.pack('<H', duration_ms), timeout)


# ---------------- Descoperire + împerechere ----------------
async def find_device(target: str | None):
    """Găsește dispozitivul după adresă (dacă e dată) sau după nume/serviciu."""
    if target:
        print(f"[BLE] Caut dispozitivul {target} ...")
        dev = await BleakScanner.find_device_by_address(target, timeout=15.0)
        if dev:
            return dev
    print(f"[BLE] Scanez dupa \"{DEVICE_NAME}\" (serviciu {SERVICE_UUID}) ...")
    dev = await BleakScanner.find_device_by_filter(
        lambda d, adv: (d.name == DEVICE_NAME)
        or (SERVICE_UUID.lower() in [u.lower() for u in (adv.service_uuids or [])]),
        timeout=15.0,
    )
    return dev


def print_telemetry(t: Telemetry, solicited: bool) -> None:
    sursa = "SOLICITAT  " if solicited else "NESOLICITAT"
    print(f"[TELE {sursa}] #{t.counter:6d} angle={t.angle:3d}° dist={t.distance_mm:5d}mm "
          f"swing={'Y' if t.swing else 'N'} servo={t.servo:3d}° vib={t.vibration:3d} "
          f"yaw={t.yaw:+.3f} pitch={t.pitch:+.3f} T={t.temp_c:.1f}°C")


# ---------------- Demonstrație ----------------
async def main() -> None:
    target = sys.argv[1] if len(sys.argv) > 1 else None
    dev = await find_device(target)
    if not dev:
        print("[BLE] Dispozitiv negasit. Verifica ca ESP-ul face advertising si e in raza.")
        return
    print(f"[BLE] Gasit: {dev.name} [{dev.address}] — conectare...")

    async with BleakClient(dev) as client:
        print("[BLE] Conectat.")
        try:
            # Împerechere (best effort). Dacă e deja bonded, e no-op; dacă nu, pe unele
            # platforme cere PIN-ul printr-un agent de sistem (vezi bluetoothctl).
            if not await client.pair():
                print("[BLE] Atentie: pair() a returnat False (poate deja bonded).")
        except Exception as e:  # noqa: BLE0001 (platforme fără suport pair())
            print(f"[BLE] pair() indisponibil pe aceasta platforma: {e} "
                  f"(imperecheaza manual cu bluetoothctl).")

        session = KerykeBLEClient(client, on_telemetry=print_telemetry)
        await session.start()
        print("[BLE] Abonat la telemetrie. Trimit comenzi de test...")

        print(f"  [ACK] servo test dreapta: {await session.servo_pose(POSE_RIGHT, immediate=True)}")
        print(f"  [ACK] vibratie scurta: intensitate {await session.vibrate(150, 300)}")

        # Push-urile nesolicitate (5 s) sunt afișate automat de callback. Solicităm
        # telemetrie la 2 s și, periodic, emitem comenzi de actuator.
        tick = 0
        while True:
            await asyncio.sleep(2.0)
            tick += 1
            await session.request_telemetry()
            if tick % 6 == 0:
                pose = POSE_LEFT if (tick // 6) % 2 else POSE_RIGHT
                await session.servo_pose(pose)      # gait-sync
                await session.vibrate(180, 600)
                print("  [ACK] poza gait-sync + vibratie trimise")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[BLE] Oprit.")
