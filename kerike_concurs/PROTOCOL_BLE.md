# Protocol KERYKE v3 — variantă Bluetooth (BLE securizat)

> **Statut: acesta este documentul de protocol al proiectului — sursa de
> adevăr, autonomă.** Varianta TCP (`esp32_keryke_tcp/`, `pi_server_tcp.py`)
> a fost un pas intermediar de dezvoltare și se păstrează **doar ca istorie**;
> nu se mai întreține în lockstep.

Protocolul binar KERYKE este transportat peste **Bluetooth Low Energy (BLE)
GATT**. Cadrul, opcode-urile și `TelemetryPacket` (50 octeți) sunt specificate
complet în §3; §2 și §4 descriu transportul și modelul de securitate.

> **De ce BLE și nu Bluetooth „clasic"?** ESP32-S3 nu are Bluetooth Classic
> (fără SPP/RFCOMM) — suportă doar BLE. Din fericire, BLE oferă nativ pairing cu
> PIN + criptare + bonding, exact securitatea cerută.

## 1. Topologie

| Rol | Funcție BLE |
|-----|-------------|
| ESP32-S3 | **periferic** BLE (advertising + GATT server) |
| Raspberry Pi | **central** BLE (scanează, se conectează, se împerechează) |

- ESP32 face advertising cu numele `KERYKE-ESP32` și cu UUID-ul serviciului KERYKE.
- Pi/PC scanează, se conectează, se **împerechează cu PIN**, apoi se abonează la
  notificări și trimite comenzi.
- La deconectare, ESP repornește advertising-ul (reconectare autonomă din partea Pi).

## 2. GATT

| Element | UUID | Proprietăți | Permisiuni |
|---------|------|-------------|------------|
| Serviciu KERYKE | `5f6d0001-9b5a-4c3d-8e2f-1a2b3c4d5e6f` | — | — |
| Caracteristică **TX** (ESP → central) | `5f6d0002-…` | NOTIFY, READ | **ENC_MITM** (citire/abonare cer criptare) |
| Caracteristică **RX** (central → ESP) | `5f6d0003-…` | WRITE, WRITE_NR | **ENC_MITM** (scrierea cere criptare) |

- **TX**: ESP trimite pe această caracteristică, prin **notificări**, toate cadrele
  de protocol (RSP_TELEMETRY, RSP_TELEMETRY_PUSH, RSP_ACK, RSP_NACK).
- **RX**: centralul scrie aici cadrele de comandă (CMD_*). Firmware-ul le trece
  prin automatul finit de reasamblare (§3).
- **MTU** negociat: 517. Cadrul de telemetrie (5 + 50 = **55 octeți**) încape într-o
  singură notificare — fără fragmentare la nivel de aplicație.

## 3. Cadru, opcode-uri și `TelemetryPacket` — specificația completă

### 3.1 Cadrul

`STX(0x02) | OPCODE | LEN(uint16 LE) | PAYLOAD | ETX(0x03)`

Recepția (ambele capete) trece printr-un **automat finit cu resincronizare**:
un ETX invalid sau un LEN > 256 (`P_MAX_PAYLOAD` în firmware, `MAX_PAYLOAD`
în `pi_client_ble.py`) aruncă cadrul și reia căutarea STX-ului. Deși un cadru
încape de regulă într-o singură notificare (MTU 517), reasamblarea e la nivel
de octet — robustă la fragmentare/concatenare.

### 3.2 Opcode-uri

| Opcode | Nume | Direcție | LEN | Payload |
|--------|------|----------|-----|---------|
| `0x10` | `CMD_GET_TELEMETRY` | central → ESP | 0 | — |
| `0x11` | `RSP_TELEMETRY` | ESP → central | 50 | `TelemetryPacket` (răspuns SOLICITAT) |
| `0x12` | `RSP_TELEMETRY_PUSH` | ESP → central | 50 | `TelemetryPacket` (emisie NESOLICITATĂ, la 5 s) |
| `0x20` | `CMD_MOTOR_ROTATE` | central → ESP | 2 | `int16 LE` grade relative servo |
| `0x22` | `CMD_SERVO_POSE` | central → ESP | 2 | `[0]` poză: 0=centru, 1=stânga, 2=dreapta; `[1]` mod: 0=gait-sync, 1=imediat |
| `0x23` | `CMD_VIBRATE` | central → ESP | 3 | `[0]` intensitate 0..255; `[1..2]` durată ms `uint16 LE` |
| `0x06` | `RSP_ACK` | ESP → central | 3 | `[0]` opcode ecou; `[1..2]` `int16 LE` valoare |
| `0x15` | `RSP_NACK` | ESP → central | 2 | `[0]` opcode ecou; `[1]` cod eroare |

Coduri NACK: `0x01` lungime invalidă, `0x02` opcode necunoscut, `0x03` coadă
actuator plină.

Semantica: **cerere–răspuns inițiat de central + push periodic nesolicitat**
(5 s) de la ESP; clientul distinge sursele prin opcode (`0x11` vs `0x12`),
fără nicio euristică de corelare.

### 3.3 `TelemetryPacket` — exact 50 de octeți

Little-endian împachetat; în Python: `struct.Struct('<IBh10f3B')`, în firmware
`struct TelemetryPacket` cu `static_assert(sizeof == 50)`.

| Câmp | Tip | Octeți | Semnificație |
|------|-----|--------|--------------|
| `counter` | `uint32` | 4 | contor de eșantioane |
| `angle` | `uint8` | 1 | unghi [0..180] derivat din yaw |
| `distance_mm` | `int16` | 2 | distanța ToF (0/negativ = fără măsurătoare validă) |
| `pitch`, `roll`, `yaw` | `float ×3` | 12 | orientare (rad) |
| `ax`, `ay`, `az` | `float ×3` | 12 | accelerație (m/s²) |
| `gx`, `gy`, `gz` | `float ×3` | 12 | viteză unghiulară (rad/s) |
| `temp_c` | `float` | 4 | temperatura IMU (°C) |
| `swing` | `uint8` | 1 | 0/1 — fază de balans detectată |
| `servo` | `uint8` | 1 | unghi servo curent [0..180] |
| `vibration` | `uint8` | 1 | nivel PWM vibrații [0..255] |

## 4. Securitate — „conexiune sigură cu parolă"

Cerința: conexiunea trebuie să fie sigură și să ceară o parolă, astfel încât
dispozitivul să nu poată fi împerecheat decât cu anumite dispozitive.

Implementare (BLE, în firmware):

1. **Pairing cu passkey (PIN) fix de 6 cifre** — `BLE_PASSKEY` în firmware
   (implicit `123456`; **schimbă-l** pentru dispozitivul tău). Mod de securitate:
   **LE Secure Connections + MITM + Bonding** (`ESP_LE_AUTH_REQ_SC_MITM_BOND`),
   capabilitate IO **DisplayOnly** (`ESP_IO_CAP_OUT`) + **passkey static**. Practic:
   centralul e obligat să **introducă PIN-ul**; dacă e greșit, pairing-ul eșuează și
   nu se stabilește niciun canal criptat.
2. **Caracteristicile cer link criptat + autentificat** (`ENC_MITM`): fără pairing
   reușit, stiva **respinge** scrierile pe RX și **abonarea** la notificările TX.
   Deci niciun octet de protocol nu curge fără PIN corect. Acesta e enforcement-ul
   efectiv — nu depinde de verificări la nivel de aplicație.
3. **Bonding**: după prima împerechere reușită, cheile se salvează în NVS.
   Dispozitivele bonded reconectează **fără** a reintroduce PIN-ul; unul nou tot
   trebuie să treacă prin pairing cu PIN.

### Ce înseamnă „doar anumite dispozitive"
Doar un dispozitiv al cărui operator **cunoaște PIN-ul** poate finaliza pairing-ul
și, implicit, poate accesa datele. Un dispozitiv fără PIN nu poate nici comanda, nici
citi telemetria. Pentru a bloca și pairing-uri noi după provizionare, se poate șterge
posibilitatea de re-împerechere (ex. dezactivarea advertising-ului după primul bond)
sau se poate migra pe NimBLE, care expune ușor whitelist-ul de dispozitive bonded.

## 5. Împerechere (o singură dată)

### Raspberry Pi OS / Linux (BlueZ)
```bash
bluetoothctl
  power on
  agent KeyboardOnly
  default-agent
  scan on                      # așteaptă KERYKE-ESP32 + adresa AA:BB:CC:DD:EE:FF
  scan off
  pair AA:BB:CC:DD:EE:FF        # cere passkey => introdu 123456
  trust AA:BB:CC:DD:EE:FF       # permite reconectare automată
  quit
```
După `pair` + `trust`, dispozitivul e bonded. Rulează apoi clientul:
```bash
pip install bleak
python pi_client_ble.py                    # caută după nume
python pi_client_ble.py AA:BB:CC:DD:EE:FF   # sau conectare la o adresă anume
```

### Windows / Android / iOS
Împerechere din setările Bluetooth ale sistemului (apare „KERYKE-ESP32"; se cere
PIN-ul `123456`). Aplicații de test GATT (ex. nRF Connect) pot apoi scrie pe RX și
se pot abona la TX după pairing.

## 6. Invariant de consistență

Formatul binar e duplicat în **3 fișiere lockstep**; orice modificare de
opcode-uri / `TelemetryPacket` / payload-uri de comandă se face în toate trei
deodată:

1. `esp32_keryke_ble/esp32_keryke_ble.ino` — `enum Opcode`,
   `struct TelemetryPacket` cu `static_assert(sizeof==50)`, `handleFrame`.
2. `pi_client_ble.py` — `TELEMETRY = struct.Struct('<IBh10f3B')`, constantele
   de opcode, metodele de comandă (`KerykeBLEClient`).
3. Acest `PROTOCOL_BLE.md` (§3).

Fișierele TCP istorice (`esp32_keryke_tcp.ino`, `pi_server_tcp.py`) NU se mai
actualizează — descriu starea de la momentul înghețării lor.

## 7. Independența achiziției de Bluetooth

Toată achiziția și controlul sunt **independente de Bluetooth** (core 1):
eșantionare IMU la 50 ms, filtrul median ToF + fail-safe, detecția fazei de
balans (gait-sync), servo pe poze, ritmul cardiac haptic la obstacol. Doar
task-ul/​callback-urile de rețea sunt GATT + advertising + pairing (core 0).
