# KERYKE v3 — Arhitectura sistemului: hardware, software, principii și motivații

> Documentul UNIC de arhitectură al proiectului. Pentru detalii
> metodă-cu-metodă vezi `docs/ARHITECTURA_SOFTWARE_ESP32_BLE.txt`; pentru
> protocolul de comunicație vezi `PROTOCOL_BLE.md`.
>
> **Statut transport: varianta BLE este singura utilizată și dezvoltată.**
> Varianta TCP (`firmware/esp32_keryke_tcp/`, cu tot cu `pi_server_tcp.py`) a
> fost un pas intermediar de dezvoltare și se păstrează **doar ca istorie** —
> nu se mai întreține în lockstep. Setul lockstep al protocolului este format
> din **3 fișiere**: `esp32_keryke_ble.ino`, `pi_client_ble.py`,
> `PROTOCOL_BLE.md`.

## 1. Scopul sistemului

KERYKE v3 este un sistem cu două dispozitive pentru **ghidarea unei persoane
nevăzătoare la ocolirea obstacolelor**:

- un **nod purtabil ESP32-S3** care măsoară distanța până la obstacol și mișcarea
  corpului, și care comunică utilizatorului — exclusiv **haptic** (vibrații) și
  mecanic (servo) — apropierea de obstacol și direcția recomandată;
- un **Raspberry Pi 5** (vesta, central de comandă) care primește telemetria,
  rulează stack-ul de percepție și decizie (ROS 2: cameră + YOLO, adâncime,
  sunete de mediu, fuziune de risc, asistent vocal — §3.3) și trimite comenzi
  de actuator.

Natura aplicației (utilizator nevăzător, în mișcare, dependent de dispozitiv
pentru siguranța fizică) a dictat aproape toate deciziile de arhitectură de mai
jos. Firul roșu al proiectului: **funcțiile de siguranță rulează local, autonom
și neîntrerupt; tot ce este auxiliar (rețea, diagnoză) poate cădea fără ca
utilizatorul să fie pus în pericol.**

## 2. Arhitectura hardware

### 2.1 Bastonul — schema bloc

```
                    ┌────────────────────────── ESP32-S3 ──────────────────────────┐
   MPU6050 (IMU)    │  I2C bus 0: SDA=GPIO41, SCL=GPIO42, adresa 0x68              │
   accel+gyro ──────┤                                                              │
                    │  I2C bus 1: SDA=GPIO39, SCL=GPIO40, adresa 0x29,             │
   VL53L1X (ToF) ───┤             XSHUT=GPIO1 (reset hardware pentru recuperare)   │
   distanță ≤ ~4 m  │                                                              │
                    │  LEDC PWM:                                                   │
   Servo SG90 ◄─────┤    GPIO21, 50 Hz / 14 biți, impuls 500–2500 µs               │
                    │                                                              │
   Motor vibrații ◄─┤    GPIO18, 5 kHz / 8 biți, prin tranzistor/MOSFET            │
   (haptic)         │                                                              │
   LED RGB WS2812 ◄─┤    GPIO48 (LED_BUILTIN/RGB_BUILTIN, integrat pe placă)       │
   (stare senzori)  │                                                              │
                    │  Radio: BLE GATT securizat (varianta finală);                │
                    │         WiFi/TCP (pas intermediar de dezvoltare)             │
                    └───────────────────────────────┬──────────────────────────────┘
                                                    │ protocol binar KERYKE
                                                    ▼
                                     Raspberry Pi (central BLE / server TCP)
```

### 2.2 Bastonul — alegerea componentelor și motivațiile

| Componentă | Rol | De ce aceasta |
|---|---|---|
| **ESP32-S3** | microcontroler central | **Dual-core** — permite separarea fizică a achiziției/hapticii de rețea (principiul central al arhitecturii, §4.1); WiFi + BLE integrate; periferale bogate (2×I2C, LEDC); cost redus. Limită acceptată: nu are Bluetooth Classic, deci varianta Bluetooth folosește BLE GATT. |
| **VL53L1X** (time-of-flight) | distanța până la obstacol | Măsoară optic (laser IR) până la ~4 m, insensibil la culoarea/textura obstacolului, față de senzorii ultrasonici — mai precis, mai rapid, fascicul mai îngust. Are **range status** per măsurătoare (permite validarea fiecărei probe) și pin **XSHUT** (permite reset hardware pentru auto-recuperare). Mod LONG (~4 m) vs SHORT (≤1,3 m, mai robust la lumină ambientală) — configurabil. |
| **MPU6050** (IMU) | mișcarea corpului: pitch/roll/yaw, detecția pasului | Accel+gyro 6-DOF ieftin, suficient pentru detecția fazei de balans a mersului (gait) și pentru orientare grosieră. Nu e nevoie de magnetometru — yaw-ul absolut nu e cerință, doar cel relativ. |
| **Servo SG90** | indicarea mecanică a direcției | Actuator simplu de poziție (0–180°), comandat direct prin PWM; „poze" discrete (stânga/centru/dreapta) sunt suficiente pentru a comunica direcția. |
| **Motor de vibrații** | canalul principal de feedback | Pentru un nevăzător, **haptica este interfața primară** — nu display, nu LED-uri. Comandat PWM prin MOSFET (curentul motorului depășește ce poate furniza un GPIO). |
| **LED RGB integrat** (WS2812) | diagnoza stării senzorilor | Pentru însoțitor/dezvoltator, nu pentru utilizatorul final; fiind deja pe placă, nu costă nimic hardware. Culorile codifică starea senzorilor (§4.5). |
| **Raspberry Pi** | central de comandă | Linux complet: mDNS, BlueZ, Python, posibilitate de aplicație web — logica de nivel înalt nu are ce căuta pe microcontroler. |

### 2.3 Bastonul — decizii de interconectare și motivațiile lor

- **Două magistrale I2C separate** (`TwoWire(0)` pentru MPU6050, `TwoWire(1)`
  pentru VL53L1X), nu un singur bus partajat. Motivație: izolarea defectelor —
  un senzor care blochează bus-ul (clock-stretching, scurt pe SDA) nu doboară și
  celălalt senzor; în plus, elimină orice interacțiune de temporizare între un
  senzor citit la cadență fixă (IMU, 50 ms) și unul event-driven (ToF).
- **Alegerea pinilor de ieșire evită pinii de strapping** (GPIO0/3/45/46).
  Motivație: un nivel impus de periferic pe un pin de strapping la reset poate
  împiedica boot-ul — inacceptabil la un dispozitiv de siguranță. De aceea
  motorul de vibrații e pe GPIO18.
- **XSHUT-ul VL53L1X e legat la GPIO1**, nu la Vcc. Motivație: permite
  firmware-ului să facă **reset hardware** senzorului când acesta amuțește pe
  I2C (auto-recuperare fără intervenția utilizatorului).
- **Consolă serială prin punte UART CH343 la 115200 baud** — diagnoza rămâne
  disponibilă și când USB-CDC nativ nu e configurat.

### 2.4 Vesta (Raspberry Pi)

Schema electrică: `docs/Schema vesta raspberry pi.drawio` (sursa de adevăr
pentru cablaj).

| Componentă | Interfață / rol |
|---|---|
| **Raspberry Pi 5** | compute central: stack-ul ROS 2 de percepție și decizie (§3.3) |
| **Cameră IMX500** | CSI; folosită doar ca imager (NPU-ul on-chip nu e folosit); citită de `host/camera_publisher.py` pe host |
| **Microfon AB13X** | USB, 48 kHz (decimat la 16 kHz pentru wake word și YAMNet) |
| **Boxă USB** | ieșirea vocală (TTS, espeak-ng) |
| **PCA9685** | driver PWM comandat de Pi pe **I2C** — comandă motoarele de vibrații plate ale vestei (feedback haptic pe corp); **nu** direct din GPIO |
| **Motoare de vibrații plate** | pe canalele PCA9685 (5 în designul documentat) |
| **Alimentare** | baterie externă → modul coborâtor DC-DC + modul de filtrare |
| **Internet** | WiFi la hotspot-ul unui telefon mobil, exclusiv pentru API-ul Gemini (§3.3) |

Note:

- Haptica vestei e comandată **local de Pi** (I2C → PCA9685) — nu trece prin
  ESP32 și nici prin protocolul baston ↔ vestă.
- În varianta TCP intermediară, Pi era și Access Point pe `wlan0` pentru ESP32,
  ceea ce cerea o a doua interfață (dongle USB) pentru internet, fiindcă
  `wlan0` nu poate fi simultan AP și client. Cu BLE (varianta finală), legătura
  cu bastonul nu mai ocupă WiFi-ul, iar `wlan0` rămâne liber pentru hotspot-ul
  de internet.

## 3. Arhitectura software — vedere de ansamblu

Sistemul are două calculatoare cu roluri complementare:

- **bastonul (ESP32-S3)** — firmware Arduino dual-core (§3.1): achiziție
  senzori + haptică + servo, autonome față de rețea;
- **vesta (Raspberry Pi 5)** — stack de percepție și decizie pe ROS 2 Humble,
  rulat în containerul Docker `hive` (§3.3).

Pe legătura baston ↔ vestă, stratul de aplicație e **protocolul binar KERYKE**
peste **BLE GATT securizat**:

```
esp32_keryke_ble.ino ── protocol binar KERYKE (50 octeți) ── pi_client_ble.py (central BLE)
```

1. **Firmware ESP32** (`esp32_keryke_ble/`): transportul = BLE GATT securizat
   (pairing cu passkey de 6 cifre, caracteristici ENC_MITM, bonding în NVS).
2. **Partea Raspberry Pi** (`pi_client_ble.py`): codec de protocol + client de
   sesiune (`KerykeBLEClient`) care dispecerizează cadrele **după opcode** (nu
   presupune că următorul cadru răspunde ultimei comenzi).
3. Varianta TCP (`esp32_keryke_tcp/` + `pi_server_tcp.py`, ESP32 = stație WiFi
   + client TCP cu mDNS) — **istorie**, neîntreținută (vezi antetul).

> **Stadiu de integrare (conform codului actual):** puntea ROS 2 spre baston
> este nodul `ble_bridge` (§3.3), care încarcă dinamic clientul de referință
> `pi_client_ble.py` și vorbește protocolul binar de mai sus peste BLE.
> Vechea pereche de prototip UDP/JSON (`sensor_bridge`/`servo_command` +
> `firmware/varianta David.cpp`) a fost eliminată din repo — istoria rămâne
> în git.

### 3.1 Firmware: arhitectura dual-core

Separarea responsabilităților pe nuclee este **decizia de arhitectură centrală**:

```
CORE 1 — loop() Arduino: TIMP REAL, nu depinde de rețea
  • eșantionare IMU la 50 ms (20 Hz); citire ToF event-driven
  • pitch/roll din accel, yaw integrat din gyro; detecție fază de balans (gait)
  • zona de obstacol -> ritmul cardiac haptic; servo pe poze
  • execuția comenzilor de actuator din coadă
  • publicarea snapshot-ului de telemetrie
  • LED de stare + reîncercarea periodică a senzorilor picați (recover)

CORE 0 — rețea (task FreeRTOS "netTask" la TCP; stiva Bluedroid + task push la BLE)
  • WiFi + mDNS + sesiune TCP cu backoff exponențial   (varianta TCP)
  • advertising + pairing + GATT server                 (varianta BLE)
  • parsarea cadrelor, dispecerizarea comenzilor, răspunsuri
  • emisia periodică nesolicitată de telemetrie (5 s)
```

**Sincronizarea între nuclee este redusă deliberat la exact două canale**,
ambele non-blocante:

- `g_snapshot` (core 1 → core 0): copia structurii de telemetrie sub spinlock
  `portMUX`; secțiunea critică conține **doar copierea** — fără I/O, fără
  formatare — deci durează microsecunde.
- `g_actQueue` (core 0 → core 1): coadă FreeRTOS de comenzi de actuator,
  `xQueueSend`/`xQueueReceive` cu timeout 0 pe ambele capete.

Motivație: fiecare canal suplimentar de stare partajată este o sursă potențială
de blocaj sau race condition între nuclee. Două canale înguste, cu semantică
clară de producător/consumator, sunt ușor de raționat și imposibil de blocat.

### 3.2 Protocolul de comunicație

Cadru binar propriu (identic și pe transportul TCP istoric):

```
STX(0x02) | OPCODE | LEN(uint16 LE) | PAYLOAD | ETX(0x03)
```

- `TelemetryPacket` = **50 de octeți** împachetați, little-endian; comenzi:
  GET_TELEMETRY, MOTOR_ROTATE, SERVO_POSE, VIBRATE; răspunsuri: TELEMETRY,
  TELEMETRY_PUSH (nesolicitat, la 5 s), ACK, NACK.
- Recepția trece printr-un **automat finit cu resincronizare** (ETX invalid sau
  pauză inter-octet > 500 ms ⇒ revenire la căutarea STX).
- Modelul: **cerere–răspuns inițiat de Pi + push periodic nesolicitat** de la
  ESP; clientul distinge sursele prin opcode (0x11 vs 0x12).

Motivațiile acestor alegeri, în §4.6.

### 3.3 Vesta (Raspberry Pi) — stack-ul ROS 2

#### Mediul de execuție

ROS 2 Humble rulează în containerul Docker **`hive`** (bază
`ros:humble-ros-base`, `network_mode: host`, `privileged`, RMW = CycloneDDS,
`ROS_DOMAIN_ID=42`); `src/`, `models/`, `scripts/` sunt montate ca volume, iar
audio intră prin `/dev/snd` + socket-ul PulseAudio. **Camera rulează pe host,
în afara containerului** (`host/camera_publisher.py`, Picamera2) și servește
JPEG-uri prin TCP pe portul 9999 către `imx500_bridge` — Picamera2 nu e
disponibil în container; senzorul IMX500 e folosit doar ca imager, fără NPU-ul
on-chip.

**Conectivitate la internet.** Raspberry Pi se conectează la internet prin
WiFi, la **hotspot-ul unui telefon mobil** — singurul motiv este accesul la
LLM-ul din cloud (Gemini, prin API, în `brain`) pentru dialogul vocal cu
utilizatorul: întrebarea e înregistrată local, trimisă la API, iar răspunsul
text e redat vocal local prin TTS (espeak-ng, boxa USB). Interfața de internet
e separată de legătura cu bastonul; fără internet, sistemul pierde doar
asistentul conversațional — percepția, fuziunea de risc, naratorul determinist
și TTS-ul rulează integral local.

**Detalii operaționale:**

- `models/` e în `.gitignore` — modelele (`yolov8n.pt` sau exportul NCNN,
  modelul de adâncime ONNX, `yamnet.tflite` + `yamnet_class_map.csv`) **nu vin
  prin `git pull`**; se copiază manual pe Pi (scp/rsync) sau se redescarcă din
  sursele oficiale.
- Configurarea se face prin variabile de mediu: `KERYKE_YOLO_MODEL` (implicit
  `yolov8n.pt`; poate indica folderul unui export NCNN pentru inferență fără
  PyTorch), `KERYKE_YOLO_CONF` (implicit 0,25), `KERYKE_PERCEPTION_CORES`
  (implicit `0,1`), `KERYKE_DEPTH_CORES` (implicit `2`), `KERYKE_ONNX_THREADS`
  (implicit 1), `KERYKE_MIC_NAME` (implicit `AB13X`), `KERYKE_SPEAKER_NAME`,
  plus `GEMINI_API_KEY` (obligatorie pentru `brain`).
- Calea NCNN pentru YOLO e opțională: `yolo export model=yolov8n.pt
  format=ncnn imgsz=320`, apoi `KERYKE_YOLO_MODEL` la folderul exportat —
  elimină PyTorch la inferență (motor C++, mai ușor pe ARM).

#### Topologia proceselor

`launch/bringup.launch.py` pornește 8 executabile; nodurile sunt consolidate
deliberat în puține procese, cu afinitate CPU explicită (Pi 5 are 4 nuclee):

| Proces | Noduri ROS | Executor / afinitate CPU |
|---|---|---|
| `perception_container` | `yolo_detection` + `yolo_segmentation` | `MultiThreadedExecutor`; pinuit pe nucleele **0,1** |
| `depth_node` | Depth Anything V2 Small (ONNX int8), ~0,67 Hz | proces separat, pinuit pe nucleul **2** |
| `decision_container` | `spatial_risk` + `narrator` + `tts` | `MultiThreadedExecutor`, fără pinning |
| `imx500_bridge`, `audio_event`, `wake`, `brain`, `dashboard` | câte un proces fiecare | — |

Motivații: (a) `depth_node` a fost scos din `perception_container` pentru că
GIL-ul Python anulează paralelismul real — consolidat, ajungea la 1,6–3,6 s pe
cadru; (b) decizia e separată de percepție ca lanțul decizie→vorbire să
supraviețuiască dacă un model greu crapă.

`ble_bridge` (puntea BLE spre ESP32) **nu e în launch** — se pornește manual
când bastonul e împerecheat și pornit. `haptic_vest` E în launch: fără hardware
I2C trece singur pe backend mock, deci nu blochează bringup-ul.

#### Nodurile și fluxul de date

```
host: camera_publisher (Picamera2) ──TCP:9999──► imx500_bridge ──► /perception/image_raw (Image)
  /perception/image_raw ─┬─► yolo_detection    ──► /perception/detections_yolo (Detection2DArray)
                         ├─► yolo_segmentation ──► /perception/walkable_status (String)
                         │                        /perception/segmentation_overlay (CompressedImage)
                         ├─► depth_node        ──► /keryke/depth/metric (Image 32FC1)
                         └─► brain (cadrul curent, pentru Gemini)

ESP32 ──BLE (telemetrie binară 50B)──► ble_bridge ──► /keryke/sensors (String JSON: distanță ToF, IMU)
microfon USB ─┬─► audio_event (YAMNet) ──► /audio/alerts (String JSON)
              └─► wake (OpenWakeWord)  ──► /audio/wake_detected, /audio/status

spatial_risk (10 Hz): detections + sensors + alerts + walkable
        ──► /keryke/risk (String JSON, incl. servo_mode + compass_azimuth_deg),
            /servo/command (JSON {"pose","mode"}), /actuator/vibrate
brain (Gemini, declanșat de wake word): imagine + audio + context
        ──► /audio/speak, /servo/command, /actuator/vibrate
narrator (determinist, RO): /keryke/risk + detections ──► /audio/speak
/audio/speak ──► tts (espeak-ng + aplay) ──► boxa USB
/servo/command + /actuator/vibrate ──► ble_bridge ──BLE (CMD_SERVO_POSE/CMD_VIBRATE)──► ESP32
/keryke/risk ──► haptic_vest ──I2C (PCA9685)──► 5 motoare LRA pe vestă
dashboard (Flask :5000): consumator terminal al tuturor topicurilor (vizualizare + SSE)
```

Rolurile, pe scurt:

- **`imx500_bridge`** — punte TCP→ROS pentru cadrele camerei, cu reconectare
  automată.
- **`yolo_detection`** — YOLOv8n pe CPU (imgsz 320, prag implicit 0,25),
  filtrează clasele relevante și calculează azimutul din centrul bbox-ului.
- **`yolo_segmentation`** — semnal binar „zonă sigură" la ~3,3 Hz (la demo:
  prag HSV pe prosopul albastru); callback-uri în `ReentrantCallbackGroup` ca
  imaginea și timerul să ruleze concurent.
- **`depth_node`** — adâncime metrică asincronă, calibrată din
  `calibration.yaml`.
- **`spatial_risk`** — fuziunea de decizie (§3.4): o singură
  acțiune per ciclu, cu prioritate explicită obstacol/alertă peste țintă.
- **`brain`** — asistentul Gemini (`gemini-2.5-flash`), declanșat de wake
  word, nu always-on; înregistrează întrebarea cu `arecord`.
- **`narrator`** — fraze deterministe în română din risc/detecții; se mută
  singur cât timp utilizatorul vorbește cu asistentul.
- **`tts`** — coadă + worker pe espeak-ng/aplay, boxa USB.
- **`wake`** — „Hey Jarvis"; la detectare eliberează microfonul pentru brain.
- **`audio_event`** — YAMNet (tflite) pe ferestre de 1 s: vehicul/sirenă/
  claxon/strigăt, pentru unghiul mort de ~280° al camerei.
- **`ble_bridge`** — puntea bidirecțională spre ESP32 pe protocolul binar
  peste BLE: publică telemetria decodată pe `/keryke/sensors` și traduce
  `/servo/command` (JSON `{"pose","mode"}`) + `/actuator/vibrate` în
  `CMD_SERVO_POSE`/`CMD_VIBRATE`; reconectare cu backoff.
- **`haptic_vest`** — vesta-busolă haptică: 5 motoare LRA pe PCA9685 (I2C,
  local pe Pi), zone de azimut din `compass_azimuth_deg`, intensitate după
  `risk_level`, aceeași prioritate ca `_decide_action()`; fail-safe totul
  oprit la date vechi/oprire; backend mock fără hardware.
- **`dashboard`** — Flask pe :5000, feed adnotat la 2 Hz (nu imagine brută
  continuă), straturi comutabile, SSE cu senzori/risc/vorbire. Două decizii de
  design: **fără duplicare de calcul** — overlay-ul de segmentare vine gata
  randat de la nodul de segmentare (`/perception/segmentation_overlay`),
  dashboard-ul doar retransmite octeții; iar **punctul-țintă afișat reflectă
  decizia reală** din `/keryke/risk`, nu detecția cea mai încrezută (care putea
  indica chiar spre utilizator).

#### Limitări cunoscute în codul actual

- `/keryke/depth/metric` e publicat dar **nu intră încă în decizie** — singurul
  consumator e dashboard-ul; `sample_bbox_distance` din `depth_node.py` așteaptă
  viitorul nod de tracking/TTC.
- `spatial_risk` și `brain` publică amândouă pe `/servo/command` și
  `/actuator/vibrate` **fără arbitraj** între ele.
- Comenzile sosite cât timp bastonul e deconectat sunt aruncate de
  `ble_bridge` (cu warning throttled) — acceptabil pentru decizia de risc
  (se republică la 10 Hz), dar o comandă punctuală din `brain` se poate
  pierde.

### 3.4 Gait-Sync + busola haptică: o decizie, două canale

Aceeași decizie de risc alimentează două actuatoare cu constrângeri de timing
diferite — separarea lor e deliberată și nu trebuie unificată:

```
spatial_risk (10 Hz) ── RiskDescriptor pe /keryke/risk
        │
        ├─► /servo/command  JSON {"pose","mode"} ─► ble_bridge ─► CMD_SERVO_POSE(poză, mod)
        │        CANALUL BASTON: CE poză şi dacă-e-urgent decide Pi-ul;
        │        CÂND (faza de balans) decide ESP32 LOCAL, din propriul IMU
        │        (§4.10) — jitterul de transport nu atinge momentul biomecanic.
        │
        └─► /keryke/risk ────────────────────────► haptic_vest ─► PCA9685 (I2C) ─► 5× LRA
                 CANALUL VESTĂ: instant, la rata percepţiei, cu anvelopă
                 proprie de puls (2-4 Hz percepuţi) — fără constrângere
                 biomecanică. Vesta NU re-decide nimic: citeşte câmpurile
                 deja decise din RiskDescriptor.
```

Dacă ambele canale ar merge pe același ritm, ori se pierde feedback-ul continuu
de orientare (totul pe gait-sync), ori se strică sincronizarea biomecanică a
bastonului (totul instant).

#### 3.4.1 Schemele de date (sursa de adevăr pentru partea ROS)

**RiskDescriptor** (`/keryke/risk`, String JSON) — câmpurile de ghidare, pe
lângă cele de risc/acțiune:

| Câmp | Valori | Semantică |
|---|---|---|
| `servo_mode` | `"immediate"` \| `"gait_sync"` | `immediate` dacă riscul raportat e critical/high (siguranța bate confortul ritmic), altfel `gait_sync` |
| `compass_azimuth_deg` | float \| null | Azimut continuu, RELATIV la cameră (negativ = stânga, NU cardinal). **Din obstacolul primar DOAR la risc critical/high** („pericol acolo"); sub high, din direcția drumului dat de segmentare („mergi încolo"). Un singur sens per semnal — altfel vesta ar ghida utilizatorul spre un obstacol benign |
| `compass_source` | `"obstacle"` \| `"path"` \| null | De unde vine azimutul |

**`/servo/command`** (String JSON): `{"pose": "left"|"right"|"center",
"mode": "gait_sync"|"immediate"}`. Producători: `spatial_risk` (modul din
`servo_mode`) și `brain` (STOP-sign → `immediate`; ghidare Gemini →
`gait_sync`). Consumator: `ble_bridge` → `CMD_SERVO_POSE(poză, mod)`.
`ble_bridge` acceptă tolerant și string simplu (`"left"`) → `gait_sync`,
pentru teste manuale cu `ros2 topic pub`.

**`/keryke/sensors`** (String JSON, publicat de `ble_bridge` din telemetria
binară de 50 de octeți): câmpuri plate `distance` (mm), `accel_mag`, `gyroZ`,
`yaw`, `pitch`, `roll`, `swing`, `servo`, `vibration`, `angle`, `counter`,
`temp`. **Semantica `accel_mag`**: deviația față de gravitație
(`|sqrt(ax²+ay²+az²) − 9.81|`), nu magnitudinea brută — astfel pragul
`ACCEL_MOVING_THRESHOLD = 0.8 m/s²` din `spatial_risk` separă real staționar
(≈0) de mers (>0.8). Consumatorii tratează datele ca **perisabile**:
`spatial_risk` le ignoră dacă sunt mai vechi de `SENSOR_STALE_S` (2,5 s) —
deconectările BLE sunt mod normal de funcționare, iar o distanță înghețată ar
ține un „critical" fals sau ar masca un obstacol real („fără date ≠ semnal",
§4.2 aplicat și pe Pi).

**`/actuator/vibrate`** (String): `vibrate_alert` → intensitate 255 / 800 ms,
`vibrate_short` → 140 / 250 ms (constante `VIBRATE_*_PARAMS` în `ble_bridge`,
de calibrat pe dispozitiv). Separat de heartbeat-ul haptic autonom al
bastonului (firmware, pe zone de distanță ToF).

**`/vest/haptic_state`** (String JSON, publicat de `haptic_vest`):
`{"active_motors": [idx...], "intensity": 0..1, "pattern": "...",
"source": "..."}` — la **tranziții + heartbeat 1 Hz** (fără heartbeat,
o decizie stabilă nu ar produce mesaje și consumatorii n-ar putea distinge
„nod mort" de „stare stabilă"). Dashboard-ul are panelul „Vesta haptica":
anvelopa de puls e redată în browser cu o copie 1:1 a `_pattern_on()` din
`haptic_vest_node.py` — de păstrat în lockstep dacă se schimbă cadențele.

#### 3.4.2 Vesta-busolă: maparea concretă

- Canale PCA9685 (parametru ROS `channels`, implicit `[10, 11, 14, 12, 13]`) =
  pozițiile fizice stânga-extremă → dreapta-extremă (confirmate pe montaj).
  PWM 60 Hz, duty 16-bit.
- Zone de azimut (HFOV 78,3° din `perception_geometry.HFOV_DEG`):
  foarte-stânga < −19,6° | stânga −19,6°..−5° | centru ±5° |
  dreapta +5°..+19,6° | foarte-dreapta > +19,6°.
- Prioritate (aceeași scară ca `_decide_action()` din `spatial_risk` — vesta
  nu re-decide):

| # | Condiție (din RiskDescriptor) | Motoare | Intensitate | Pattern |
|---|---|---|---|---|
| 1 | `risk_level == critical` | TOATE | 1,00 | puls rapid 4 Hz |
| 2 | `audio_alert.nivel == "pericol"` | toate | 0,80 | puls DUBLU (distinct de obstacolul vizual) |
| 3 | `risk_level == high` | zona obstacolului | 0,75 | puls 2,5 Hz |
| 4 | `walkable_status.on_path == false` | toate | 0,35 | puls scurt generic (fără direcție) |
| 5 | ghidare normală (`compass_azimuth_deg` ≠ null) | UN motor de zonă; centru → blip de confirmare | 0,35 / 0,25 | puls 2,5 Hz / blip la 2 s |
| 6 | nimic | — | 0 | oprit |

- Fail-safe: pornire cu totul oprit; RiskDescriptor mai vechi de 1 s → totul
  oprit; excepție la redare → totul oprit; shutdown → canale 0 + `deinit()`.
  Fără hardware I2C (sau `force_mock:=true`): backend mock care doar loghează.

#### 3.4.3 Puntea BLE (`ble_bridge`): decizii de integrare

- NU copiază codul de protocol: încarcă dinamic
  `firmware/esp32_keryke_ble/pi_client_ble.py` (implementarea de referință)
  din `/ws/firmware` (volum în compose) sau din calea relativă a repo-ului —
  o copie în pachetul ROS ar fi fost al 4-lea fișier lockstep.
- bleak are nevoie de BlueZ-ul host-ului → `/run/dbus` montat în container.
  Împerecherea (passkey) se face O DATĂ, pe host, cu `bluetoothctl` — vezi
  `PROTOCOL_BLE.md` §5.
- Reconectare cu backoff (3 s → 15 s). Comenzile sosite cât bastonul e
  deconectat se aruncă cu warning throttled (vezi limitările din §3.3).
- Comenzile sunt **serializate cu un lock** pe bucla asyncio:
  `KerykeBLEClient._command` golește coada comună de ACK înainte de fiecare
  trimitere, iar `spatial_risk` publică servo + vibrate spate-în-spate — fără
  serializare, comenzile concurente și-ar arunca reciproc ACK-ul (timeout fals).

#### 3.4.4 Probleme cunoscute, asumate

1. `servo_mode` e la nivel de descriptor, nu per-acțiune — azi `spatial_risk`
   comandă servo doar la critical/high, deci comenzile lui sunt toate
   `immediate`; sursele reale de `gait_sync` sunt `brain` și viitoarea
   urmărire de rută.
2. Pragul 0,8 pentru `user_moving` și cadențele/intensitățile vestei
   (§3.4.2) sunt valori inițiale rezonabile — de calibrat fizic, pe piele,
   cu utilizatoarea.

#### 3.4.5 Verificare

Statică: `python -m py_compile` pe fișierele atinse; firmware-ul se compilă cu
`arduino-cli` doar dacă a fost atins.

Pe Pi, fără hardware (mock):
```bash
ros2 run hive_perception haptic_vest --ros-args -p force_mock:=true
# alt terminal — câte un RiskDescriptor pe treaptă de prioritate:
ros2 topic pub -1 /keryke/risk std_msgs/String \
  '{data: "{\"risk_level\":\"critical\",\"compass_azimuth_deg\":0.0}"}'
ros2 topic pub -1 /keryke/risk std_msgs/String \
  '{data: "{\"risk_level\":\"none\",\"compass_azimuth_deg\":15.0}"}'
ros2 topic echo /vest/haptic_state
```
Așteptat: critical → `[mock vest]` toate motoarele 1,00 pulsând; azimut +15°
→ doar motorul 3 la 0,35; după 1 s fără mesaje → totul 0 (stale).

Pe hardware (numai pe dispozitiv):
1. **Vestă reală**: rebuild imagine (`docker compose up -d --build hive`),
   `i2cdetect -y 1` în container (adresa 0x40 vizibilă); pornește
   `haptic_vest` fără `force_mock`; repetă mesajele de mai sus → motoarele
   fizice corecte vibrează; Ctrl+C → totul se oprește.
2. **Punte BLE**: împerechere `bluetoothctl` (`PROTOCOL_BLE.md` §5), apoi
   `ros2 run hive_perception ble_bridge`; `ros2 topic echo /keryke/sensors` →
   `distance` plauzibil, `swing` comută la pendularea bastonului, `accel_mag`
   ≈0 în repaus și >0.8 la mers.
3. **Gait-Sync pe bancă (prioritatea #1)**: cu monitorul serial la 115200:
   `ros2 topic pub -1 /servo/command std_msgs/String '{data: "{\"pose\":\"left\",\"mode\":\"gait_sync\"}"}'`
   → servoul NU mișcă; pendulează bastonul în mână (dip de accelerație +
   |gyroZ| > 0.3 rad/s) → servoul aplică poza, ține ~2 s, revine la centru.
   Apoi `{"pose":"right","mode":"immediate"}` → mișcă instant.
4. **Integrat (purtat)**: obstacol adus rapid central (<40 cm de ToF) → vesta
   trece pe alertă totală + servo `immediate`; mers normal pe „drum" fără
   obstacol → un singur motor de zonă pe vestă + redirecționările bastonului
   cad DOAR în faza de balans — exact ipoteza de validat fizic.

## 4. Principiile generale de proiectare și motivațiile lor

### 4.1 Siguranța nu depinde de rețea

**Principiu:** achiziția senzorilor și feedback-ul haptic rulează pe core 1 și
funcționează identic cu sau fără WiFi/BLE/Pi.

**Motivație:** utilizatorul se mișcă printre obstacole reale. O rețea WiFi care
cade, un Pi care se restartează sau o sesiune TCP în backoff **nu au voie** să
întârzie nici cu o milisecundă alerta de obstacol. De aici derivă direct:
alegerea unui MCU dual-core, pinning-ul task-ului de rețea pe core 0 și regula
celor două canale de sincronizare (§3.1).

### 4.2 „Fără date ≠ liber" (fail-safe)

**Principiu:** absența unei măsurători valide de distanță se tratează ca
**necunoscut**, nu ca „drum liber". Concret: dacă ToF nu produce o probă validă
timp de 300 ms (`TOF_STALE_MS`), distanța devine necunoscută și motorul de
vibrații **se oprește** — nu rămâne pe ultima valoare, nu presupune „departe".

**Motivație:** eroarea periculoasă la acest dispozitiv este falsul negativ
(obstacol nesemnalat). Un senzor mort care ar lăsa ritmul „lent" activ ar
comunica implicit „obstacol departe" — o minciună potențial periculoasă.
Oprirea vibrației este un semnal onest: „nu știu".

### 4.3 Robustețea măsurătorii înaintea vitezei

**Principiu:** lanțul distanței este: validare per-probă (range status = 0 și
d > 0) → **filtru median pe 5 probe valide** → prag de prospețime → decizie.
Senzorul mut pe I2C este **resetat hardware** (XSHUT) și reinițializat automat
la fiecare 2 s până revine.

**Motivație:** VL53L1X produce ocazional probe aberante (reflexii, lumină
ambientală); mediana elimină outlier-ii fără întârzierea și „coada" unei medii
alunecătoare. Auto-recuperarea există pentru că dispozitivul e purtabil:
conectorii se mișcă, contactele revin — dispozitivul trebuie să se vindece
singur, fără restart manual.

### 4.4 Nimic blocant în bucla de timp real

**Principiu:** pe core 1 nu există `delay()` și nu există așteptări. Toate
comportamentele temporizate — ritmul cardiac haptic „lub-dub", rampa de
vibrații, menținerea și revenirea pozei servo, clipirea LED-ului de stare —
sunt **mașini de stări pe `millis()`**, avansate la fiecare iterație a buclei.

**Motivație:** bucla trebuie să revină la citirea senzorilor la fiecare câteva
milisecunde. Un singur `delay(200)` pentru un puls de vibrație ar găuri
eșantionarea IMU și ar întârzia detecția obstacolului. Excepția tolerată:
self-test-ul și semnalul haptic de diagnostic din `setup()` — rulează o singură
dată, înainte de pornirea achiziției.

### 4.5 Pornire nerestricționată, dar semnalizată

**Principiu:** eșecul de inițializare al unui senzor **nu blochează boot-ul**.
Dispozitivul pornește în mod degradat, semnalizează starea și reîncearcă
periodic senzorul:

- **LED-ul RGB integrat** (non-blocant, din loop()): albastru = setup în curs;
  verde cu puls scurt la 3 s = totul OK; roșu clipind = ToF indisponibil;
  galben clipind = IMU indisponibil; roșu/galben alternat = ambii.
- **Semnal haptic la boot** (utilizatorul final nu vede LED-ul): 3 pulsuri
  lungi = ToF picat, 2 pulsuri = IMU picat.
- Senzorii picați sunt reîncercați automat (ToF la 2 s, IMU la 5 s); revenirea
  lor readuce sistemul în modul complet, fără restart.

**Motivație:** versiunea inițială oprea boot-ul într-o buclă infinită la primul
senzor absent — dispozitivul părea mort, fără nicio indicație, iar BLE nu mai
pornea deloc. Pentru un dispozitiv de teren, un mod degradat vizibil și
diagnosticabil de la distanță (telemetria continuă să curgă) este strict mai
bun decât un blocaj mut. Regula fail-safe (§4.2) garantează că modul degradat
rămâne sigur: fără ToF, motorul tace („nu știu"), nu improvizează.

### 4.6 Protocol binar propriu, minim, agnostic de transport

**Principiu:** un singur strat de aplicație (cadru + opcode-uri +
`TelemetryPacket` de 50 de octeți), purtat neschimbat peste TCP sau BLE GATT.

**Motivații:**

- **De ce binar și nu JSON/MQTT/HTTP:** pachetul de 50 de octeți încape într-o
  singură notificare BLE (MTU 517) și se serializează prin simpla copiere a
  structurii (`memcpy`) pe ESP32 și un `struct.unpack` pe Pi — zero parsare,
  zero alocări, footprint minim pe microcontroler. Un broker MQTT sau un stack
  HTTP nu aduce nimic aici: topologia este fix 1-la-1.
- **De ce little-endian împachetat:** este ordinea nativă și pe Xtensa (ESP32)
  și pe ARM (Pi) — serializarea devine o copiere de memorie, fără conversii.
- **De ce STX/ETX + automat finit cu resincronizare:** peste TCP integritatea o
  garantează transportul, dar delimitarea cadrelor tot e necesară (TCP e flux,
  nu mesaje); automatele cu resincronizare fac protocolul portabil și pe
  transporturi nesigure (UART/RS-485 — caz în care s-ar adăuga un CRC-8; peste
  BLE/TCP integritatea o garantează transportul).
- **De ce push periodic + cerere–răspuns:** push-ul la 5 s dă Pi-ului un
  „heartbeat" al nodului fără polling; opcode-uri distincte pentru răspuns
  solicitat (0x11) vs push (0x12) elimină orice euristică de corelare pe client.
- **Invariant de consistență:** formatul e duplicat în cele **3 fișiere
  lockstep** (`esp32_keryke_ble.ino`, `pi_client_ble.py`, `PROTOCOL_BLE.md`);
  orice modificare se face în lockstep în toate — impus prin
  `static_assert(sizeof==50)` în C++ și `assert TELEMETRY.size == 50` în
  Python. Fișierele TCP istorice NU se mai actualizează.

### 4.7 Descoperire prin nume, nu prin IP

**Principiu (doar varianta TCP, pasul intermediar):** ESP32 rezolvă serverul prin **mDNS**
(`PI_HOST.local`), cu IP de rezervă; re-rezolvă automat când DHCP schimbă
adresa. Conexiunea TCP se restabilește autonom cu **backoff exponențial**
(0,5 s → 8 s).

**Motivație:** dispozitivul e folosit în rețele domestice unde IP-urile se
schimbă; o configurație cu IP hardcodat ar cere reprogramare la fiecare
schimbare. Backoff-ul exponențial evită inundarea rețelei când serverul e
oprit, dar reconectează prompt când revine. Rolul de client TCP e pe ESP
tocmai pentru ca logica de reconectare să fie a dispozitivului, nu a omului.

### 4.8 Securitate impusă de stivă, nu de aplicație (varianta BLE)

**Principiu:** pairing cu **passkey fix de 6 cifre** (LE Secure Connections +
MITM + bonding), iar caracteristicile GATT cer link **criptat și autentificat**
(ENC_MITM). Fără PIN corect, stiva BLE respinge orice scriere și orice abonare
— **niciun octet de protocol nu curge**.

**Motivație:** dispozitivul comandă actuatori purtați de o persoană vulnerabilă;
un terț care i-ar trimite comenzi de vibrație/servo ar fi inacceptabil.
Enforcement-ul e pus la nivelul stivei (permisiuni GATT), nu în logică de
aplicație, pentru că doar acolo este garantat — codul de aplicație nici nu vede
traficul nepairuit. Bonding-ul (chei în NVS) face ca PIN-ul să fie cerut o
singură dată per dispozitiv împerecheat.

### 4.9 Interfața cu utilizatorul este haptica, proiectată pe intuiție

**Principiu:** apropierea de obstacol e comunicată printr-un **„ritm cardiac"**
lub-dub pe motorul de vibrații, a cărui frecvență crește cu apropierea:
< 1,5 m lent (~55 bpm), < 1 m mediu (~80 bpm), < 0,5 m alert (~150 bpm),
≥ 1,5 m oprit.

**Motivație:** un ritm cardiac care se accelerează este o metaforă pe care
oricine o decodează instinctiv, fără antrenament — spre deosebire de coduri
abstracte de pulsuri. Pattern-ul bifazic lub-dub e distinct de vibrațiile
„utilitare" (notificări), deci nu se confundă cu impulsurile comandate de Pi
(`CMD_VIBRATE`), față de care are prioritate pe motor.

### 4.10 Actuarea sincronizată cu mersul (gait-sync)

**Principiu:** IMU detectează faza de balans a pasului (scădere a magnitudinii
accelerației + vârf pe |gyroZ|, într-o fereastră glisantă de 10 eșantioane);
o comandă de poză servo în mod „gait" se aplică **în următoarea fază de
balans**, se menține ~2 s, apoi revine la centru.

**Motivație:** o indicație de direcție livrată în timpul fazei de sprijin se
pierde sau dezechilibrează; livrată în faza de balans, se integrează natural în
pas. Modul „imediat" există în paralel pentru teste și calibrare. Degradare
grațioasă: fără IMU funcțional, pozele se aplică imediat — comanda nu rămâne
niciodată agățată.

**Decuplarea esențială — decizia și momentul aplicării vin din surse diferite:**

| | Cine decide | Unde | Cum |
|---|---|---|---|
| **CE poză** (stânga/dreapta/centru) | Pi (fuziunea de risc, §3.3) | prin rețea | `CMD_SERVO_POSE(poză, mod=gait)` |
| **CÂND se aplică** | ESP32, core 1, local | fără rețea | detecția fazei de balans pe bucla IMU de 50 ms |

Comanda sosită pe rețea doar se **stochează**; aplicarea o face bucla locală,
la următoarea fază de balans detectată. Consecința: jitterul de transport
(WiFi/BLE) nu poate strica sincronizarea biomecanică — latența rețelei
afectează cel mult *ce* poză e curentă, niciodată *momentul* aplicării ei.
Acesta este argumentul central de originalitate al proiectului (ISEF).

## 5. Sinteza deciziilor: decizie → alternativă respinsă → motiv

| Decizie | Alternativa respinsă | Motivul alegerii |
|---|---|---|
| ESP32-S3 dual-core, rețea pe core 0 | un singur core cu scheduler cooperativ | izolarea fizică a timpului real de rețea; un stack TCP/BLE ocupat nu poate întârzia alerta de obstacol |
| ToF laser (VL53L1X) | senzor ultrasonic HC-SR04 | precizie, viteză, fascicul îngust, range-status per probă, XSHUT pentru reset hardware |
| două bus-uri I2C | un bus partajat | izolarea defectelor între senzori; fără interferențe de temporizare |
| protocol binar propriu (50 B) | JSON / MQTT / HTTP | zero parsare pe MCU, încape într-o notificare BLE, topologie fix 1-la-1 nu justifică broker |
| mDNS + fallback IP | IP static hardcodat | supraviețuiește schimbărilor DHCP fără reprogramare |
| BLE cu passkey + ENC_MITM | BLE deschis / filtrare în aplicație | comenzi de actuator către o persoană vulnerabilă; enforcement garantat doar la nivelul stivei |
| fail-safe „necunoscut ⇒ motor oprit" | păstrarea ultimei distanțe valide | falsul negativ (obstacol nesemnalat) este eroarea periculoasă |
| filtru median (5 probe) | medie alunecătoare | elimină outlier-ii fără să tragă valoarea spre ei |
| mașini de stări pe millis() | delay()-uri | bucla de timp real nu are voie să se oprească |
| boot nerestricționat + LED/haptic | blocaj `while(1)` la senzor absent | mod degradat diagnosticabil > dispozitiv mut; recuperare automată la revenirea senzorului |
| ritm cardiac haptic gradat | praguri simple on/off | decodare instinctivă a urgenței, fără antrenament |
| aplicare poză servo în faza de balans | aplicare imediată mereu | indicația integrată în pas nu dezechilibrează utilizatorul |

## 6. Mecanisme de siguranță și degradare — rezumat

| Defect | Comportament | Revenire |
|---|---|---|
| WiFi/BLE/Pi indisponibil | achiziția + haptica continuă neafectate; reconectare autonomă (backoff / re-advertising) | automată |
| ToF fără probe valide > 300 ms | distanță „necunoscută", motor vibrații oprit | automată, la prima probă validă |
| ToF mut pe I2C | reset XSHUT + reinițializare la fiecare 2 s; LED roșu clipind | automată |
| IMU absent/mut | telemetria continuă (câmpuri inerțiale 0), gait dezactivat, pozele servo se aplică imediat; reinițializare la 5 s; LED galben clipind | automată |
| senzor absent la boot | boot-ul continuă (mod degradat), 3/2 pulsuri haptice de diagnostic, BLE pornit pentru diagnoză | automată, prin recover |
| cadru corupt / flux desincronizat | automatul finit resincronizează pe STX; timeout inter-octet 500 ms | automată |
| coadă de actuator plină | NACK 0x03 către Pi (comanda nu se pierde silențios) | decizia rămâne la Pi |

## 7. Referințe

- `PROTOCOL_BLE.md` — **specificația completă a protocolului** (cadru,
  opcode-uri, `TelemetryPacket`, GATT, pairing) — sursa de adevăr, autonomă.
- `firmware/esp32_keryke_ble/esp32_keryke_ble.ino` — firmware-ul bastonului.
- `firmware/esp32_keryke_ble/pi_client_ble.py` — clientul de referință
  Raspberry Pi (codec + `KerykeBLEClient`), încărcat dinamic de `ble_bridge`.
- `docs/ARHITECTURA_SOFTWARE_ESP32_BLE.txt` — arhitectura software a
  firmware-ului, metodă cu metodă.
- `src/hive_perception/` — pachetul ROS 2 al vestei (noduri, containere de
  procese, `launch/bringup.launch.py`).
- `docker/` — imaginea și compose-ul containerului `hive`.
- `host/camera_publisher.py` — publisher-ul de cameră de pe host (în afara
  Docker-ului).
- Istorie (neîntreținute): `firmware/esp32_keryke_tcp/` (cu tot cu
  `pi_server_tcp.py`), `docs/ARHITECTURA_SOFTWARE_ESP32.txt` (varianta TCP,
  pas intermediar de dezvoltare).
