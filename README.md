# Keryke-OS

<img src="media/keryke-logo.png" alt="Logo" width="120">

[![Platform](https://img.shields.io/badge/platform-RaspberryPi5-blue?style=flat-square)]()
[![MCU](https://img.shields.io/badge/MCU-ESP32--S3-blue?style=flat-square)]()
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square)]()
[![Python](https://img.shields.io/badge/python-3.11+-blue?style=flat-square)]()
[![AI Model](https://img.shields.io/badge/YOLOv8-DepthAnythingV2-orange?style=flat-square)]()
[![Link](https://img.shields.io/badge/BLE-Secured-green?style=flat-square)]()

> **Keryke-OS** este un sistem asistiv purtabil, format din două dispozitive cooperante, destinat ghidării persoanelor nevăzătoare la ocolirea obstacolelor. Bastonul inteligent, un nod ESP32, S3 cu senzor de distanță time-of-flight, unitate inerțială, servomotor de indicare a direcției și motor de vibrații, percepe obstacolele frontale și comunică pericolul exclusiv haptic și mecanic, complet autonom. Vesta, un Raspberry Pi 5 cu cameră, microfon, difuzor și o matrice de cinci motoare de vibrații, rulează stiva de percepție și decizie (detecție de obiecte, segmentarea zonei navigabile, adâncime monoculară, clasificarea sunetelor de mediu, asistent vocal) și transmite bastonului comenzi de ghidare printr-o legătură Bluetooth Low Energy securizată.
Firul roșu al întregii arhitecturi este o teză de siguranță formulată explicit și aplicată consecvent la fiecare nivel: funcțiile de siguranță rulează local, autonom și neîntrerupt, iar tot ce este auxiliar( rețeaua, percepția avansată, asistentul vocal, diagnoza) poate cădea fără ca utilizatorul să fie pus în pericol. Din această teză derivă direct separarea fizică a timpului real de comunicație pe cele două nuclee ale microcontrolerului, semantica fail-safe „fără date ≠ liber”, pornirea nerestricționată în mod degradat cu auto-recuperare și securitatea impusă la nivelul stivei radio.
Contribuția de originalitate a proiectului este mecanismul gait-sync: indicația mecanică de direcție (poza servomotorului de sub degetul utilizatorului) nu se aplică la momentul sosirii comenzii, ci este sincronizată cu faza de balans a pasului, detectată local din semnătura inerțială a mersului. Decizia (ce poză) și momentul aplicării (când) provin astfel din surse diferite, decizia de la vestă, prin rețea. Momentul de la baston, local, astfel încât jitterul de transport nu poate perturba niciodată sincronizarea biomecanică. Documentul de față descrie integral arhitectura hardware și software, principiile de proiectare cu motivațiile și alternativele respinse, mecanismele de siguranță și criteriile de performanță atinse.

![](docs/Demo.mp4)
*Video demonstrativ — va fi adăugat în `docs/`*

---

## Cuprins
- [Descriere generală](#descriere-generală)
- [Platformă & biblioteci](#platformă--biblioteci)
- [Arhitectura software](#arhitectura-software)
- [Modele AI folosite](#modele-ai-folosite)
- [Flux operațional (overview)](#flux-operațional-overview)
- [Comunicare baston (BLE)](#comunicare-baston-ble)
- [Interfață web (dashboard)](#interfață-web-dashboard)
- [Setup & rulare](#setup--rulare)
- [Structura proiectului](#structura-proiectului)
- [Demo — poze & filmări](#demo--poze--filmări)

---

## Descriere generală

**Keryke-OS** rulează **local pe Raspberry Pi 5**, în vestă, și pe un **ESP32-S3**, în bastonul purtat de utilizator:
- **detectează obstacole și analizează traseul** din imaginea camerei, în timp real;
- **estimează distanța** printr-un senzor Time-of-Flight montat pe baston;
- **sincronizează orice corecție de direcție cu faza de balans a pasului** (Gait-Sync), astfel încât bastonul să nu fie redirecționat niciodată brusc, cât timp susține greutatea utilizatorului;
- **transmite informația de context** simultan pe două canale haptice — bastonul (servo, sincronizat cu mersul) și vesta (5 motoare de vibrație, instant, ca busolă direcțională pe piele).

Sistemul este **modular**, **independent de cloud** (cu excepția asistentului vocal, opțional) și proiectat să funcționeze **în timp real**, pe hardware purtat, alimentat din acumulator.

---

## Platformă & biblioteci

- **Platformă centrală:** Raspberry Pi 5 (8 GB) — percepție, fuziune de decizie, dashboard
- **Microcontroler baston:** ESP32-S3 (WROOM-2) — timp real, senzori, servo, BLE
- **ROS2 Humble + Docker:** orchestrarea nodurilor de percepție/decizie, rulate într-un singur container
- **Ultralytics YOLO (detect/seg):** detecție obstacole, segmentarea zonei sigure de mers
- **ONNX Runtime:** hartă de adâncime metrică (Depth Anything V2 Small, cuantizat int8)
- **tflite-runtime:** clasificarea sunetelor de mediu (YAMNet)
- **OpenWakeWord:** activare prin cuvânt-cheie, pentru asistentul vocal
- **Gemini 2.5 Flash (API):** asistent vocal multimodal, singura componentă care folosește internetul
- **BLE (Bluetooth Low Energy):** protocol binar propriu pentru legătura baston ↔ centrală
- **DRV2605L (I2C):** control independent pentru cele 5 actuatoare haptice de pe vestă

---

## Arhitectura software

Lansatorul pornește nouă executabile, consolidate deliberat în puține procese, cu afinitate CPU explicită:
- **`perception_container`** — detecția și segmentarea YOLO, executor multi-thread (nucleele 0–1)
- **`depth_node`** — harta de adâncime, proces separat, decuplat de rata detectorului (nucleul 2)
- **`decision_container`** — fuziunea de risc, naratorul determinist local și sinteza vocală
- **`ble_bridge`** — puntea bidirecțională cu bastonul (telemetrie IMU/ToF în, comenzi servo/vibrație afară)
- **`haptic_vest_node`** — control pentru cele 5 actuatoare de pe vestă
- **`dashboard`** — interfața de diagnoză (dezvoltare/jurizare)

Consolidarea proceselor nu e o preferință de stil, ci rezultatul unei măsurători reale: rulate separat, cele trei runtime-uri de inferență (PyTorch, ONNX Runtime, TensorFlow) își duplicau amprenta de memorie, ducând sistemul în swap. `decision_container` rămâne totuși separat de `perception_container`, intenționat: dacă un model de percepție cade, decizia și vocea rămân funcționale — utilizatorul nu rămâne în tăcere completă.

Comunicarea dintre noduri se face exclusiv prin topicuri ROS 2 publice, documentate ca un contract intern — nicio legătură privată între module.

---

## Modele AI folosite

| Model | Rol |
|---|---|
| **YOLOv8n** | detecție de obstacole + azimut din încadrare |
| **YOLOv8n-seg** (custom) | segmentarea zonei sigure de mers |
| **Depth Anything V2 Small** | hartă de adâncime metrică |
| **YAMNet** | clasificarea sunetelor de mediu (vehicule, sirene, claxoane, strigăte)(AudioSet) 
| **OpenWakeWord** | cuvânt de activare pentru asistentul vocal
| **Gemini 2.5 Flash** | asistent vocal multimodal, declanșat/serviciu cloud, prin API |

Principiul de selecție a fost disciplinat: niciun model nou fără o sarcină reală pe care s-o rezolve, și modele pre-antrenate oriunde a fost posibil.

---

## Flux operațional (overview)

1. **Camera → cadre** intră în pipeline-ul de percepție
2. **YOLO detect** găsește obstacolele; **YOLO seg** delimitează zona sigură de mers; **harta de adâncime** rulează asincron, pe nucleul dedicat
3. **`spatial_risk`** (10 Hz) combină detecțiile vizuale, distanța ToF + inerția de la baston, alertele audio și starea zonei sigure, cu o **prioritate explicită** (obstacol critic > alertă audio de pericol > obstacol ridicat > ieșire din zonă sigură > ...), nu un maxim numeric simplu
4. Decizia (`RiskDescriptor`) hrănește **două canale simultan**: `ble_bridge` → servo-ul bastonului (mod `gait_sync` sau `immediate`) și `haptic_vest_node` → cele 5 motoare de pe vestă (instant, ca busolă)
5. **Dashboard-ul** afișează fluxurile video adnotate și telemetria, pentru dezvoltare și jurizare — interfața utilizatorului final rămâne exclusiv haptică și vocală

---

## Comunicare baston (BLE)

- **Conectivitate:** Bluetooth Low Energy, protocol binar propriu, împerechere cu passkey (o singură dată, la configurare)
- **Telemetrie:** cadru binar de 50 de octeți (IMU + ToF), decodat și republicat ca JSON pe topicul `/keryke/sensors`
- **Comenzi:** poziție servo + mod de aplicare (`gait_sync`/`immediate`), impuls de vibrație
- **Robustețe:** reconectare automată cu interval crescător (3–15 s); regulă fail-safe — „fără date ≠ liber", prag de prospețime de 2,5 s pe toate intrările perisabile

---

## Interfață web (dashboard)

- **Video live:** trei fluxuri (cadru adnotat, segmentare, adâncime) prin HTTP/MJPEG, la 2 Hz
- **Evenimente live:** flux server-sent (SSE) — vorbire, alerte audio, decizia de risc, starea vestei, telemetria bastonului
- **Scop:** exclusiv dezvoltare și jurizare — interfața reală a utilizatorului e haptică și vocală, nu vizuală

---

## Setup & rulare

**1) Dependențe (minim):**
- ROS2 Humble, Docker
- Python 3.11+, Ultralytics (YOLO), ONNX Runtime, tflite-runtime, OpenWakeWord
- Firmware ESP32-S3 (Arduino/ESP-IDF), flash prin USB de pe laptop

**2) Pornire locală (exemplu):**
```bash
# 1. clonează repo-ul
git clone https://github.com/yokosumei/Keryke-OS.git
cd Keryke-OS

# 2. build + pornire container (Raspberry Pi)
docker compose up --build

# 3. în container: sursează ROS2 și lansează stack-ul
source /opt/ros/humble/setup.bash
ros2 launch keryke_bringup bringup.launch.py
```

**3) Configurare:** toată configurarea de exploatare se face prin variabile de mediu, fără modificări de cod — același container rulează pe laptopul de dezvoltare și pe dispozitiv.

---

## Structura proiectului

```
Keryke-OS/
├── firmware/cane_esp32/       # firmware ESP32-S3 (Gait-Sync, BLE, senzori)
├── ros2_ws/src/
│   ├── perception/            # yolo_detection, yolo_segmentation, depth_node
│   ├── spatial_risk/          # fuziunea de decizie
│   ├── ble_bridge/            # puntea cu bastonul
│   ├── haptic_vest/           # controlul celor 5 actuatoare de pe vestă
│   └── dashboard/             # interfața de diagnoză
├── docker/                    # definiția containerului
└── docs/                      # documentație tehnică
```

---

## Demo — poze & filmări

*Se adaugă în `media/` pe măsură ce testarea fizică avansează.*
