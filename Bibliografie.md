# Bibliografie Keryke-OS

Bibliografie a tehnicilor, bibliotecilor, modelelor pre-antrenate și tehnologiilor externe folosite în cod, cu sursa
canonică (repo oficial, documentație, paper sau tutorial de referință) pentru fiecare. Construită prin citirea directă a
fiecărui fișier sursă din repo, cu verificare pe web a originii fiecărei tehnici și a stării curente a surselor citate.

---

## 1. Percepție vizuală AI

**1.1 Ultralytics YOLOv8 / YOLO11** — `yolo_detection_node.py`: `from ultralytics import YOLO`, `yolov8n.pt`, clase COCO.
Repo oficial: [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) · Docs: [docs.ultralytics.com/models/yolov8](https://docs.ultralytics.com/models/yolov8)

**1.2 YOLOv8-seg (segmentare instanțe)** — `yolo_segmentation_node.py` (`--mode yolo`, `yolov8n-seg.pt`), variantă la thresholding HSV.
[Instance Segmentation, Ultralytics Docs](https://docs.ultralytics.com/tasks/segment)

**1.3 Export YOLO în format NCNN** — `yolo export model=yolov8n.pt format=ncnn imgsz=320`, motivat de eliminarea PyTorch la inferență pe ARM.
[NCNN Integration](https://docs.ultralytics.com/integrations/ncnn) · [YOLO on Raspberry Pi](https://docs.ultralytics.com/guides/raspberry-pi) · Repo motor: [Tencent/ncnn](https://github.com/Tencent/ncnn)

**1.4 Depth Anything V2 Small (ONNX, int8)** — `depth_node.py`, model de la Hugging Face, licență Apache 2.0; convenție de **disparitate** (nu adâncime canonică), identică MiDaS.
Repo oficial: [DepthAnything/Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2) · Paper: [arXiv:2406.09414](https://arxiv.org/abs/2406.09414) (NeurIPS 2024) · Model ONNX: [onnx-community/depth-anything-v2-small](https://huggingface.co/onnx-community/depth-anything-v2-small) · Sursa convenției disparity=1/depth: [isl-org/MiDaS](https://github.com/isl-org/MiDaS)
*Notă:* pagina HF a modelului ONNX e convertită special pentru Transformers.js (rulare în browser), nu documentat explicit pentru `onnxruntime` Python pe CPU ARM — funcționează, dar e o utilizare în afara cazului descris de autori.

**1.5 Metric3D v2 (înlocuit — referință istorică)** — model anterior, prea lent pe Pi (68–96 s/cadru).
Repo oficial: [YvanYin/Metric3D](https://github.com/YvanYin/Metric3D) · Paper: [arXiv:2404.15506](https://arxiv.org/abs/2404.15506)

**1.6 ByteTrack (planificat, nu încă activ)** — `perception_container.py` menționează tracking+TTC viitor via `model.track()`.
Paper original: [arXiv:2110.06864](https://arxiv.org/abs/2110.06864) (ECCV 2022) · Repo activ menținut: [FoundationVision/ByteTrack](https://github.com/FoundationVision/ByteTrack) (repo-ul original `ifzhang/ByteTrack` citat în paper e mai puțin activ) · [Multi-Object Tracking, Ultralytics Docs](https://docs.ultralytics.com/modes/track) (`model.track()`, `bytetrack.yaml`)

**1.7 ONNX Runtime** — `depth_node.py`: `ort.InferenceSession`, `CPUExecutionProvider`, `intra_op_num_threads`.
Repo oficial: [microsoft/onnxruntime](https://github.com/microsoft/onnxruntime) · [Threading/Performance Tuning](https://github.com/microsoft/onnxruntime/blob/gh-pages/docs/performance/tune-performance/threading.md)

**1.8 COCO dataset** — sursa claselor de detecție (`RELEVANT_CLASSES`).
[cocodataset.org](https://cocodataset.org) · Lin et al., „Microsoft COCO: Common Objects in Context", [arXiv:1405.0312](https://arxiv.org/abs/1405.0312) (ECCV 2014)

**1.9 Roboflow** — adnotare dataset custom (~150 poze, o clasă) pentru segmentarea drumului la demo.
[roboflow.com](https://roboflow.com)

**1.10 Cityscapes și ADE20K** — opțiuni viitoare pentru segmentare reală drum/trotuar.
[Cityscapes](https://www.cityscapes-dataset.com) · [CSAILVision/ADE20K](https://github.com/CSAILVision/ADE20K), MIT (Zhou et al., CVPR 2017)

---

## 2. Percepție audio AI și voce

**2.1 YAMNet** — `audio_event_node.py`, model `.tflite` antrenat pe AudioSet (521 clase); codul citează explicit sursa claselor.
Clase: [tensorflow/models — yamnet_class_map.csv](https://github.com/tensorflow/models/blob/master/research/audioset/yamnet/yamnet_class_map.csv) · Paper AudioSet: Gemmeke et al., ICASSP 2017, doi:10.1109/ICASSP.2017.7952261 · Paper arhitectură: Hershey et al., „CNN Architectures for Large-Scale Audio Classification", ICASSP 2017 · Model: [Kaggle Models — google/yamnet](https://www.kaggle.com/models/google/yamnet/tfLite/classification-tflite/1)

**2.2 AudioSet** — [research.google.com/audioset](https://research.google.com/audioset/) (2.084.320 clipuri etichetate, ontologie ierarhică)

**2.3 tflite-runtime → ai-edge-litert** — `audio_event_node.py` are fallback explicit `tflite_runtime` → `ai_edge_litert`.
PyPI (nou): [ai-edge-litert](https://pypi.org/project/ai-edge-litert/) · Anunț oficial rebranding: [developers.googleblog.com/tensorflow-lite-is-now-litert](https://developers.googleblog.com/en/tensorflow-lite-is-now-litert/) · [Ghid migrare oficial](https://ai.google.dev/edge/litert/migration) · Repo: [google-ai-edge/LiteRT](https://github.com/google-ai-edge/LiteRT)
*Notă confirmată:* ghidul oficial de migrare spune explicit „replace the PIP package from `tflite-runtime` to `ai-edge-litert`" — fallback-ul din cod e exact calea recomandată de Google.

**2.4 OpenWakeWord** — `wake_node.py`, model preantrenat `hey_jarvis`, `inference_framework="onnx"`.
Repo oficial: [dscripka/openWakeWord](https://github.com/dscripka/openWakeWord) · [Model card hey_jarvis](https://github.com/dscripka/openWakeWord/blob/main/docs/models/hey_jarvis.md)

**2.5 PyAudio** — `wake_node.py`, captură stream 48 kHz, downsampling manual la 16 kHz.
Documentație oficială (Hubert Pham): [people.csail.mit.edu/hubert/pyaudio/docs](https://people.csail.mit.edu/hubert/pyaudio/docs/)

**2.6 espeak-ng** — `tts_node.py`, motor TTS, voce română (`-v ro`).
Repo oficial: [espeak-ng/espeak-ng](https://github.com/espeak-ng/espeak-ng)

**2.7 Piper TTS (upgrade posibil, menționat în cod)**
Repo original, MIT, **arhivat octombrie 2025**: [rhasspy/piper](https://github.com/rhasspy/piper) · Dezvoltare activă mutată la [OHF-Voice/piper1-gpl](https://github.com/OHF-Voice/piper1-gpl), licență **GPL-3.0**.
*Notă confirmată prin verificare (iulie 2026):* migrarea și schimbarea de licență sunt reale și definitive — dacă proiectul reia ideea Piper, orice cod nou trebuie legat de fork-ul GPL-3.0, nu de repo-ul original arhivat (MIT). Relevant direct pentru licențierea proiectului, dat fiind mizapierea de reutilizare/concurs. Există și fork-uri alternative cu licență permisivă (ex. `piper-plus`, MIT), utile de menționat doar dacă se dorește evitarea GPL.

**2.8 Web Speech API (fallback vocal browser, `dashboard_node.py`)**
MDN: [SpeechSynthesis](https://developer.mozilla.org/en-US/docs/Web/API/SpeechSynthesis) · [SpeechSynthesisUtterance](https://developer.mozilla.org/en-US/docs/Web/API/SpeechSynthesisUtterance)

**2.9 Bug PortAudio `maxInputChannels=0` pe microfon USB** — motivul documentat în cod pentru folosirea `arecord` CLI în loc de sounddevice/PyAudio.
Nu există un issue canonic unic pentru placa AB13X. Precedent apropiat: [forum Raspberry Pi](https://forums.raspberrypi.com/viewtopic.php?t=16525) și probleme similare pe [PortAudio/portaudio issues](https://github.com/PortAudio/portaudio/issues). De citat ca observație empirică proprie + context de forum, nu ca sursă oficială.

**2.10 Google Gemini API** — `brain_node.py`: `import google.generativeai as genai`, `gemini-2.5-flash`, input audio+imagine+text.
Documentație: [ai.google.dev/gemini-api/docs](https://ai.google.dev/gemini-api/docs) · SDK folosit efectiv (**deprecated**): [google-gemini/deprecated-generative-ai-python](https://github.com/google-gemini/deprecated-generative-ai-python) · SDK nou recomandat: [googleapis/python-genai](https://github.com/googleapis/python-genai)
*Notă confirmată prin verificare (iulie 2026):* deprecarea e definitivă și oficial documentată — README-ul repo-ului vechi confirmă că e „legacy" și recomandă migrarea. Recomandare de mentenanță: migrare la `google-genai` cât mai curând, cu import `from google import genai` și `Client()` în loc de `genai.configure()`; SDK-ul vechi primește doar fix-uri critice.

---

## 3. Infrastructură software / ROS2

**3.1 ROS2 Humble Hawksbill** — bază pentru tot stack-ul de noduri (`rclpy`, `Node`, publisher/subscriber, launch files).
[Documentație oficială](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)

**3.2 rclpy issue #1025 (citat direct în `perception_container.py`)** — codul citează acest issue ca justificare pentru „MultiThreadedExecutor + GIL nu dau paralelism real pt. CPU-bound".
Issue real, verificat: [github.com/ros2/rclpy/issues/1025](https://github.com/ros2/rclpy/issues/1025) — „Release Python GIL in `Subscription::take_message`"
*Notă confirmată prin verificare (iulie 2026):* discrepanța semnalată e reală. Issue-ul e depus de mentenanții `rosbridge_server`, care profilaseră performanța la *primirea* mesajelor pe subiecte diverse (I/O, `take_message`), nu execuția callback-urilor CPU-bound de inferență (onnxruntime/NCNN). Confirmă premisa generală — GIL-ul serializează codul Python într-un proces — dar nu e literalmente sursa afirmației din cod. Recomandare: comentariul din cod ar trebui reformulat ca „vezi și" / „context conex", nu ca citat direct al sursei problemei.

**3.3 ReentrantCallbackGroup** — `yolo_segmentation_node.py`, permite rularea concurentă a `_on_image`/`_on_timer`.
[Using Callback Groups, ROS 2 Docs (Humble)](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html)

**3.4 CycloneDDS** — RMW alternativ + `ROS_DOMAIN_ID=42` (`docker/docker-compose.yml`).
Repo oficial: [eclipse-cyclonedds/cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds) · [Working with Eclipse CycloneDDS, ROS 2 Docs](https://docs.ros.org/en/rolling/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html)

**3.5 cv_bridge** — conversie `sensor_msgs/Image` ↔ OpenCV ndarray, în toate nodurile de percepție.
Repo oficial: [ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv)

**3.6 vision_msgs** — `Detection2D`, `Detection2DArray`, `BoundingBox2D`, `ObjectHypothesisWithPose`.
Repo oficial: [ros-perception/vision_msgs](https://github.com/ros-perception/vision_msgs)

**3.7 OpenCV** — `cv2.inRange` + morphology (segmentare HSV a drumului), `cv2.applyColorMap(..., COLORMAP_JET)` (colorizare adâncime în dashboard).
[Thresholding — inRange](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html) · [Morphological Transformations](https://docs.opencv.org/4.13.0/d9/d61/tutorial_py_morphological_ops.html) · [ColorMaps in OpenCV](https://docs.opencv.org/4.13.0/d3/d50/group__imgproc__colormap.html)

**3.8 Flask + MJPEG multipart streaming** — `dashboard_node.py`: `Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")`.
[Streaming Contents, Flask Docs](https://flask.palletsprojects.com/en/stable/patterns/streaming/) · Tutorial de referință: [Video Streaming with Flask, Miguel Grinberg](https://blog.miguelgrinberg.com/post/video-streaming-with-flask) · Alternativ: [PyImageSearch](https://pyimagesearch.com/2019/09/02/opencv-stream-video-to-web-browser-html-page/)

**3.9 Server-Sent Events (SSE)** — `dashboard_node.py`: `/events`, `text/event-stream`, push `risk`/`audio_alert`/`speak`/`sensor_data`.
[Using server-sent events, MDN](https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events/Using_server-sent_events) · [Specificație normativă, WHATWG HTML](https://html.spec.whatwg.org/dev/server-sent-events.html)

**3.10 Three.js (r128, via cdnjs)** — `dashboard_node.py`, vizualizare 3D IMU baston (Scene, PerspectiveCamera, CylinderGeometry, GridHelper).
[threejs.org](https://threejs.org/) · [GridHelper docs](https://threejs.org/docs/pages/GridHelper.html) · [Versiune r128 pe cdnjs](https://cdnjs.com/libraries/three.js/r128)

**3.11 Docker Compose** — containerul `hive`, `network_mode: host`.
[Networking in Compose](https://docs.docker.com/compose/how-tos/networking/) · [Define services](https://docs.docker.com/reference/compose-file/services/)

---

## 4. Hardware și firmware (ESP32-S3, baston)

**4.1 Adafruit MPU6050 + Adafruit Unified Sensor** — `esp32_keryke_ble.ino`, `varianta David.cpp`, IMU.
[adafruit/Adafruit_MPU6050](https://github.com/adafruit/Adafruit_MPU6050) · [adafruit/Adafruit_Sensor](https://github.com/adafruit/adafruit_sensor) · [Tutorial Adafruit Learning System](https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/arduino)

**4.2 Adafruit VL53L1X** — ToF, `VL53L1X_SetDistanceMode`, `setTimingBudget`, `VL53L1X_SetInterMeasurementInMs`.
[adafruit/Adafruit_VL53L1X](https://github.com/adafruit/Adafruit_VL53L1X) · [Datasheet oficial STMicroelectronics (PDF)](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)

**4.3 ESP32Servo** — varianta TCP (`varianta David.cpp`), control SG90 prin LEDC.
[madhephaestus/ESP32Servo](https://github.com/madhephaestus/ESP32Servo)

**4.4 ESP32 BLE GATT server securizat (Bluedroid)** — `esp32_keryke_ble.ino`: `BLEDevice`, `BLEServer`, `BLESecurity`, `BLE2902`, `ESP_LE_AUTH_REQ_SC_MITM_BOND`, passkey static, `ESP_IO_CAP_OUT`.
Librăria BLE: [espressif/arduino-esp32](https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE) · [API docs BLE](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ble.html) · [GAP API, ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html) · [SMP (Security Manager Protocol), ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/ble/smp.html) · Exemplu pairing cu passkey: [nkolban/esp32-snippets](https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/Arduino/security/BLE_client/BLE_client_passkey/BLE_client_passkey.ino) · [„Implementing BLE Security on ESP32", dev.to](https://dev.to/makepkg/implementing-ble-security-on-esp32-le-secure-connections-the-hard-way-2kb9)

**4.5 FreeRTOS pe ESP32 (dual-core)** — task-uri (`blePushTask`, `logTask`), cozi (`QueueHandle_t`), secțiuni critice (`portMUX_TYPE`).
[xTaskCreate, FreeRTOS oficial](https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate) · [FreeRTOS (IDF), Espressif](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html) · [FreeRTOS SMP / task pinning](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html)

**4.6 WS2812 / `rgbLedWrite()`** — LED de stare integrat pe DevKitC-1 (`RGB_BUILTIN`).
Implementare: [esp32-hal-rgb-led, arduino-esp32](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-rgb-led.h) · [Exemplu oficial BlinkRGB.ino](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/GPIO/BlinkRGB/BlinkRGB.ino)

**4.7 bleak** — `firmware/pi_client_ble.py`, client BLE Python de pe Raspberry Pi.
Repo oficial: [hbldh/bleak](https://github.com/hbldh/bleak)

**4.8 Picamera2** — `host/camera_publisher.py`: `from picamera2 import Picamera2`.
Repo oficial: [raspberrypi/picamera2](https://github.com/raspberrypi/picamera2) · [Manual oficial (PDF)](https://pip.raspberrypi.com/documents/RP-008156-DS-picamera2-manual.pdf)

**4.9 Sony IMX500 / Raspberry Pi AI Camera** — senzor folosit doar ca sursă de imagine (nu NPU on-chip, vezi `imx500_bridge_node.py`).
[AI Camera, Raspberry Pi Docs](https://www.raspberrypi.com/documentation/accessories/ai-camera.html) · [Sony AITRIOS — IMX500](https://www.aitrios.sony-semicon.com/edge-ai-devices/imx500)

**4.10 PCA9685** — driver PWM I2C pentru motoarele haptice de pe vestă (menționat în CLAUDE.md, nu încă implementat în cod).
[Datasheet oficial NXP (PDF)](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf) · [adafruit/Adafruit-PWM-Servo-Driver-Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)

---

## 5. Algoritmul Gait-Sync — status și context bibliografic

`detectSwingPhase()` (identic în `esp32_keryke_ble.ino` și `varianta David.cpp`): fereastră glisantă de 10 eșantioane, prag pe scăderea `accelMag` sub media ferestrei ȘI vârf `|gyroZ|`, debounce 500 ms.

**Concluzie confirmată prin căutare:** nu există o sursă directă — combinația exactă de parametri (fereastră de 10, prag 0,5 pe accelerație, prag 0,3 rad/s pe giroscop, debounce 500 ms) nu apare în literatura găsită. Algoritmul e original. Referințe conceptuale utile pentru bibliografia de concurs (context, nu sursă):

- Willemsen, Bloemhof, Boom — „Automatic stance-swing phase detection from accelerometer data for peroneal nerve stimulation", *IEEE Trans. Biomed. Eng.*, 1990 — [ieeexplore.ieee.org/document/64463](https://ieeexplore.ieee.org/document/64463/) (prag pe accelerometru, fără giroscop — precursor conceptual)
- „The kinematics of the swing phase obtained from accelerometer and gyroscope measurements", IEEE — [ieeexplore.ieee.org/document/651817](https://ieeexplore.ieee.org/document/651817) (combină accel+gyro, cel mai apropiat conceptual)
- „Stance and Swing Detection Based on the Angular Velocity of Lower Limb Segments During Walking", *Frontiers in Neurorobotics*, 2019 — [doi:10.3389/fnbot.2019.00057](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2019.00057/full)
- „A Computer Vision and Depth Sensor-Powered Smart Cane for Real-Time Obstacle Detection and Navigation Assistance for the Visually Impaired", arXiv, 2025 — [arXiv:2508.16698](https://arxiv.org/abs/2508.16698) (proiect recent similar ca scop — baston + IMU + feedback haptic — util ca referință de context)
