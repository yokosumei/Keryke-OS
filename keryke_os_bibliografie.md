# Bibliografie Keryke-OS

Tehnici, biblioteci, modele pre-antrenate și tehnologii externe folosite în cod, cu sursa canonică pentru fiecare.

---

## 1. Percepție vizuală AI

**1.1 Ultralytics YOLOv8 / YOLO11** — detecție de obiecte, `yolov8n.pt`, clase COCO (`yolo_detection_node.py`).
[ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) · [docs.ultralytics.com/models/yolov8](https://docs.ultralytics.com/models/yolov8)

**1.2 YOLOv8-seg** — segmentarea zonei sigure de mers, `yolov8n-seg.pt` (`yolo_segmentation_node.py`).
[Instance Segmentation, Ultralytics Docs](https://docs.ultralytics.com/tasks/segment)

**1.3 Export YOLO în format NCNN** — inferență fără dependența de PyTorch pe ARM (`yolo export ... format=ncnn imgsz=320`).
[NCNN Integration, Ultralytics](https://docs.ultralytics.com/integrations/ncnn) · [Tencent/ncnn](https://github.com/Tencent/ncnn)

**1.4 Depth Anything V2 Small (ONNX, int8)** — hartă de adâncime metrică, scalată static față de ToF (`depth_node.py`).
[DepthAnything/Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2) · Paper: [arXiv:2406.09414](https://arxiv.org/abs/2406.09414) · Model: [onnx-community/depth-anything-v2-small](https://huggingface.co/onnx-community/depth-anything-v2-small) · Convenția disparitate=1/adâncime: [isl-org/MiDaS](https://github.com/isl-org/MiDaS)

**1.5 Metric3D v2** — model de adâncime testat inițial, înlocuit cu Depth Anything V2 (prea lent, 68–96 s/cadru pe Pi).
[YvanYin/Metric3D](https://github.com/YvanYin/Metric3D) · Paper: [arXiv:2404.15506](https://arxiv.org/abs/2404.15506)

**1.6 ByteTrack** — urmărire în timp + estimare timp-până-la-coliziune, planificat peste detecțiile YOLO (`model.track()`).
Paper: [arXiv:2110.06864](https://arxiv.org/abs/2110.06864) · [FoundationVision/ByteTrack](https://github.com/FoundationVision/ByteTrack) · [Multi-Object Tracking, Ultralytics Docs](https://docs.ultralytics.com/modes/track)

**1.7 ONNX Runtime** — motor de inferență pentru modelul de adâncime, `CPUExecutionProvider` (`depth_node.py`).
[microsoft/onnxruntime](https://github.com/microsoft/onnxruntime) · [Threading/Performance Tuning](https://github.com/microsoft/onnxruntime/blob/gh-pages/docs/performance/tune-performance/threading.md)

**1.8 COCO dataset** — sursa claselor de detecție (`RELEVANT_CLASSES`).
[cocodataset.org](https://cocodataset.org) · Lin et al., [arXiv:1405.0312](https://arxiv.org/abs/1405.0312) (ECCV 2014)

**1.9 Roboflow** — adnotare dataset custom (~150 poze, o clasă) pentru segmentarea drumului.
[roboflow.com](https://roboflow.com)

**1.10 Cityscapes / ADE20K** — opțiuni evaluate pentru segmentare reală drum/trotuar.
[Cityscapes](https://www.cityscapes-dataset.com) · [CSAILVision/ADE20K](https://github.com/CSAILVision/ADE20K)

---

## 2. Percepție audio AI și voce

**2.1 YAMNet** — clasificarea sunetelor de mediu, model `.tflite` pe AudioSet, 521 clase (`audio_event_node.py`).
Clase: [yamnet_class_map.csv, tensorflow/models](https://github.com/tensorflow/models/blob/master/research/audioset/yamnet/yamnet_class_map.csv) · Paper AudioSet: Gemmeke et al., ICASSP 2017 · Paper arhitectură: Hershey et al., ICASSP 2017 · Model: [Kaggle — google/yamnet](https://www.kaggle.com/models/google/yamnet/tfLite/classification-tflite/1)

**2.2 AudioSet** — sursa datelor de antrenare a YAMNet.
[research.google.com/audioset](https://research.google.com/audioset/)

**2.3 tflite-runtime / ai-edge-litert** — runtime pentru YAMNet pe Pi, cu fallback între cele două pachete (`audio_event_node.py`).
[ai-edge-litert, PyPI](https://pypi.org/project/ai-edge-litert/) · [google-ai-edge/LiteRT](https://github.com/google-ai-edge/LiteRT) · [Ghid migrare oficial](https://ai.google.dev/edge/litert/migration)

**2.4 OpenWakeWord** — cuvânt de activare, model preantrenat `hey_jarvis`, `onnx` (`wake_node.py`).
[dscripka/openWakeWord](https://github.com/dscripka/openWakeWord) · [Model card hey_jarvis](https://github.com/dscripka/openWakeWord/blob/main/docs/models/hey_jarvis.md)

**2.5 PyAudio** — captură microfon 48 kHz, downsampling manual la 16 kHz (`wake_node.py`).
[Documentație oficială](https://people.csail.mit.edu/hubert/pyaudio/docs/)

**2.6 espeak-ng** — motor TTS, voce română (`-v ro`) (`tts_node.py`).
[espeak-ng/espeak-ng](https://github.com/espeak-ng/espeak-ng)

**2.7 Piper TTS** — evaluat ca upgrade posibil față de espeak-ng, pentru voce naturală offline.
[OHF-Voice/piper1-gpl](https://github.com/OHF-Voice/piper1-gpl)

**2.8 Web Speech API** — fallback vocal în browser, pe interfața de diagnoză (`dashboard_node.py`).
[SpeechSynthesis, MDN](https://developer.mozilla.org/en-US/docs/Web/API/SpeechSynthesis) · [SpeechSynthesisUtterance, MDN](https://developer.mozilla.org/en-US/docs/Web/API/SpeechSynthesisUtterance)

**2.9 arecord (CLI)** — captură audio de pe microfonul USB, folosit în locul sounddevice/PyAudio din cauza unui bug de driver (`maxInputChannels=0`).
Context: [forum Raspberry Pi](https://forums.raspberrypi.com/viewtopic.php?t=16525) · [PortAudio/portaudio, issues](https://github.com/PortAudio/portaudio/issues)

**2.10 Google Gemini API** — asistent vocal multimodal (audio+imagine+text), `gemini-2.5-flash` (`brain_node.py`).
[Documentație Gemini API](https://ai.google.dev/gemini-api/docs) · SDK: [googleapis/python-genai](https://github.com/googleapis/python-genai)

---

## 3. Infrastructură software / ROS2

**3.1 ROS2 Humble Hawksbill** — bază pentru tot stack-ul de noduri (`rclpy`, publisher/subscriber, launch files).
[Documentație oficială](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)

**3.2 MultiThreadedExecutor + GIL** — justificare pentru consolidarea de procese descrisă în arhitectură, în loc de paralelism real pe thread-uri Python (`perception_container.py`).
[github.com/ros2/rclpy/issues/1025](https://github.com/ros2/rclpy/issues/1025) — „Release Python GIL in `Subscription::take_message`"

**3.3 ReentrantCallbackGroup** — rulare concurentă a `_on_image`/`_on_timer` (`yolo_segmentation_node.py`).
[Using Callback Groups, ROS 2 Docs (Humble)](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html)

**3.4 CycloneDDS** — RMW folosit, cu `ROS_DOMAIN_ID=42` (`docker/docker-compose.yml`).
[eclipse-cyclonedds/cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds) · [Working with Eclipse CycloneDDS](https://docs.ros.org/en/rolling/Installation/RMW-Implementations/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html)

**3.5 cv_bridge** — conversie `sensor_msgs/Image` ↔ OpenCV ndarray, în toate nodurile de percepție.
[ros-perception/vision_opencv](https://github.com/ros-perception/vision_opencv)

**3.6 vision_msgs** — tipuri de mesaje pentru detecții (`Detection2D`, `Detection2DArray`, `BoundingBox2D`).
[ros-perception/vision_msgs](https://github.com/ros-perception/vision_msgs)

**3.7 OpenCV** — segmentare HSV a drumului (`cv2.inRange` + morphology), colorizare hartă de adâncime (`cv2.applyColorMap`).
[Thresholding — inRange](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html) · [Morphological Transformations](https://docs.opencv.org/4.13.0/d9/d61/tutorial_py_morphological_ops.html) · [ColorMaps in OpenCV](https://docs.opencv.org/4.13.0/d3/d50/group__imgproc__colormap.html)

**3.8 Flask + MJPEG multipart streaming** — fluxurile video ale dashboard-ului (`dashboard_node.py`).
[Streaming Contents, Flask Docs](https://flask.palletsprojects.com/en/stable/patterns/streaming/) · [Video Streaming with Flask, Miguel Grinberg](https://blog.miguelgrinberg.com/post/video-streaming-with-flask)

**3.9 Server-Sent Events (SSE)** — canalul `/events` al dashboard-ului (risc, alerte audio, vorbire, telemetrie).
[Using server-sent events, MDN](https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events/Using_server-sent_events) · [Specificație WHATWG HTML](https://html.spec.whatwg.org/dev/server-sent-events.html)

**3.10 Three.js (r128, via cdnjs)** — vizualizare 3D a orientării bastonului pe dashboard.
[threejs.org](https://threejs.org/) · [GridHelper docs](https://threejs.org/docs/pages/GridHelper.html) · [r128 pe cdnjs](https://cdnjs.com/libraries/three.js/r128)

**3.11 Docker Compose** — containerul `hive`, `network_mode: host`.
[Networking in Compose](https://docs.docker.com/compose/how-tos/networking/) · [Define services](https://docs.docker.com/reference/compose-file/services/)

---

## 4. Hardware și firmware (ESP32-S3, baston)

**4.1 Adafruit MPU6050 + Adafruit Unified Sensor** — IMU (`esp32_keryke_ble.ino`, `varianta David.cpp`).
[adafruit/Adafruit_MPU6050](https://github.com/adafruit/Adafruit_MPU6050) · [adafruit/Adafruit_Sensor](https://github.com/adafruit/adafruit_sensor)

**4.2 Adafruit VL53L1X** — senzor ToF (`VL53L1X_SetDistanceMode`, `setTimingBudget`).
[adafruit/Adafruit_VL53L1X](https://github.com/adafruit/Adafruit_VL53L1X) · [Datasheet ST (PDF)](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)

**4.3 ESP32Servo** — control servomotor SG90 prin LEDC (`varianta David.cpp`).
[madhephaestus/ESP32Servo](https://github.com/madhephaestus/ESP32Servo)

**4.4 ESP32 BLE GATT server securizat (Bluedroid)** — legătura baston↔centrală (`BLEDevice`, `BLESecurity`, `ESP_LE_AUTH_REQ_SC_MITM_BOND`, passkey static) (`esp32_keryke_ble.ino`).
[Librăria BLE, arduino-esp32](https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE) · [API docs BLE](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ble.html) · [GAP API, ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html) · [SMP, ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/ble/smp.html)

**4.5 FreeRTOS pe ESP32 (dual-core)** — task-uri, cozi și secțiuni critice pentru separarea nucleelor (`blePushTask`, `logTask`, `QueueHandle_t`, `portMUX_TYPE`).
[xTaskCreate, FreeRTOS oficial](https://www.freertos.org/Documentation/02-Kernel/04-API-references/01-Task-creation/01-xTaskCreate) · [FreeRTOS (IDF), Espressif](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos_idf.html) · [FreeRTOS SMP / task pinning](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html)

**4.6 WS2812 / `rgbLedWrite()`** — LED de stare integrat pe DevKitC-1 (`RGB_BUILTIN`).
[esp32-hal-rgb-led, arduino-esp32](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-rgb-led.h)

**4.7 bleak** — client BLE Python de pe Raspberry Pi (`pi_client_ble.py`).
[hbldh/bleak](https://github.com/hbldh/bleak)

**4.8 Picamera2** — captură cameră pe sistemul-gazdă (`camera_publisher.py`).
[raspberrypi/picamera2](https://github.com/raspberrypi/picamera2) · [Manual oficial (PDF)](https://pip.raspberrypi.com/documents/RP-008156-DS-picamera2-manual.pdf)

**4.9 Sony IMX500 / Raspberry Pi AI Camera** — senzor folosit ca sursă de imagine (`imx500_bridge_node.py`).
[AI Camera, Raspberry Pi Docs](https://www.raspberrypi.com/documentation/accessories/ai-camera.html) · [Sony AITRIOS — IMX500](https://www.aitrios.sony-semicon.com/edge-ai-devices/imx500)

**4.10 PCA9685** — driver PWM I2C planificat pentru motoarele haptice de pe vestă.
[Datasheet NXP (PDF)](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf) · [adafruit/Adafruit-PWM-Servo-Driver-Library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)

---

## 5. Gait-Sync — context bibliografic

`detectSwingPhase()` (`esp32_keryke_ble.ino`, `varianta David.cpp`): fereastră glisantă de 10 eșantioane, prag pe scăderea `accelMag` sub media ferestrei ȘI vârf `|gyroZ|`, debounce 500 ms — algoritm original, fără sursă directă în literatură. Referințe conceptuale folosite ca punct de plecare:

- Willemsen, Bloemhof, Boom — „Automatic stance-swing phase detection from accelerometer data for peroneal nerve stimulation", *IEEE Trans. Biomed. Eng.*, 1990 — [ieeexplore.ieee.org/document/64463](https://ieeexplore.ieee.org/document/64463/)
- „The kinematics of the swing phase obtained from accelerometer and gyroscope measurements", IEEE — [ieeexplore.ieee.org/document/651817](https://ieeexplore.ieee.org/document/651817)
- „Stance and Swing Detection Based on the Angular Velocity of Lower Limb Segments During Walking", *Frontiers in Neurorobotics*, 2019 — [doi:10.3389/fnbot.2019.00057](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2019.00057/full)
- „A Computer Vision and Depth Sensor-Powered Smart Cane for Real-Time Obstacle Detection and Navigation Assistance for the Visually Impaired", arXiv, 2025 — [arXiv:2508.16698](https://arxiv.org/abs/2508.16698)
