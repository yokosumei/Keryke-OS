#!/usr/bin/env python3
"""
Porneste tot stack-ul de perceptie/decizie/audio dintr-o comanda:
    ros2 launch hive_perception bringup.launch.py

Camera (host/camera_publisher.py, Picamera2) NU e ROS2 -- ramane
separata, in propriul terminal pe host, inainte de acest launch
(imx500_bridge reincearca conexiunea TCP pana porneste camera, nu
crapa daca ordinea e inversata).

yolo_detection si yolo_segmentation NU mai sunt procese separate --
ruleaza consolidate in perception_container (vezi perception_container.py),
ca sa nu duplice runtime-ul greu (torch/NCNN) de doua ori in memorie.

depth_node ruleaza SEPARAT (proces propriu) -- consolidat cu YOLO+
segmentare, Python GIL nu dadea paralelism real intre threadurile lor
CPU-bound (timp de inferenta crescator, 1.6-3.6s fata de 1194ms izolat).
Separat, cu afinitate CPU proprie (KERYKE_DEPTH_CORES), primeste timp de
procesor garantat de la sistemul de operare -- vezi depth_node.py.

spatial_risk, narrator si tts ruleaza consolidate in decision_container
(Python pur, fara framework ML greu -- consolidarea aici reduce numarul
de procese, nu resursele). Ramane INTENTIONAT separat de
perception_container: daca perceptia pica, narator/tts tot pot vorbi
(ex. raspunsuri Gemini), utilizatorul nu ramane in tacere completa.

audio_event_ros, wake si brain raman separate (ritm/IO propriu, nu
beneficiaza de consolidare).

ble_bridge (puntea BLE catre bastonul ESP32) nu e inclus aici -- necesita
bastonul imperecheat (bluetoothctl, passkey) si pornit. `ros2 run
hive_perception ble_bridge` separat cand e cazul.

haptic_vest (vesta-busola pe PCA9685) E inclus: daca hardware-ul I2C
lipseste, nodul trece singur pe backend mock si doar logheaza -- pornirea
stack-ului nu depinde de prezenta fizica a vestei.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

NODES = [
    "imx500_bridge",
    "perception_container",
    "depth_node",
    "decision_container",
    "audio_event_ros",
    "dashboard",
    "wake",
    "brain",
    "haptic_vest",
]


def generate_launch_description():
    return LaunchDescription([
        Node(package="hive_perception", executable=exe, name=exe, output="screen")
        for exe in NODES
    ])
