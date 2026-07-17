#!/usr/bin/env python3
"""
perception_container.py -- yolo_detection si yolo_segmentation intr-un
singur proces, in loc de doua separate.

Fiecare proces Python separat platea din nou overhead-ul de interpretor
+ starea interna a framework-ului (torch/onnxruntime), nu doar CPU-ul de
inferenta -- pe Pi 4B, cele trei concurente duceau la swap activ si load
average 16.93/4 nuclee. Consolidate aici, cu MultiThreadedExecutor (nu
SingleThreaded) ca sa nu se blocheze reciproc.

depth_node NU mai e aici -- a fost scos intr-un proces separat (vezi
depth_node.py). Motiv: MultiThreadedExecutor + Python GIL nu dau
paralelism real pentru munca CPU-bound (onnxruntime/NCNN) in ACELASI
proces -- GIL-ul serializeaza executia, doar operatiile I/O beneficiaza
de threading (github.com/ros2/rclpy/issues/1025). Masurat: depth_node
consolidat aici ajungea la 1.6-3.6s/cadru (crescator in timp), fata de
1194ms izolat -- separarea + afinitate CPU explicita (vezi mai jos)
rezolva asta la radacina, nu doar muta problema.

cv2.setNumThreads(1): OpenCV, implicit, incearca sa foloseasca TOATE
nucleele pentru orice operatie (resize/cvtColor/morphology) -- intr-un
proces care ruleaza si NCNN (care isi are propriile threaduri), asta
inseamna doua biblioteci concurand pentru aceleasi nuclee. Segmentarea
HSV e ieftina, nu are nevoie de threading propriu.

Afinitate CPU (os.sched_setaffinity, vezi KERYKE_PERCEPTION_CORES): fara
ea, o limita de threaduri e doar o sugestie catre biblioteca -- tot
"crede" ca are toate nucleele placii. Afinitatea e impusa de kernel:
procesul FIZIC nu poate rula pe alt nucleu decat cele alocate.

MultiThreadedExecutor(num_threads=...): implicit, foloseste
multiprocessing.cpu_count() -- numarul de nuclee din TOT sistemul (4),
nu cele alocate procesului prin afinitate (2). Fara sa-l fixam explicit,
executorul ROS2 insusi porneste mai multe threaduri Python decat nuclee
disponibile in setul restrictionat -- suprasolicitare in interiorul
propriilor 2 nuclee, separat de orice face NCNN intern. Il sincronizam
cu numarul de nuclee din KERYKE_PERCEPTION_CORES.

audio_event_node si brain_node RAMAN separate -- audio are ritm propriu
(ferestre de 1s), brain e I/O-bound pe Gemini, nu beneficiaza de
consolidare in acelasi proces.

Cand se construieste tracking/TTC (ByteTrack via model.track(), peste
ACELASI model YOLO deja incarcat pentru detectie -- vezi ARHITECTURA_SISTEM.md),
va trai in interiorul YoloDetectionRosNode, nu ca nod separat aici.
"""
import os

import cv2
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .model_paths import pin_current_process_to_cores
from .yolo_detection_node import YoloDetectionRosNode
from .yolo_segmentation_node import SegmentationRosNode

PERCEPTION_CORES_ENV = "KERYKE_PERCEPTION_CORES"
PERCEPTION_CORES_DEFAULT = "0,1"


def main(args=None):
    pin_current_process_to_cores(PERCEPTION_CORES_ENV, PERCEPTION_CORES_DEFAULT)
    cv2.setNumThreads(1)

    cores_str = os.environ.get(PERCEPTION_CORES_ENV, PERCEPTION_CORES_DEFAULT)
    n_cores = len([c for c in cores_str.split(",") if c.strip()])

    rclpy.init(args=args)
    nodes = [YoloDetectionRosNode(), SegmentationRosNode()]
    executor = MultiThreadedExecutor(num_threads=n_cores)
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
