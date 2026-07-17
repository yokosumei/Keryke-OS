#!/usr/bin/env python3
"""
depth_node.py

Nod ROS2 care ruleaza Depth Anything V2 Small (ONNX, quantizat int8)
asincron, la o rata joasa, si publica harta de adancime METRICA (deja
scalata cu constanta k calibrata) pe un topic separat de restul
stack-ului care ruleaza la rata mare (YOLO ~10-12fps, spatial_risk_node
10Hz).

INLOCUIESTE Metric3D vit_small: masurat real pe Pi 4B, 68-96s/cadru la
616x1064 (170-240x peste tinta de 400ms) -- prea lent, indiferent de
KERYKE_ONNX_THREADS. Depth Anything V2 Small la 238x238 masoara 1194ms
real pe acelasi Pi 4B (~85x mai rapid ca Metric3D) -- vezi
scripts/benchmark_depth_model.py. 154x154 fusese ales initial (492ms,
mai rapid) si arata bine VIZUAL, dar la calibrare disparitatea nu
urmarea corect distanta reala (nemonoton) -- vezi
scripts/diagnose_calibration.py, care a aratat 238/336 monotone corect
si 154/518 nu. Aspectul vizual NU garanteaza precizie numerica.

DE CE ASINCRON, NU LA RATA DETECTORULUI:
Geometria statica a scenei (podea, pereti, obstacole fixe) se schimba
lent -- n-ai nevoie de o harta noua la fiecare cadru. YOLO ramane rapid
pentru reactie/azimut; acest nod da substratul geometric, esantionat la
bbox-urile YOLO.

PROCES SEPARAT, NU IN perception_container: rulat consolidat cu YOLO+
segmentare (MultiThreadedExecutor), timpul de inferenta ajungea la
1.6-3.6s (crescator), fata de 1194ms izolat -- Python GIL nu da
paralelism real intre threaduri CPU-bound in acelasi proces. Separat,
plus afinitate CPU explicita (KERYKE_DEPTH_CORES, implicit nucleul 2),
sistemul de operare ii da timp de CPU garantat, independent de YOLO/
segmentare.

CONVENTIE DE OUTPUT -- vezi depth_calib_utils.py: modelul da
DISPARITATE (valoare mare = aproape), nu adancime canonica proportionala
(cum dadea Metric3D) -- conversia in metri e disparity_to_metric(), nu
o simpla inmultire.

DEPENDENTE DE INSTALAT (pe Pi, in containerul hive):
    pip install onnxruntime opencv-python-headless pyyaml --break-system-packages
    (rclpy si cv_bridge vin din ROS2 Humble)

MODEL: models/depth_anything_v2_small/model_quantized.onnx (descarcat de
pe huggingface.co/onnx-community/depth-anything-v2-small, licenta
Apache 2.0) -- gitignored, copiaza manual (scp/rsync) pe fiecare masina.

CALIBRARE OBLIGATORIE INAINTE DE PRODUCTIE: ruleaza calibrate_depth.py
cu ToF-ul de pe baston ca adevar de referinta -- fara calibration.yaml,
nodul foloseste un k implicit NECALIBRAT si distantele sunt eronate.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from .depth_calib_utils import disparity_to_metric
except ImportError:
    from depth_calib_utils import disparity_to_metric

try:
    from .model_paths import resolve_model_path, pin_current_process_to_cores
except ImportError:
    from model_paths import resolve_model_path, pin_current_process_to_cores

# ---------------------------------------------------------------------------
# Config -- ancorate la directorul modulului, nu la CWD-ul procesului
# (ros2 run porneste nodul din orice director curent, nu neaparat de aici)
# ---------------------------------------------------------------------------
_THIS_DIR = Path(__file__).resolve().parent
_MODEL_DIR = resolve_model_path("depth_anything_v2_small")
ONNX_MODEL_PATH = (
    str(Path(_MODEL_DIR) / "model_quantized.onnx") if _MODEL_DIR
    else str(_THIS_DIR / "depth_anything_v2_small" / "model_quantized.onnx")
)
CALIBRATION_PATH = str(_THIS_DIR / "calibration.yaml")

# Implicit, onnxruntime insfaca toate nucleele disponibile pentru o
# singura inferenta -- bun cand ruleaza singur, dezastru cand concureaza
# cu alte noduri grele pe Pi (amplifica thrashing-ul, nu doar CPU brut).
# Implicit 1 -- procesul e restrictionat la 1 nucleu (KERYKE_DEPTH_CORES,
# vezi main()), 2 threaduri pe 1 nucleu doar adauga context-switching
# fara castig. Suprascrie cu KERYKE_ONNX_THREADS daca aloci mai multe
# nuclee prin KERYKE_DEPTH_CORES.
ONNX_INTRA_OP_THREADS = int(os.environ.get("KERYKE_ONNX_THREADS", "1"))

# 238x238 -- 154x154 arata bine VIZUAL dar disparitatea nu urmarea corect
# distanta reala la calibrare (neconsistent, uneori crescator cu distanta
# in loc sa scada) -- vezi scripts/diagnose_calibration.py, care ruleaza
# acelasi cadru la mai multe rezolutii: 154 nemonoton, 238 si 336 monotone
# corect. 238 ales fata de 336 pt viteza (1194ms vs 2626ms masurat real pe
# Pi 4B, ambele corecte numeric). NU rescala fara sa re-rulezi
# diagnoze_calibration.py -- precizia numerica NU se poate presupune doar
# din aspectul vizual.
MODEL_INPUT_HW = 238
# cf. preprocessor_config.json din onnx-community/depth-anything-v2-small
# (verificat, nu presupus, image_mean/image_std reale)
NORM_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
NORM_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

# 1.5s -- peste cele 1194ms masurate real pe Pi 4B la 238x238 (izolat;
# concurent cu YOLO+segmentare in perception_container e mai mult), ca sa
# nu spamam avertismentul de mai jos la fiecare cadru; ajusteaza dupa FPS real.
INFERENCE_PERIOD_S = 1.5

DEPTH_CLAMP_MIN_M = 0.15
DEPTH_CLAMP_MAX_M = 10.0


def preprocess_depth_anything(
    rgb: np.ndarray, hw: int = MODEL_INPUT_HW
) -> np.ndarray:
    """
    Resize direct la (hw, hw) + normalizare DPT. Fara letterbox/padding
    -- verificat vizual (scripts/depth_visualize.py) ca geometria
    (podea/obstacole) ramane distinsa la resize direct, patratic.
    """
    resized = cv2.resize(rgb, (hw, hw), interpolation=cv2.INTER_LINEAR)
    normalized = (resized.astype(np.float32) / 255.0 - NORM_MEAN) / NORM_STD
    chw = np.transpose(normalized, (2, 0, 1))
    return np.expand_dims(chw, axis=0).astype(np.float32)


def postprocess_disparity(
    raw_output: np.ndarray, original_hw: tuple[int, int]
) -> np.ndarray:
    """Upsample disparitatea bruta la dimensiunea originala a cadrului."""
    disparity = raw_output.squeeze()
    orig_h, orig_w = original_hw
    return cv2.resize(disparity, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)


class DepthNode(Node):
    def __init__(self):
        super().__init__("depth_node")

        self.bridge = CvBridge()
        self.k_calibrated = self._load_calibration(CALIBRATION_PATH)

        self.session = self._load_onnx_session(ONNX_MODEL_PATH)

        self._latest_frame: Optional[np.ndarray] = None
        self._last_inference_time = 0.0

        # Acelasi topic publicat de imx500_bridge_node (consumat si de brain_node).
        self.image_sub = self.create_subscription(
            Image, "/perception/image_raw", self._on_image, 1
        )
        self.depth_pub = self.create_publisher(
            Image, "/keryke/depth/metric", 1
        )

        self.timer = self.create_timer(INFERENCE_PERIOD_S, self._on_timer)

        self.get_logger().info(
            f"DepthNode pornit. k_calibrated={self.k_calibrated:.4f}, "
            f"rata inferenta={1.0/INFERENCE_PERIOD_S:.1f} Hz (asincron fata de detector)."
        )

    def _load_calibration(self, path: str) -> float:
        try:
            with open(path, "r") as f:
                calib = yaml.safe_load(f)
            k = float(calib["k_calibrated"])
            self.get_logger().info(f"Calibrare incarcata din {path}: k={k:.4f}")
            return k
        except FileNotFoundError:
            self.get_logger().warn(
                f"{path} nu exista -- NU e calibrat! Ruleaza calibrate_depth.py "
                f"inainte de productie. Folosesc temporar k=1.0, care e aproape "
                f"sigur GRESIT si va da distante metrice eronate in tacere."
            )
            return 1.0

    def _load_onnx_session(self, path: str):
        import onnxruntime as ort
        try:
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = ONNX_INTRA_OP_THREADS
            session = ort.InferenceSession(
                path, sess_options=opts, providers=["CPUExecutionProvider"])
            self.get_logger().info(
                f"Sesiune ONNX incarcata din {path} "
                f"(intra_op_num_threads={ONNX_INTRA_OP_THREADS})")
            return session
        except Exception as e:
            self.get_logger().error(
                f"Nu am putut incarca modelul ONNX de la {path}: {e}. "
                f"Verifica ca ai copiat models/depth_anything_v2_small/ pe masina asta."
            )
            raise

    def _on_image(self, msg: Image) -> None:
        # Doar tine cel mai recent cadru -- inferenta ruleaza pe timer,
        # nu pe fiecare mesaj (asincron fata de rata camerei/detectorului).
        self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def _on_timer(self) -> None:
        if self._latest_frame is None:
            return

        frame = self._latest_frame
        t0 = time.monotonic()

        nchw = preprocess_depth_anything(frame)

        input_name = self.session.get_inputs()[0].name
        raw_output = self.session.run(None, {input_name: nchw})[0]

        disparity = postprocess_disparity(raw_output, frame.shape[:2])
        depth_metric = disparity_to_metric(
            disparity,
            self.k_calibrated,
            clamp_min=DEPTH_CLAMP_MIN_M,
            clamp_max=DEPTH_CLAMP_MAX_M,
        )

        elapsed_ms = (time.monotonic() - t0) * 1000.0
        if elapsed_ms > INFERENCE_PERIOD_S * 1000.0:
            self.get_logger().warn(
                f"Inferenta a durat {elapsed_ms:.0f}ms, mai mult decat "
                f"perioada tinta {INFERENCE_PERIOD_S*1000:.0f}ms -- "
                f"scade rata (INFERENCE_PERIOD_S) sau reduce MODEL_INPUT_HW."
            )

        out_msg = self.bridge.cv2_to_imgmsg(depth_metric.astype(np.float32), encoding="32FC1")
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_pub.publish(out_msg)


def sample_bbox_distance(
    depth_metric_map: np.ndarray, bbox_xyxy: tuple[int, int, int, int]
) -> float:
    """
    Utilitar de integrare pentru target_tracking_node: distanta la un
    obiect detectat de YOLO = mediana adancimii metrice in interiorul
    bbox-ului (mediana, nu media -- robust la margini/fundal care intra
    in bbox).

    NOTA: harta de adancime poate fi usor "invechita" fata de bbox-ul
    curent (acest nod ruleaza ~1.7Hz, YOLO la 10-12Hz) -- acceptabil
    pentru geometrie statica, dar nu folosi asta pentru obiecte care se
    misca rapid; pentru alea, bazeaza-te pe estimarea din marimea bbox
    (pinhole + prior de clasa) care e sincrona cu detectia.
    """
    x0, y0, x1, y1 = bbox_xyxy
    h, w = depth_metric_map.shape[:2]
    x0, x1 = max(0, x0), min(w, x1)
    y0, y1 = max(0, y0), min(h, y1)

    region = depth_metric_map[y0:y1, x0:x1]
    valid = region[(region > DEPTH_CLAMP_MIN_M) & (region < DEPTH_CLAMP_MAX_M)]
    if valid.size == 0:
        raise ValueError("niciun pixel valid de adancime in bbox")
    return float(np.median(valid))


def main(args=None):
    pin_current_process_to_cores("KERYKE_DEPTH_CORES", "2")

    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
