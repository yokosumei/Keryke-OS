#!/usr/bin/env python3
"""
yolo_detection_node.py  --  Modelul 1: detectie obiecte + azimut

Ruleaza pe Raspberry Pi cu ultralytics. Pentru fiecare cadru:
  - detecteaza obiecte (YOLOv8n/YOLO11n)
  - filtreaza doar clasele relevante pentru un nevazator
  - calculeaza azimutul relativ al fiecarui obiect din centrul bbox
  - deseneaza overlay (bbox + label + azimut) pe feed
  - (optional) alege obiectul-tinta si scoate un unghi de servo

RULARE STANDALONE (azi, ca sa vezi ca merge):
    python3 yolo_detection_node.py
    -> deschide fereastra cu feed + overlay. 'q' inchide.

RULARE HEADLESS (Pi fara monitor, peste SSH -- vezi scripts/test_ai.sh):
    python3 yolo_detection_node.py --headless
    -> salveaza overlay-ul pe disc la /tmp/keryke_preview.jpg (~2Hz),
       vizibil din browser de pe laptop via http.server.

RULARE PE PI cu NCNN (mai rapid, cand ai exportat):
    yolo export model=yolov8n.pt format=ncnn imgsz=320
    python3 yolo_detection_node.py --model yolov8n_ncnn_model --imgsz 320

RULARE CA NOD ROS2 (in containerul hive, pentru dashboard):
    ros2 run hive_perception yolo_detection_ros
    -> consuma /perception/image_raw (publicat de imx500_bridge_node,
       care e doar sursa de imagine -- IMX500 nu ruleaza AI on-chip
       aici), publica Detection2DArray pe /perception/detections_yolo.

Structura e gandita sa se lege usor la ROS2: functia process_frame()
e pura (cadru -> detectii + overlay), o apelezi identic dintr-un
callback rclpy sau din bucla de mai jos.
"""

from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np

try:
    from .perception_geometry import azimuth_from_bbox_center, azimuth_to_servo_angle
except ImportError:
    from perception_geometry import azimuth_from_bbox_center, azimuth_to_servo_angle

try:
    from .model_paths import resolve_model_path
except ImportError:
    from model_paths import resolve_model_path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
from vision_msgs.msg import (BoundingBox2D, Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)

# Clase COCO relevante pentru un nevazator (nume -> pentru filtrare).
# YOLO da id-uri numerice; mapam prin numele din model.names.
RELEVANT_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "bus", "truck",
    "traffic light", "stop sign", "bench", "chair",
    "dog", "cat", "fire hydrant", "backpack", "suitcase",
}

# Nume model cautat prin resolve_model_path() -- implicit .pt (torch),
# suprascrie cu numele folderului NCNN exportat (vezi docstring de mai
# sus) dupa "yolo export ... format=ncnn" -- elimina PyTorch la
# inferenta, motor C++ mult mai usor si mai rapid pe ARM.
YOLO_MODEL_NAME = os.environ.get("KERYKE_YOLO_MODEL", "yolov8n.pt")

# 0.35 (implicit ultralytics) respinge detectii reale, intermitent, la
# yolov8n (cel mai mic/mai putin precis model din familie) in conditii
# variabile de lumina/unghi/ocluzie partiala -- confirmat separat cu
# diagnose_yolo.py (persoana detectata la 0.918 in conditii favorabile,
# dar increderea reala fluctueaza sub asta in conditii mai grele). 0.25
# e mai permisiv -- accepta mai multe detectii reale, cu riscul catorva
# fals-pozitive in plus (acceptabil pt un sistem de avertizare precoce,
# unde a rata un obstacol e mai grav decat o alerta in plus).
YOLO_CONF_THRESHOLD = float(os.environ.get("KERYKE_YOLO_CONF", "0.25"))


class YoloDetector:
    """Wrapper peste ultralytics, cu punct de substitutie izolat."""

    def __init__(self, model_path: str | None = None, imgsz: int = 320,
                 conf: float = 0.35):
        self.imgsz = imgsz
        self.conf = conf
        self.model = self._load(model_path or resolve_model_path("yolov8n.pt") or "yolov8n.pt")

    def _load(self, model_path: str):
        try:
            from ultralytics import YOLO
            model = YOLO(model_path)
            print(f"[detection] model incarcat: {model_path}")
            return model
        except Exception as e:
            print(f"[detection] AVERTISMENT: nu am putut incarca ultralytics/"
                  f"model ({e}). Nodul va rula fara detectii reale.")
            return None

    def infer(self, frame: np.ndarray) -> list[dict]:
        """
        Intoarce lista de detectii:
        [{label, conf, bbox=(x1,y1,x2,y2), azimuth_deg}, ...]
        filtrate pe clasele relevante.
        """
        if self.model is None:
            return []

        results = self.model(frame, imgsz=self.imgsz, conf=self.conf, verbose=False)
        frame_w = frame.shape[1]
        detections = []

        for r in results:
            names = r.names
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = names.get(cls_id, str(cls_id))
                if label not in RELEVANT_CLASSES:
                    continue
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                cx = (x1 + x2) / 2.0
                detections.append({
                    "label": label,
                    "conf": float(box.conf[0]),
                    "bbox": (x1, y1, x2, y2),
                    "azimuth_deg": azimuth_from_bbox_center(cx, frame_w),
                })
        return detections


def draw_overlay(frame: np.ndarray, detections: list[dict]) -> np.ndarray:
    """Deseneaza bbox + label + azimut. Nu modifica frame-ul original."""
    out = frame.copy()
    for d in detections:
        x1, y1, x2, y2 = (int(v) for v in d["bbox"])
        az = d["azimuth_deg"]
        side = "S" if az < -2 else ("D" if az > 2 else "centru")
        color = (0, 200, 0)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        text = f"{d['label']} {d['conf']:.2f} | {az:+.0f}gr {side}"
        cv2.putText(out, text, (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    # linia centrala (referinta 0 grade)
    h, w = out.shape[:2]
    cv2.line(out, (w // 2, 0), (w // 2, h), (120, 120, 120), 1)
    return out


def select_target(detections: list[dict], target_label: str | None) -> dict | None:
    """
    Alege obiectul-tinta. Daca target_label e dat, cel mai apropiat de
    centru din acea clasa; altfel, cel mai increzator obiect din cadru.
    (Distanta reala vine de la depth_node/ToF -- aici doar selectie.)
    """
    if not detections:
        return None
    if target_label:
        candidates = [d for d in detections if d["label"] == target_label]
        if not candidates:
            return None
        return min(candidates, key=lambda d: abs(d["azimuth_deg"]))
    return max(detections, key=lambda d: d["conf"])


def process_frame(detector: YoloDetector, frame: np.ndarray,
                  target_label: str | None = None) -> tuple[np.ndarray, dict]:
    """
    Functia pura reutilizabila (bucla locala SAU callback ROS2 o apeleaza
    identic). Intoarce (frame_cu_overlay, info_decizie).
    """
    detections = detector.infer(frame)
    target = select_target(detections, target_label)

    info = {"detections": detections, "target": target, "servo_angle": None}
    if target is not None:
        info["servo_angle"] = azimuth_to_servo_angle(target["azimuth_deg"])

    overlay = draw_overlay(frame, detections)
    if target is not None:
        x1, y1, x2, y2 = (int(v) for v in target["bbox"])
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 140, 255), 3)
        cv2.putText(overlay, f"TINTA -> servo {info['servo_angle']:.0f}gr",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2)
    return overlay, info


class YoloDetectionRosNode(Node):
    """
    Nod ROS2: consuma /perception/image_raw, ruleaza yolov8n.pt (din
    models/) si publica Detection2DArray pe /perception/detections_yolo.
    Nu deschide camera singur -- spre deosebire de main()/YoloDetector
    de mai sus, foloseste frame-urile deja publicate de imx500_bridge_node.
    """

    def __init__(self):
        super().__init__("yolo_detection_node")
        self.bridge = CvBridge()
        model_path = resolve_model_path(YOLO_MODEL_NAME) or YOLO_MODEL_NAME
        self.detector = YoloDetector(model_path=model_path, imgsz=320,
                                     conf=YOLO_CONF_THRESHOLD)
        self._frame_count = 0
        self._slow_frame_count = 0
        self.det_pub = self.create_publisher(
            Detection2DArray, "/perception/detections_yolo", 10)
        self.create_subscription(RosImage, "/perception/image_raw", self._on_image, 10)
        self.get_logger().info(
            f"YoloDetectionRosNode pornit (model {model_path}, "
            f"conf={YOLO_CONF_THRESHOLD}), publica pe /perception/detections_yolo.")

    def _on_image(self, msg: RosImage) -> None:
        t0 = time.monotonic()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = self.detector.infer(frame)

        elapsed_ms = (time.monotonic() - t0) * 1000.0
        self._frame_count += 1
        if elapsed_ms > 150.0:  # ~sub 7fps -- sub tinta de 10-12fps a proiectului
            self._slow_frame_count += 1
            if self._slow_frame_count % 10 == 1:  # nu spamam la fiecare cadru lent
                self.get_logger().warn(
                    f"Detectie YOLO a durat {elapsed_ms:.0f}ms (tinta ~100ms "
                    f"la 10fps) -- {self._slow_frame_count}/{self._frame_count} "
                    f"cadre lente pana acum. Daca apare des, verifica daca "
                    f"segmentarea concureaza pt aceleasi nuclee "
                    f"(KERYKE_PERCEPTION_CORES).")

        det_array = Detection2DArray()
        det_array.header = msg.header
        for d in detections:
            x1, y1, x2, y2 = d["bbox"]
            det = Detection2D()
            det.header = msg.header
            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            det.bbox = bbox
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d["label"]
            hyp.hypothesis.score = d["conf"]
            det.results.append(hyp)
            det_array.detections.append(det)
        self.det_pub.publish(det_array)


def main_ros(args=None):
    """Punct de intrare ROS2 (`ros2 run hive_perception yolo_detection_ros`)."""
    rclpy.init(args=args)
    node = YoloDetectionRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=None,
                         help="implicit: cauta yolov8n.pt in models/ (Docker/repo/env)")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--source", default="0", help="index camera sau cale video")
    parser.add_argument("--target", default=None, help="clasa-tinta (ex: person)")
    parser.add_argument("--headless", action="store_true",
                         help="fara fereastra grafica (Pi fara monitor, peste SSH) -- "
                              "salveaza overlay-ul periodic pe disc, la --preview-path")
    parser.add_argument("--preview-path", default="/tmp/keryke_preview.jpg",
                         help="unde salvez ultimul cadru cu overlay cand --headless")
    args = parser.parse_args()

    detector = YoloDetector(args.model, imgsz=args.imgsz)

    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"[detection] nu pot deschide sursa {args.source}")
        return

    if args.headless:
        print(f"[detection] rulez headless. Overlay salvat la {args.preview_path} "
              f"(~2Hz). Ctrl+C opreste.")
    else:
        print("[detection] rulez. Apasa 'q' in fereastra ca sa inchizi.")

    last_save = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            overlay, info = process_frame(detector, frame, args.target)
            if info["target"]:
                print(f"tinta={info['target']['label']} "
                      f"az={info['target']['azimuth_deg']:+.0f} "
                      f"servo={info['servo_angle']:.0f}")
            if args.headless:
                now = time.monotonic()
                if now - last_save > 0.5:
                    cv2.imwrite(args.preview_path, overlay)
                    last_save = now
            else:
                cv2.imshow("Keryke - YOLO detectie", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
