#!/usr/bin/env python3
"""
diagnose_yolo.py -- ruleaza modelul YOLO (NCNN, acelasi ca in productie)
pe o poza reala, FARA ROS2/dashboard, si arata TOATE detectiile brute
(orice clasa COCO, orice prag de incredere) -- ca sa vedem daca
modelul insusi detecteaza ceva, indiferent de filtrarea RELEVANT_CLASSES
sau de conf=0.35 din productie.

Daca aici nu apare NIMIC (nici macar la conf foarte mic), problema e
modelul/exportul NCNN, nu topicurile ROS2 sau desenarea din dashboard.

RULARE (in container, cu o poza deja capturata, ex. via grab_frame.py):
    python3 /ws/scripts/diagnose_yolo.py /ws/scripts/frame_1m.jpg
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "hive_perception" / "hive_perception"))
from model_paths import resolve_model_path  # noqa: E402


def main() -> None:
    if len(sys.argv) != 2:
        print("Uzaj: python3 diagnose_yolo.py <cale_imagine.jpg>")
        sys.exit(1)
    image_path = sys.argv[1]

    from ultralytics import YOLO

    model_path = resolve_model_path("yolov8n_ncnn_model") or "yolov8n_ncnn_model"
    print(f"Model: {model_path}")
    model = YOLO(model_path)

    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Nu pot citi {image_path}")
        sys.exit(1)
    print(f"Imagine: {image_path} ({frame.shape[1]}x{frame.shape[0]})")

    # conf foarte mic (0.01) si imgsz 320 (acelasi ca in productie) --
    # vrem sa vedem ORICE, nu doar ce trece de pragul de productie (0.35).
    results = model(frame, imgsz=320, conf=0.01, verbose=False)

    total = 0
    for r in results:
        names = r.names
        if r.boxes is None:
            continue
        for box in r.boxes:
            cls_id = int(box.cls[0])
            label = names.get(cls_id, str(cls_id))
            conf = float(box.conf[0])
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            print(f"  {label:<15} conf={conf:.3f}  bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})")
            total += 1

    print(f"\nTotal detectii brute (orice clasa, conf>=0.01): {total}")
    if total == 0:
        print("NIMIC detectat -- problema e modelul/exportul NCNN, nu ROS2/dashboard.")
    else:
        print("Modelul detecteaza ceva -- daca pe dashboard tot nu apare nimic, "
              "problema e in alta parte (topic/filtrare RELEVANT_CLASSES/desenare).")


if __name__ == "__main__":
    main()
