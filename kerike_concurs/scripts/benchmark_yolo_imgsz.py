#!/usr/bin/env python3
"""
benchmark_yolo_imgsz.py -- masoara timpul real de inferenta NCNN (YOLO)
la mai multe rezolutii de input, pe o poza reala deja capturata.

Motiv: camera captureaza la 640x480 (vezi host/camera_publisher.py), dar
productia ruleaza inferenta la imgsz=320 -- la 2-2.5m o persoana ocupa
deja putini pixeli in cadrul original; downscale-ul suplimentar la 320
ii mai reduce detaliul, plauzibil sub ce are nevoie modelul nano ca sa
detecteze cu incredere. Un imgsz mai mare (mai aproape de 640 nativ) ar
trebui sa ajute la distanta, dar costa timp de inferenta -- masuram
real inainte sa decidem compromisul, nu ghicim.

NOTA: modelul NCNN a fost exportat cu imgsz=320. Daca acest script
esueaza/da rezultate ciudate la un imgsz mai mare, inseamna ca trebuie
re-exportat la acel imgsz (yolo export model=yolov8n.pt format=ncnn
imgsz=<nou>), nu doar schimbat parametrul in cod -- exact ce vrem sa
aflam rulandu-l.

RULARE (in container, cu o poza capturata la 2-2.5m, ex. via grab_frame.py):
    python3 /ws/scripts/benchmark_yolo_imgsz.py /ws/scripts/frame_2m.jpg
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "hive_perception" / "hive_perception"))
from model_paths import resolve_model_path  # noqa: E402

IMGSZ_CANDIDATES = [320, 416, 480, 640]
N_RUNS = 3


def main() -> None:
    if len(sys.argv) != 2:
        print("Uzaj: python3 benchmark_yolo_imgsz.py <cale_imagine.jpg>")
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
    print(f"Imagine: {image_path} ({frame.shape[1]}x{frame.shape[0]})\n")

    print(f"{'imgsz':<8} {'timp mediu (ms)':<18} {'persoana detectata?':<20} {'conf max'}")
    for imgsz in IMGSZ_CANDIDATES:
        try:
            model(frame, imgsz=imgsz, conf=0.01, verbose=False)  # warmup
        except Exception as e:
            print(f"{imgsz:<8} EROARE la acest imgsz -- probabil are nevoie de "
                  f"re-export NCNN la {imgsz} ({e})")
            continue

        times = []
        best_conf = 0.0
        found_person = False
        for _ in range(N_RUNS):
            t0 = time.monotonic()
            results = model(frame, imgsz=imgsz, conf=0.01, verbose=False)
            times.append((time.monotonic() - t0) * 1000.0)
            for r in results:
                names = r.names
                if r.boxes is None:
                    continue
                for box in r.boxes:
                    label = names.get(int(box.cls[0]), "")
                    conf = float(box.conf[0])
                    if label == "person" and conf > best_conf:
                        best_conf = conf
                        found_person = True

        avg_ms = sum(times) / len(times)
        print(f"{imgsz:<8} {avg_ms:<18.1f} {'DA' if found_person else 'NU':<20} {best_conf:.3f}")


if __name__ == "__main__":
    main()
