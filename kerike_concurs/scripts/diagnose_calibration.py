#!/usr/bin/env python3
"""
diagnose_calibration.py -- ruleaza cadrele DEJA capturate pt calibrare
(frame_1m.jpg/frame_2m.jpg/frame_3m.jpg) prin model la mai multe
rezolutii, ca sa vedem daca disparitatea scade corect cu distanta la
vreo rezolutie mai mare -- fara sa mai recapturezi nimic.

Daca 154x154 da valori neconsistente dar o rezolutie mai mare da o
relatie corecta (disparitate scade monoton cu distanta), inseamna ca
154x154 e prea grosier pentru precizie NUMERICA (nu doar pentru
aspectul vizual, care a fost validat separat).

RULARE (in container, dupa ce ai deja frame_1m/2m/3m.jpg din scripts/):
    python3 /ws/scripts/diagnose_calibration.py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "hive_perception" / "hive_perception"))
from model_paths import resolve_model_path  # noqa: E402

NORM_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
NORM_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
ENSURE_MULTIPLE_OF = 14
ONNX_THREADS = int(os.environ.get("KERYKE_ONNX_THREADS", "2"))
HW_CANDIDATES = [154, 238, 336, 518]

FRAMES = [
    (1.0, Path(__file__).resolve().parent / "frame_1m.jpg"),
    (2.0, Path(__file__).resolve().parent / "frame_2m.jpg"),
    (3.0, Path(__file__).resolve().parent / "frame_3m.jpg"),
]


def _round_to_multiple(value: int, multiple: int) -> int:
    return max(multiple, int(round(value / multiple)) * multiple)


def preprocess(rgb: np.ndarray, hw: int) -> np.ndarray:
    target = _round_to_multiple(hw, ENSURE_MULTIPLE_OF)
    resized = cv2.resize(rgb, (target, target), interpolation=cv2.INTER_LINEAR)
    normalized = (resized.astype(np.float32) / 255.0 - NORM_MEAN) / NORM_STD
    chw = np.transpose(normalized, (2, 0, 1))
    return np.expand_dims(chw, axis=0).astype(np.float32)


def sample_center(disp_map: np.ndarray, patch_radius: int = 3) -> float:
    h, w = disp_map.shape[:2]
    cx, cy = w // 2, h // 2
    x0, x1 = max(0, cx - patch_radius), min(w, cx + patch_radius + 1)
    y0, y1 = max(0, cy - patch_radius), min(h, cy + patch_radius + 1)
    patch = disp_map[y0:y1, x0:x1]
    valid = patch[patch > 0]
    return float(np.median(valid)) if valid.size else float("nan")


def main() -> None:
    for _dist, path in FRAMES:
        if not path.exists():
            print(f"LIPSESTE: {path}")
            sys.exit(1)

    model_dir = resolve_model_path("depth_anything_v2_small")
    model_path = Path(model_dir) / "model_quantized.onnx"
    so = ort.SessionOptions()
    so.intra_op_num_threads = ONNX_THREADS
    sess = ort.InferenceSession(str(model_path), sess_options=so, providers=["CPUExecutionProvider"])
    input_name = sess.get_inputs()[0].name

    print(f"{'rezolutie':<10} {'1m':<10} {'2m':<10} {'3m':<10} {'monoton?'}")
    for hw in HW_CANDIDATES:
        disps = []
        for _dist, path in FRAMES:
            bgr = cv2.imread(str(path))
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            x = preprocess(rgb, hw)
            raw = sess.run(None, {input_name: x})[0].squeeze()
            disps.append(sample_center(raw))
        monotonic_decreasing = disps[0] > disps[1] > disps[2]
        print(f"{hw:<10} {disps[0]:<10.4f} {disps[1]:<10.4f} {disps[2]:<10.4f} "
              f"{'DA' if monotonic_decreasing else 'NU'}")


if __name__ == "__main__":
    main()
