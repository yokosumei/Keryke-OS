#!/usr/bin/env python3
"""
benchmark_depth_model.py -- cronometreaza inferenta reala a modelului
Depth Anything V2 Small (candidat pentru inlocuirea Metric3D pe Pi 4B,
vezi metric3d_depth_node.py -- 68-96s/cadru masurat real, mult peste
tinta de 400ms).

Prima rulare (518x518, cf. preprocessor_config.json oficial) a dat pe
Pi 4B ~7.1-7.6s -- 9-13x mai rapid ca Metric3D, dar tot 18x peste
tinta. Scalarea cu rezolutia e aproape patratica (masurat local: 645ms
la 518 -> 50ms la 154), deci testam mai multe rezolutii ca sa vedem
unde ajungem aproape de tinta REAL pe Pi, nu extrapolat.

ATENTIE: o rezolutie mai mica inseamna geometrie mai grosiera (podea/
obstacol) -- odata gasita o rezolutie rapida, trebuie verificat VIZUAL
(nu doar cronometrat) ca harta de adancime tot arata a ceva folositor,
nu doar zgomot rapid.

NU e inca legat de ROS2/nodul de productie -- doar masoara viteza bruta
de inferenta pe hardware-ul real (Pi), inainte sa decidem daca merita
integrarea.

RULARE (in containerul hive, unde exista onnxruntime):
    docker compose exec hive bash
    python3 /ws/scripts/benchmark_depth_model.py

Foloseste KERYKE_ONNX_THREADS (acelasi env var ca depth_node.py)
ca sa testezi cu acelasi echilibru de fire pe care il vei rula in productie.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "hive_perception" / "hive_perception"))
from model_paths import resolve_model_path  # noqa: E402

# cf. preprocessor_config.json din onnx-community/depth-anything-v2-small
# (verificat, nu presupus -- image_mean/image_std reale de pe HuggingFace).
# HW_CANDIDATES: 518 e marimea oficiala; restul sunt multipli de 14 mai
# mici, testati ca sa vedem unde scade sub tinta REAL pe Pi.
HW_CANDIDATES = [518, 336, 238, 154]
NORM_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
NORM_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
ENSURE_MULTIPLE_OF = 14

ONNX_THREADS = int(os.environ.get("KERYKE_ONNX_THREADS", "2"))
TARGET_MS = 400
N_RUNS = 3


def _round_to_multiple(value: int, multiple: int) -> int:
    return max(multiple, int(round(value / multiple)) * multiple)


def preprocess(rgb: np.ndarray, hw: int) -> np.ndarray:
    """Letterbox + normalizare, acelasi tipar ca preprocess_for_metric3d()."""
    import cv2

    target = _round_to_multiple(hw, ENSURE_MULTIPLE_OF)
    resized = cv2.resize(rgb, (target, target), interpolation=cv2.INTER_LINEAR)
    normalized = (resized.astype(np.float32) / 255.0 - NORM_MEAN) / NORM_STD
    chw = np.transpose(normalized, (2, 0, 1))
    return np.expand_dims(chw, axis=0).astype(np.float32)


def benchmark_at_resolution(sess, input_name: str, dummy_rgb: np.ndarray, hw: int) -> float:
    x = preprocess(dummy_rgb, hw)
    sess.run(None, {input_name: x})  # warmup -- exclude costul de optimizare a grafului

    times_ms = []
    for _ in range(N_RUNS):
        t0 = time.time()
        out = sess.run(None, {input_name: x})
        times_ms.append((time.time() - t0) * 1000)

    avg = sum(times_ms) / len(times_ms)
    verdict = "OK, sub tinta" if avg <= TARGET_MS else f"peste tinta ({avg / TARGET_MS:.1f}x)"
    print(f"  {hw}x{hw} -> output {out[0].shape}, timpi: "
          + ", ".join(f"{t:.0f}ms" for t in times_ms)
          + f" -- medie {avg:.0f}ms -- {verdict}")
    return avg


def benchmark_one(model_path: str) -> None:
    import onnxruntime as ort

    name = Path(model_path).name
    size_mb = Path(model_path).stat().st_size / 1e6
    print(f"\n=== {name} ({size_mb:.1f}MB, intra_op_num_threads={ONNX_THREADS}) ===")

    so = ort.SessionOptions()
    so.intra_op_num_threads = ONNX_THREADS
    sess = ort.InferenceSession(model_path, sess_options=so, providers=["CPUExecutionProvider"])
    input_name = sess.get_inputs()[0].name
    dummy_rgb = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    for hw in HW_CANDIDATES:
        benchmark_at_resolution(sess, input_name, dummy_rgb, hw)


def main() -> None:
    model_dir = resolve_model_path("depth_anything_v2_small")
    if not model_dir:
        print("Nu gasesc models/depth_anything_v2_small/ -- ai copiat folderul pe Pi?")
        sys.exit(1)

    # doar quantizat -- prima rulare a aratat ca fp32 nu e mai lent
    # semnificativ pe Cortex-A72 (fara accelerare int8 dedicata), deci
    # nu merita sa dublam timpul de test; quantizat e si mai mic de copiat.
    model_path = Path(model_dir) / "model_quantized.onnx"
    if not model_path.exists():
        print(f"Nu gasesc {model_path}")
        sys.exit(1)

    benchmark_one(str(model_path))


if __name__ == "__main__":
    main()
