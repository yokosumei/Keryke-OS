#!/usr/bin/env python3
"""
depth_visualize.py -- ruleaza Depth Anything V2 Small (quantizat) pe un
cadru REAL (nu zgomot sintetic) la cateva rezolutii candidate, salveaza
harta de adancime colorata ca sa vedem VIZUAL daca geometria (podea vs
obstacol) tot e folositoare la rezolutii mici, nu doar rapida.

benchmark_depth_model.py a aratat pe Pi 4B: 154x154 -> 492ms (1.2x peste
tinta de 400ms), dar aia e doar viteza -- asta verifica daca imaginea
rezultata chiar arata a ceva util.

RULARE (in containerul hive):
    # intai, pe HOST (in afara containerului), prinde un cadru real:
    rpicam-still -o ~/THE-HIVE-Perception-Hub/scripts/test_frame.jpg --immediate

    docker compose exec hive bash
    python3 /ws/scripts/depth_visualize.py

Scrie test_frame_depth_<HW>.png langa test_frame.jpg (in scripts/, deci
vizibile si pe host dupa -- monteaza in container prin ../scripts:/ws/scripts).
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "hive_perception" / "hive_perception"))
from model_paths import resolve_model_path  # noqa: E402

NORM_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
NORM_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
ENSURE_MULTIPLE_OF = 14
ONNX_THREADS = int(os.environ.get("KERYKE_ONNX_THREADS", "2"))
HW_CANDIDATES = [238, 154]

FRAME_PATH = Path(__file__).resolve().parent / "test_frame.jpg"


def _round_to_multiple(value: int, multiple: int) -> int:
    return max(multiple, int(round(value / multiple)) * multiple)


def preprocess(rgb: np.ndarray, hw: int) -> np.ndarray:
    target = _round_to_multiple(hw, ENSURE_MULTIPLE_OF)
    resized = cv2.resize(rgb, (target, target), interpolation=cv2.INTER_LINEAR)
    normalized = (resized.astype(np.float32) / 255.0 - NORM_MEAN) / NORM_STD
    chw = np.transpose(normalized, (2, 0, 1))
    return np.expand_dims(chw, axis=0).astype(np.float32)


def colorize(depth: np.ndarray) -> np.ndarray:
    """Normalizeaza la 0-255 si aplica o paleta -- doar pt inspectie vizuala,
    nu e scalare metrica (aia vine dupa, cu focala calibrata)."""
    d = depth.astype(np.float32)
    d_norm = (d - d.min()) / (d.max() - d.min() + 1e-6)
    d_u8 = (d_norm * 255).astype(np.uint8)
    return cv2.applyColorMap(d_u8, cv2.COLORMAP_TURBO)


def main() -> None:
    if not FRAME_PATH.exists():
        print(f"Nu gasesc {FRAME_PATH}. Pe HOST (nu in container), prinde un cadru:\n"
              f"  rpicam-still -o {FRAME_PATH} --immediate")
        sys.exit(1)

    model_dir = resolve_model_path("depth_anything_v2_small")
    if not model_dir:
        print("Nu gasesc models/depth_anything_v2_small/")
        sys.exit(1)
    model_path = Path(model_dir) / "model_quantized.onnx"

    import onnxruntime as ort

    so = ort.SessionOptions()
    so.intra_op_num_threads = ONNX_THREADS
    sess = ort.InferenceSession(str(model_path), sess_options=so, providers=["CPUExecutionProvider"])
    input_name = sess.get_inputs()[0].name

    bgr = cv2.imread(str(FRAME_PATH))
    if bgr is None:
        print(f"Nu am putut citi {FRAME_PATH} (fisier corupt?)")
        sys.exit(1)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    out_path = FRAME_PATH.parent / "test_frame_depth_518_referinta.png"
    x = preprocess(rgb, 518)
    depth = sess.run(None, {input_name: x})[0].squeeze()
    cv2.imwrite(str(out_path), colorize(depth))
    print(f"518x518 (referinta, lenta dar precisa) -> {out_path}")

    for hw in HW_CANDIDATES:
        x = preprocess(rgb, hw)
        depth = sess.run(None, {input_name: x})[0].squeeze()
        out_path = FRAME_PATH.parent / f"test_frame_depth_{hw}.png"
        cv2.imwrite(str(out_path), colorize(depth))
        print(f"{hw}x{hw} -> {out_path}")

    print("\nCompara vizual cele 3 PNG-uri (referinta 518 vs 238 vs 154) --"
          " conteaza daca podeaua/obstacolele tot se disting clar, nu doar viteza.")


if __name__ == "__main__":
    main()
