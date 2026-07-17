#!/usr/bin/env python3
"""
calibrate_depth.py

Rutina de calibrare a scarei de disparitate pentru Depth Anything V2
Small, folosind ToF-ul (VL53L1X) de pe baston ca adevar de referinta
metric.

PROCEDURA (manuala, offline -- se ruleaza o data, sau ori de cate ori
schimbi camera/rezolutia/modul de captura/MODEL_INPUT_HW din depth_node.py):

1. Aliniaza bastonul astfel incat raza ToF sa cada pe axa optica a
   camerei (aprox. centrul cadrului) -- tine bastonul drept in fata
   pieptului, paralel cu directia de privire a camerei de pe vesta.
   ASTA e o conditie de calibrare deliberata, nu o presupunere de
   runtime: in mers normal, ToF-ul si camera NU sunt rigid aliniate
   (bastonul se misca independent), de-asta calibrarea se face static.

2. Pune un obiect plat/vizibil in fata, pe axa, la o distanta cunoscuta
   (masurata cu ToF-ul insusi sau cu o ruleta).

3. Ruleaza acest script cu argumentul de distanta -> captureaza un cadru,
   ruleaza inferenta, citeste disparitatea la pixelul de calibrare
   (implicit: centrul cadrului), salveaza esantionul.

4. Repeta la MINIM 2, ideal 3 distante diferite (ex: 1m, 2m, 3m).

5. Scriptul rezolva k_calibrated prin regresie liniara (in spatiul
   1/distanta <-> disparitate, vezi depth_calib_utils.py) si il salveaza
   in calibration.yaml -- fisier incarcat de depth_node.py.

Uzaj:
    python3 calibrate_depth.py sample --distance 1.0 --image frame_1m.png
    python3 calibrate_depth.py sample --distance 2.0 --image frame_2m.png
    python3 calibrate_depth.py sample --distance 3.0 --image frame_3m.png
    python3 calibrate_depth.py solve

(In productie, inlocuieste --image cu captura live de la camera si
--distance cu citirea live a ToF-ului -- structura de mai jos e gandita
sa fie usor de conectat la un topic ROS2 in loc de fisiere.)
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import yaml

try:
    from .depth_calib_utils import solve_disparity_scale_least_squares
except ImportError:
    from depth_calib_utils import solve_disparity_scale_least_squares

SAMPLES_FILE = "calibration_samples.json"
CALIBRATION_OUT = "calibration.yaml"

# Pixelul (x, y) unde presupunem ca a cazut raza ToF in cadrul aliniat.
# Implicit: centrul cadrului. Ajusteaza daca in setup-ul tau ToF-ul
# nu e exact pe axa optica a camerei chiar si in pozitia de calibrare.
DEFAULT_CALIB_PIXEL_FRACTION = (0.5, 0.5)  # (fractie_x, fractie_y) din (W, H)


def load_samples() -> list[dict]:
    if not os.path.exists(SAMPLES_FILE):
        return []
    with open(SAMPLES_FILE, "r") as f:
        return json.load(f)


def save_samples(samples: list[dict]) -> None:
    with open(SAMPLES_FILE, "w") as f:
        json.dump(samples, f, indent=2)


def sample_disparity_at_pixel(
    disparity_map: np.ndarray,
    pixel_fraction: tuple[float, float] = DEFAULT_CALIB_PIXEL_FRACTION,
    patch_radius: int = 3,
) -> float:
    """
    Esantioneaza disparitatea intr-un mic patch in jurul pixelului de
    calibrare (mediana, nu un singur pixel -- robustete la zgomot).
    """
    h, w = disparity_map.shape[:2]
    cx = int(pixel_fraction[0] * w)
    cy = int(pixel_fraction[1] * h)

    x0, x1 = max(0, cx - patch_radius), min(w, cx + patch_radius + 1)
    y0, y1 = max(0, cy - patch_radius), min(h, cy + patch_radius + 1)

    patch = disparity_map[y0:y1, x0:x1]
    valid = patch[patch > 0]
    if valid.size == 0:
        raise RuntimeError(
            f"Nicio valoare valida de disparitate in patch-ul de calibrare "
            f"({x0}:{x1}, {y0}:{y1}). Verifica alinierea si inferenta."
        )
    return float(np.median(valid))


def run_inference_disparity(image_path: str) -> np.ndarray:
    """
    Ruleaza acelasi preprocess + sesiune ONNX ca depth_node.py (fara sa
    porneasca nodul ROS2 -- doar functiile pure + sesiunea), pe o
    imagine capturata de pe disc. Intoarce harta de DISPARITATE bruta
    (nescalata metric inca -- scalarea metrica se face separat, in
    cmd_solve, cu k calibrat rezultat din acest script).
    """
    import cv2
    import onnxruntime as ort

    try:
        from .depth_node import (
            preprocess_depth_anything, postprocess_disparity, ONNX_MODEL_PATH,
            ONNX_INTRA_OP_THREADS,
        )
    except ImportError:
        from depth_node import (
            preprocess_depth_anything, postprocess_disparity, ONNX_MODEL_PATH,
            ONNX_INTRA_OP_THREADS,
        )

    bgr = cv2.imread(image_path)
    if bgr is None:
        raise FileNotFoundError(f"Nu pot citi imaginea {image_path}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = ONNX_INTRA_OP_THREADS
    session = ort.InferenceSession(ONNX_MODEL_PATH, sess_options=opts, providers=["CPUExecutionProvider"])
    nchw = preprocess_depth_anything(rgb)
    input_name = session.get_inputs()[0].name
    raw_output = session.run(None, {input_name: nchw})[0]

    return postprocess_disparity(raw_output, rgb.shape[:2])


def cmd_sample(args: argparse.Namespace) -> None:
    disparity_map = run_inference_disparity(args.image)
    disp = sample_disparity_at_pixel(disparity_map)

    samples = load_samples()
    samples.append({
        "tof_distance_m": args.distance,
        "disparity": disp,
        "image": args.image,
    })
    save_samples(samples)

    print(f"Esantion salvat: ToF={args.distance:.3f} m, disparitate={disp:.5f}")
    print(f"Total esantioane: {len(samples)}")


def cmd_solve(args: argparse.Namespace) -> None:
    samples = load_samples()
    if len(samples) < 2:
        print(f"EROARE: ai doar {len(samples)} esantioane, minim 2 necesare "
              f"(ideal 3, la distante diferite). Ruleaza --sample de mai multe ori.")
        sys.exit(1)

    tof_dists = [s["tof_distance_m"] for s in samples]
    disparities = [s["disparity"] for s in samples]

    k = solve_disparity_scale_least_squares(tof_dists, disparities)

    # Diagnostic: verifica daca eroarea e un factor de scara constant
    # (=> k e cauza) sau variaza cu distanta (=> domain shift/neliniaritate)
    predicted = [k / d for d in disparities]
    errors_pct = [
        100.0 * (p - t) / t for p, t in zip(predicted, tof_dists)
    ]

    print(f"\nScara de disparitate calibrata: k = {k:.5f}")
    print("\nDiagnostic per esantion:")
    for s, p, e in zip(samples, predicted, errors_pct):
        print(f"  ToF={s['tof_distance_m']:.2f}m  prezis={p:.3f}m  eroare={e:+.2f}%")

    spread = max(errors_pct) - min(errors_pct)
    if spread < 3.0:
        print(f"\nOK: spread de eroare intre esantioane = {spread:.2f}% (mic) "
              f"-> calibrarea e consistenta pe range-ul testat.")
    else:
        signs = [1 if e >= 0 else -1 for e in errors_pct]
        alternating = len(set(signs)) > 1
        if alternating:
            print(f"\nATENTIE: spread de eroare intre esantioane = {spread:.2f}% (mare), "
                  f"iar semnul erorii ALTERNEAZA intre esantioane (nu creste monoton "
                  f"intr-o singura directie) -> probabil zgomot de POZITIONARE/CENTRARE "
                  f"intre capturi, nu o limitare a modelului. Repeta cu MAI MULTE "
                  f"esantioane (5-6, nu doar 3) si un obiect fix, usor de centrat "
                  f"identic de fiecare data (nu o persoana, postura variaza).")
        else:
            print(f"\nATENTIE: spread de eroare intre esantioane = {spread:.2f}% (mare) "
                  f"-> eroarea NU e doar k gresit, probabil domain shift sau "
                  f"neliniaritate a retelei la anumite distante. Nu te baza orbeste pe k "
                  f"calibrat aici pentru tot range-ul -- vezi test_acceptance.py.")

    with open(CALIBRATION_OUT, "w") as f:
        yaml.safe_dump({
            "k_calibrated": k,
            "n_samples": len(samples),
            "calibration_pixel_fraction": list(DEFAULT_CALIB_PIXEL_FRACTION),
            "error_spread_pct": spread,
        }, f)
    print(f"\nSalvat in {CALIBRATION_OUT} -- incarca-l in depth_node.py")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_sample = sub.add_parser("sample")
    p_sample.add_argument("--distance", type=float, required=True,
                           help="Distanta ToF masurata, in metri")
    p_sample.add_argument("--image", type=str, required=True,
                           help="Cale catre cadrul capturat la acea distanta")
    p_sample.set_defaults(func=cmd_sample)

    p_solve = sub.add_parser("solve")
    p_solve.set_defaults(func=cmd_solve)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
