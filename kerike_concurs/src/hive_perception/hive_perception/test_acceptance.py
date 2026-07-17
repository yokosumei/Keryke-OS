#!/usr/bin/env python3
"""
test_acceptance.py

Testul de acceptanta OBLIGATORIU inainte sa conectezi stratul de
adancime (depth_node.py, Depth Anything V2 Small) la costmap/
target_tracking. Verifica daca stratul metric calibrat e de incredere
la 1m, 2m, 3m (range-ul de interes pentru Keryke).

PROCEDURA:
1. Calibreaza intai scara de disparitate cu calibrate_depth.py (minim 2-3 distante).
2. Pune un obiect NOU (nefolosit la calibrare!) pe axa, la 1m, 2m, 3m
   (masurate independent -- ruleta sau ToF).
3. Ruleaza acest script cu cadrele + distantele ToF corespunzatoare.

CRITERIU DE ACCEPTARE: eroare < ~10% la toate trei distantele.

DIAGNOSTIC (asta e partea importanta, nu doar pass/fail):
- Daca eroarea e aproximativ ACELASI FACTOR la toate distantele
  (ex: +8%, +9%, +7%) -> mai ai o eroare reziduala de k. Solutie:
  re-calibreaza cu mai multe esantioane sau verifica modul de captura
  folosit la calibrare vs. la inferenta (trebuie sa fie identice).
- Daca eroarea CRESTE cu distanta (ex: +2%, +9%, +22%) -> nu mai e
  k-ul, e domain shift / neliniaritate a retelei la distanta. Solutie:
  nu extinde range-ul de incredere peste distanta la care eroarea
  devine inacceptabila -- limiteaza clamp_max in productie la acel prag.
- Daca eroarea e NECONSISTENTA (nu urmeaza niciun pattern clar) ->
  suspecteaza alinierea camera-ToF la momentul capturii (verifica daca
  bastonul chiar a fost pe axa optica in fiecare cadru) sau zgomot de
  senzor ToF la acea distanta.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import yaml

try:
    from .depth_calib_utils import disparity_to_metric
except ImportError:
    from depth_calib_utils import disparity_to_metric


def run_single_test(
    tof_distance_m: float,
    disparity_at_axis: float,
    k_calibrated: float,
) -> dict:
    """
    Aplica transformul disparitate->metric pe un singur esantion de
    referinta (pixelul de pe axa, la aceeasi pozitie folosita si la
    calibrare) si calculeaza eroarea fata de ToF.
    """
    predicted = disparity_to_metric(
        np.array([[disparity_at_axis]]),
        k_calibrated,
        clamp_min=0.0,
        clamp_max=100.0,
    )[0, 0]

    error_pct = 100.0 * (predicted - tof_distance_m) / tof_distance_m

    return {
        "tof_distance_m": tof_distance_m,
        "predicted_m": float(predicted),
        "error_pct": float(error_pct),
    }


def diagnose(results: list[dict]) -> str:
    errors = [r["error_pct"] for r in results]
    distances = [r["tof_distance_m"] for r in results]

    max_abs_error = max(abs(e) for e in errors)
    spread = max(errors) - min(errors)

    # verifica daca eroarea creste monoton cu distanta (semn de domain shift)
    sorted_pairs = sorted(zip(distances, errors))
    sorted_errors = [e for _, e in sorted_pairs]
    is_monotonic_increasing = all(
        sorted_errors[i] <= sorted_errors[i + 1] + 1.0  # toleranta 1%
        for i in range(len(sorted_errors) - 1)
    )

    lines = []
    if max_abs_error < 10.0:
        lines.append(f"ACCEPTAT: eroare maxima absoluta = {max_abs_error:.2f}% (< prag 10%).")
    else:
        lines.append(f"RESPINS: eroare maxima absoluta = {max_abs_error:.2f}% (>= prag 10%).")

    if spread < 3.0:
        lines.append(
            f"Spread mic intre distante ({spread:.2f}%) -> eroarea pare a fi un "
            f"FACTOR DE SCARA CONSTANT. Cauza probabila: k calibrat usor gresit. "
            f"Solutie: re-ruleaza calibrate_depth.py cu mai multe esantioane."
        )
    elif is_monotonic_increasing and spread >= 3.0:
        lines.append(
            f"Spread mare ({spread:.2f}%) si eroarea CRESTE cu distanta -> "
            f"nu mai e problema de k, e DOMAIN SHIFT / neliniaritate a "
            f"retelei la distante mai mari. Solutie: limiteaza clamp_max in "
            f"productie la distanta unde eroarea era inca acceptabila."
        )
    else:
        lines.append(
            f"Spread mare ({spread:.2f}%) FARA pattern monoton clar -> "
            f"suspecteaza alinierea camera-ToF in momentul capturii cadrelor "
            f"de test, sau zgomot de senzor. Repeta testul cu aliniere mai atenta."
        )

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", default="calibration.yaml")
    parser.add_argument(
        "--sample", nargs=2, action="append", metavar=("TOF_M", "DISPARITY"),
        required=True,
        help="Perechi (distanta_tof_m, disparitate_masurata) -- repeta "
             "flagul pentru fiecare din cele 1/2/3m. Disparitatea se "
             "obtine rulind inferenta si citind valoarea la pixelul de pe axa "
             "(acelasi folosit la calibrare).",
    )
    args = parser.parse_args()

    with open(args.calibration, "r") as f:
        calib = yaml.safe_load(f)
    k = float(calib["k_calibrated"])

    results = []
    for tof_str, disp_str in args.sample:
        r = run_single_test(float(tof_str), float(disp_str), k)
        results.append(r)
        print(f"ToF={r['tof_distance_m']:.2f}m  "
              f"prezis={r['predicted_m']:.3f}m  "
              f"eroare={r['error_pct']:+.2f}%")

    print()
    print(diagnose(results))

    max_abs_error = max(abs(r["error_pct"]) for r in results)
    sys.exit(0 if max_abs_error < 10.0 else 1)


if __name__ == "__main__":
    main()
