"""
depth_calib_utils.py

Utilitare pentru transformul disparitate <-> metric al modelului de
adancime (Depth Anything V2 Small, ONNX) si pentru calibrarea scarei
fata de ToF-ul de pe baston.

CONVENTIE DE OUTPUT -- VERIFICATA, NU PRESUPUSA:
Depth Anything V2 (varianta relativa, necesara aici -- varianta cu cap
metric are export ONNX raportat defect, vezi issue #49 din repo-ul
oficial) e antrenata cu loss scale/shift-invariant in spatiul de
DISPARITATE (1/adancime), exact conventia MiDaS: valoare MARE = aproape
de camera, valoare MICA = departe. ASTA E INVERS fata de Metric3D
(canonical depth: valoare mare = departe) -- daca refolosesti formula
veche neschimbata, distantele ies inversate in tacere.

Model: disparitate_i ~= k_calibrat / distanta_reala_i
=> k_calibrat = distanta_reala_i * disparitate_i (constanta, daca
   disparitatea e cu-adevarat proportionala cu 1/distanta)

DOUA SURSE DE k:
1. Fara calibrare -- NU exista un echivalent geometric simplu (spre
   deosebire de focala, care putea fi estimata din HFOV) -- scara
   disparitatii e invatata de retea, arbitrara pana la calibrare.
2. k CALIBRAT -- rezultatul din calibrate_depth.py, care foloseste
   ToF-ul ca adevar de referinta. ASTA e valoarea care trebuie incarcata
   in nodul de productie (depth_node.py).

ATENTIE CRITICA:
- Calibrarea e valabila DOAR pentru rezolutia de input folosita la
  calibrare (154x154 in prezent) -- daca schimbi MODEL_INPUT_HW in
  depth_node.py, re-calibreaza. Spre deosebire de o focala geometrica,
  scara disparitatii NU se rescaleaza liniar si predictibil cu
  rezolutia (e invatata, nu geometrica) -- nu presupune, re-masoara.
"""

from __future__ import annotations

import numpy as np


def disparity_to_metric(
    disparity: np.ndarray,
    k_calibrated: float,
    clamp_min: float = 0.15,
    clamp_max: float = 10.0,
    eps: float = 1e-6,
) -> np.ndarray:
    """
    Converteste disparitatea bruta a modelului in adancime metrica
    reala (metri), folosind constanta k CALIBRATA (nu o valoare
    presupusa).

    distanta_reala = k_calibrat / disparitate

    clamp_min/clamp_max sunt setate implicit pentru regimul near-field
    relevant pentru Keryke (~0.15m - 10m). eps evita impartirea la zero
    la pixelii cu disparitate foarte mica (fundal/infinit).
    """
    d = disparity.astype(np.float32)
    depth_metric = k_calibrated / np.maximum(d, eps)
    return np.clip(depth_metric, clamp_min, clamp_max)


def solve_disparity_scale_least_squares(
    tof_distances_m: list[float],
    disparities_at_tof_pixel: list[float],
) -> float:
    """
    Rezolva k_calibrated printr-o regresie liniara PRIN ORIGINE, in
    spatiul (1/distanta, disparitate), folosind N perechi
    (distanta_ToF, disparitate_la_pixelul_ToF).

    Model: disparitate_i ~= k * (1/d_tof_i)
    Fie x_i = 1/d_tof_i, y_i = disparitate_i
    => k = sum(x_i * y_i) / sum(x_i^2)

    Foloseste MINIM 2 distante diferite (ideal 3+: 1m, 2m, 3m) ca sa poti
    verifica ulterior daca eroarea e un factor de scara constant (k
    gresit) sau creste neuniform cu distanta (domain shift / neliniaritate
    a retelei) -- vezi test_acceptance.py pentru diagnosticul asta.
    """
    if len(tof_distances_m) != len(disparities_at_tof_pixel):
        raise ValueError("listele trebuie sa aiba aceeasi lungime")
    if len(tof_distances_m) < 2:
        raise ValueError("minim 2 masuratori la distante diferite pentru o calibrare robusta")

    d_tof = np.array(tof_distances_m, dtype=np.float64)
    disp = np.array(disparities_at_tof_pixel, dtype=np.float64)

    if np.any(d_tof <= 0):
        raise ValueError("distantele ToF trebuie sa fie pozitive")
    if np.any(disp <= 0):
        raise ValueError("disparitatile trebuie sa fie pozitive (verifica pixelul esantionat)")

    x = 1.0 / d_tof
    denom = np.sum(x ** 2)
    if denom == 0:
        raise ValueError("denominator zero -- verifica esantioanele")

    k = np.sum(x * disp) / denom
    return float(k)
