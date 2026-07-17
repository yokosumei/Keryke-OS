"""
perception_geometry.py

Logica pura de geometrie a perceptiei Keryke, izolata de ultralytics,
OpenCV video capture si ROS2. Contine:
  - azimut din bbox (pentru YOLO detectie)
  - azimut din masca de segmentare (pentru YOLO-seg drum)
  - conversia azimut -> unghi de servo

"Azimut" aici inseamna RELATIV la axa optica a camerei:
  0 grade   = drept in fata (centrul cadrului)
  negativ   = la STANGA ta
  pozitiv   = la DREAPTA ta
NU e cardinal (N/S/E/V) -- nu exista magnetometru in sistem.

FOV-ul e al camerei IMX500 (~78.3 grade orizontal pe senzorul complet).
ATENTIE: daca capturezi pe un mod cropped/binned, FOV-ul efectiv difera
-- vezi discutia despre focala. Ajusteaza HFOV_DEG la modul real.
"""

from __future__ import annotations

import numpy as np

HFOV_DEG = 78.3  # FOV orizontal camera IMX500 (verifica pt modul tau de captura)


def azimuth_from_bbox_center(
    bbox_center_x: float, frame_width: int, hfov_deg: float = HFOV_DEG
) -> float:
    """
    Azimut relativ (grade) al unui obiect din pozitia orizontala a
    centrului bbox-ului in cadru.

    azimut = ((cx / W) - 0.5) * HFOV
      cx = W/2 (centru)  -> 0 grade (drept in fata)
      cx = 0   (stanga)  -> -HFOV/2 (maxim stanga)
      cx = W   (dreapta) -> +HFOV/2 (maxim dreapta)
    """
    if frame_width <= 0:
        raise ValueError("frame_width trebuie sa fie pozitiv")
    normalized = (bbox_center_x / frame_width) - 0.5  # [-0.5, 0.5]
    return normalized * hfov_deg


def azimuth_from_mask_band(
    mask: np.ndarray,
    band_fraction: float = 0.35,
    hfov_deg: float = HFOV_DEG,
) -> float | None:
    """
    Azimut al DIRECTIEI drumului dintr-o masca de segmentare binara.

    Nu folosim centroidul intregii masti (ar da directia medie a tot ce
    e vizibil, inclusiv drum departe care coteste). Folosim doar o BANDA
    ORIZONTALA din partea de JOS a cadrului -- adica drumul IMEDIAT din
    fata utilizatorului -- si-i luam centroidul pe orizontala. Asta e
    "incotro merge drumul chiar acum sub picioarele mele".

    band_fraction = ce fractiune din inaltime, masurata de jos in sus,
    formeaza banda (0.35 = treimea de jos aprox).

    Intoarce None daca nu exista drum in banda (masca goala acolo) ->
    semnal ca ai pierdut drumul, de tratat in nodul de decizie.
    """
    h, w = mask.shape[:2]
    band_top = int(h * (1.0 - band_fraction))
    band = mask[band_top:h, :]

    ys, xs = np.nonzero(band)
    if xs.size == 0:
        return None  # niciun pixel de drum in banda din fata

    centroid_x = float(np.mean(xs))
    normalized = (centroid_x / w) - 0.5
    return normalized * hfov_deg


def azimuth_from_mask_centroid(
    mask: np.ndarray,
    hfov_deg: float = HFOV_DEG,
) -> float | None:
    """
    Azimut din centroidul intregii masti de segmentare (nu doar banda din
    fata folosita de azimuth_from_mask_band). FALLBACK pentru cazul in
    care banda din fata nu are niciun pixel walkable -- indica incotro e
    cea mai apropiata zona sigura vizibila in cadru, ca utilizatorul sa
    se poata reorienta, in loc sa primeasca doar "drum pierdut".

    Intoarce None daca masca e complet goala (nicio zona sigura vizibila
    nicaieri in cadru).
    """
    h, w = mask.shape[:2]
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None

    centroid_x = float(np.mean(xs))
    normalized = (centroid_x / w) - 0.5
    return normalized * hfov_deg


def azimuth_to_servo_angle(
    azimuth_deg: float,
    servo_center_deg: float = 90.0,
    servo_min_deg: float = 0.0,
    servo_max_deg: float = 180.0,
    invert: bool = False,
) -> float:
    """
    Mapeaza azimutul relativ (-HFOV/2 .. +HFOV/2) pe unghiul fizic al
    servoului SG90 (0..180, centru 90).

    servo = servo_center + azimut  (sau - azimut daca invert=True,
    in functie de orientarea fizica a servoului pe baston).

    Rezultatul e clampat in [servo_min, servo_max] ca sa nu ceri
    servoului un unghi imposibil.
    """
    delta = -azimuth_deg if invert else azimuth_deg
    angle = servo_center_deg + delta
    return float(np.clip(angle, servo_min_deg, servo_max_deg))
