#!/usr/bin/env python3
"""
model_paths.py

Rezolva calea catre fisierele/folderele de model (YOLO .pt/NCNN, Depth
Anything .onnx, YAMNet tflite) indiferent daca nodul ruleaza in containerul
Docker `hive`, direct din checkout-ul de sursa, sau la teste locale.

ORDINE DE CAUTARE (primul care exista castiga):
1. $KERYKE_MODELS_DIR/<name>  -- override explicit
2. /ws/models/<name>          -- mount-ul Docker (vezi docker-compose.yml,
                                  volumes: ../models:/ws/models)
3. models/ gasit urcand din acest fisier -- checkout local fara Docker

Daca nu gaseste nimic, intoarce None si caller-ul decide fallback-ul
(ex: numele gol lasa ultralytics sa downloadeze automat din internet).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

_DOCKER_MODELS_DIR = Path("/ws/models")
_THIS_DIR = Path(__file__).resolve().parent


def _find_repo_models_dir(start: Path, max_up: int = 6) -> Optional[Path]:
    current = start
    for _ in range(max_up):
        candidate = current / "models"
        if candidate.is_dir():
            return candidate
        if current.parent == current:
            break
        current = current.parent
    return None


def resolve_model_path(name: str) -> Optional[str]:
    """Cauta `name` (fisier sau folder) in models/; None daca nu-l gaseste."""
    candidates = []
    env_dir = os.environ.get("KERYKE_MODELS_DIR")
    if env_dir:
        candidates.append(Path(env_dir) / name)
    candidates.append(_DOCKER_MODELS_DIR / name)
    repo_models = _find_repo_models_dir(_THIS_DIR)
    if repo_models:
        candidates.append(repo_models / name)

    for path in candidates:
        if path.exists():
            return str(path)
    return None


def pin_current_process_to_cores(env_var: str, default_cores: str) -> None:
    """
    Restrictioneaza procesul curent la un set fix de nuclee CPU (afinitate).
    O limita de threaduri (ex. onnxruntime intra_op_num_threads, cv2.setNumThreads)
    e doar o sugestie catre biblioteca -- fiecare biblioteca CPU-bound tot
    "crede" ca are toate nucleele placii, ceea ce cauzeaza suprasolicitare
    reala cand mai multe procese grele ruleaza simultan pe Pi 4B (masurat:
    depth_node 1.6s->3.6s in perception_container, in loc de 1194ms izolat).
    Afinitatea e impusa de kernel -- procesul FIZIC nu poate rula pe alt
    nucleu, indiferent ce crede biblioteca din interior.

    env_var: ex. "KERYKE_PERCEPTION_CORES" -- override, string "0,1"
    default_cores: folosit daca env_var nu e setat, acelasi format
    """
    if not hasattr(os, "sched_setaffinity"):
        return  # nu exista pe macOS/Windows -- no-op in loc sa crape
    cores_str = os.environ.get(env_var, default_cores)
    try:
        cores = {int(c) for c in cores_str.split(",") if c.strip()}
        os.sched_setaffinity(0, cores)
        print(f"[affinity] proces PID {os.getpid()} restrictionat la nucleele "
              f"{sorted(cores)} ({env_var}={cores_str})")
    except (OSError, ValueError) as e:
        print(f"[affinity] AVERTISMENT: nu am putut seta afinitatea la "
              f"{cores_str} ({env_var}): {e}")
