#!/usr/bin/env python3
"""
audio_devices.py

Descoperire dinamica a device-urilor ALSA (captura SI redare) dupa NUME,
nu dupa index/numar de card. Indexul se schimba de fiecare data cand
plugi/scoti device-uri USB (dovedit repetat in teste: acelasi card a
aparut ca 2, apoi 1, apoi 3 in sesiuni diferite) -- hardcodarea unui
numar de card e o garantie ca se strica la urmatorul reboot/hotplug.

Folosit de audio_event_node.py (captura, YAMNet), brain_node.py (captura,
intrebari catre Gemini) si tts_node.py (redare, raspunsuri vorbite).
"""
from __future__ import annotations

import re
import subprocess


def _find_device(list_cmd: list[str], name_substr: str) -> str | None:
    try:
        out = subprocess.check_output(list_cmd, stderr=subprocess.DEVNULL, text=True)
    except Exception:
        return None

    for line in out.splitlines():
        m = re.match(
            r"card (\d+): .*\[.*" + re.escape(name_substr) + r".*\].*device (\d+):",
            line,
        )
        if m:
            card, dev = m.group(1), m.group(2)
            return f"plughw:CARD={card},DEV={dev}"
    return None


def find_arecord_device(name_substr: str) -> str | None:
    """Cauta in `arecord -l` (device-uri de CAPTURA) cardul al carui nume contine name_substr."""
    return _find_device(["arecord", "-l"], name_substr)


def find_aplay_device(name_substr: str) -> str | None:
    """Cauta in `aplay -l` (device-uri de REDARE) cardul al carui nume contine name_substr."""
    return _find_device(["aplay", "-l"], name_substr)
