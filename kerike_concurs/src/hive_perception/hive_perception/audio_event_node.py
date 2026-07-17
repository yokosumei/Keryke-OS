#!/usr/bin/env python3
"""
audio_event_node.py  --  Modelul 4: clasificare sunete de mediu (YAMNet)

Acopera unghiul mort de ~280 grade pe care camera frontala nu-l vede.
Foloseste YAMNet -- model PRE-ANTRENAT pe AudioSet (521 clase), zero
antrenare din partea ta. Tu doar alegi ce clase te intereseaza si le
mapezi pe alerte pentru RiskDescriptor.

INTRARE: audio 16kHz mono (fix rata pe care o cere YAMNet, si exact rata
la care decimezi deja microfonul AB13X de la 48kHz).

RULARE STANDALONE (test azi):
    # dintr-un fisier wav:
    python3 audio_event_node.py --wav test_claxon.wav
    # live din microfon (necesita sounddevice):
    python3 audio_event_node.py --live

RULARE CA NOD ROS2 (in containerul hive, pentru dashboard):
    ros2 run hive_perception audio_event_ros
    -> asculta microfonul continuu (sounddevice, prin ALSA/pulse
       montate in container), publica alertele peste prag ca JSON
       pe /audio/alerts (std_msgs/String).

INSTALARE pe Pi:
    pip install tflite-runtime soundfile numpy --break-system-packages
    # pentru --live: pip install sounddevice
    # model + clase (nu se descarca automat, vezi YamnetClassifier docstring):
    #   models/yamnet.tflite, models/yamnet_class_map.csv

NOTA: tensorflow complet (SavedModel + tensorflow-hub) a fost inlocuit cu
tflite-runtime -- pe Pi, alaturi de restul stack-ului, incarcarea unui
toolchain TF complet doar ca sa ruleze un clasificator mic ducea la swap
activ si load average masiv (masurat pe Pi 4B). Logica de mapare alerte
ramane identica, doar motorul de inferenta s-a schimbat.
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SAMPLE_RATE = 16000  # YAMNet cere fix 16kHz mono

# Numele microfonului cautat in `arecord -l` -- suprascrie cu
# KERYKE_MIC_NAME daca ai conectat alt microfon USB decat AB13X.
MIC_NAME = os.environ.get("KERYKE_MIC_NAME", "AB13X")

# Maparea claselor AudioSet (nume exact din yamnet_class_map.csv) pe
# alerte Keryke. Cheie = substring din numele clasei AudioSet; valoare =
# (tip_alerta, nivel). Un sunet care contine substring-ul declanseaza alerta.
# Am ales clasele relevante pentru un pieton nevazator in oras.
ALERT_MAP = {
    # vehicule care se apropie -- cel mai important, unghi mort
    "Vehicle": ("vehicul", "atentie"),
    "Car": ("masina", "atentie"),
    "Motorcycle": ("motocicleta", "atentie"),
    "Truck": ("camion", "atentie"),
    "Bus": ("autobuz", "atentie"),
    "Bicycle": ("bicicleta", "atentie"),
    # semnale de pericol -- prioritate maxima
    "Vehicle horn": ("claxon", "pericol"),
    "Car alarm": ("alarma", "pericol"),
    "Emergency vehicle": ("urgenta", "pericol"),
    "Siren": ("sirena", "pericol"),
    "Ambulance (siren)": ("ambulanta", "pericol"),
    "Police car (siren)": ("politie", "pericol"),
    "Reversing beeps": ("mers_inapoi", "pericol"),
    "Train horn": ("tren", "pericol"),
    # interactiune sociala -- ceva se adreseaza utilizatorului
    "Shout": ("strigat", "info"),
    "Yell": ("strigat", "info"),
    "Speech": ("voce", "info"),
    "Bicycle bell": ("sonerie_bicicleta", "atentie"),
}

# Prag minim de scor ca sa ridicam o alerta (YAMNet da scoruri 0..1).
SCORE_THRESHOLD = 0.30


class YamnetClassifier:
    """
    Incarca YAMNet ca model .tflite (tflite-runtime), NU ca SavedModel
    prin tensorflow-hub -- tensorflow complet e prea greu ca sa ruleze
    permanent alaturi de restul stack-ului pe Pi (testul real a dus la
    swap activ si load average 16.93/4 nuclee). Interpretorul
    TFLite incarca doar motorul de inferenta, nu tot toolchain-ul TF.

    Model + clase NU sunt generate/presupuse aici -- se pun manual in
    models/ (acelasi mecanism ca restul, vezi model_paths.py):
      - models/yamnet.tflite           (TF Hub / Kaggle: cauta "yamnet tflite")
      - models/yamnet_class_map.csv    (github.com/tensorflow/models, research/
                                         audioset/yamnet/yamnet_class_map.csv)
    """

    def __init__(self):
        self.interpreter = None
        self.class_names: list[str] = []
        self._input_details = None
        self._scores_output_index = None
        self._load()

    def _load(self):
        try:
            try:
                from tflite_runtime.interpreter import Interpreter
            except ImportError:
                # tflite-runtime are uneori wheel-uri lipsa pe aarch64 recent;
                # ai-edge-litert e succesorul oficial, acelasi API de Interpreter.
                from ai_edge_litert.interpreter import Interpreter
            import csv
            try:
                from .model_paths import resolve_model_path
            except ImportError:
                from model_paths import resolve_model_path

            model_path = resolve_model_path("yamnet.tflite")
            if not model_path:
                raise FileNotFoundError(
                    "yamnet.tflite nu e in models/ -- descarca-l (TF Hub/Kaggle, "
                    "cauta 'yamnet tflite')."
                )
            self.interpreter = Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()
            self._input_details = self.interpreter.get_input_details()[0]
            self._scores_output_index = self.interpreter.get_output_details()[0]["index"]

            class_map_path = resolve_model_path("yamnet_class_map.csv")
            if not class_map_path:
                raise FileNotFoundError(
                    "yamnet_class_map.csv nu e in models/ -- descarca-l de la "
                    "github.com/tensorflow/models (research/audioset/yamnet/)."
                )
            with open(class_map_path) as f:
                reader = csv.DictReader(f)
                self.class_names = [row["display_name"] for row in reader]
            print(f"[audio] YAMNet (tflite) incarcat din {model_path}, "
                  f"{len(self.class_names)} clase")
        except Exception as e:
            print(f"[audio] nu am putut incarca YAMNet ({e}). "
                  f"Verifica modelele in models/ si instalarea tflite-runtime.")
            self.interpreter = None

    def classify(self, waveform: np.ndarray) -> list[dict]:
        """
        waveform: np.float32 mono, 16kHz, in [-1, 1].
        Intoarce top-5 clase detectate: [{class_name, score}, ...]
        """
        if self.interpreter is None:
            return []
        wav = waveform.astype(np.float32)
        try:
            input_index = self._input_details["index"]
            if list(wav.shape) != list(self._input_details["shape"]):
                # fereastra de intrare are alta lungime decat cea alocata
                # curent -- redimensioneaza tensorul o data, nu la fiecare
                # apel (in bucla normala, ferestrele au aceeasi lungime).
                self.interpreter.resize_tensor_input(input_index, [len(wav)])
                self.interpreter.allocate_tensors()
                self._input_details = self.interpreter.get_input_details()[0]
            self.interpreter.set_tensor(input_index, wav)
            self.interpreter.invoke()
            scores = self.interpreter.get_tensor(self._scores_output_index)
        except Exception as e:
            print(f"[audio] eroare la inferenta YAMNet: {e}")
            return []
        mean_scores = np.mean(scores, axis=0) if scores.ndim > 1 else np.asarray(scores)
        top5_idx = np.argsort(mean_scores)[-5:][::-1]
        return [
            {"class_name": self.class_names[i], "score": float(mean_scores[i])}
            for i in top5_idx
        ]


def map_to_alerts(classifications: list[dict]) -> list[dict]:
    """
    Converteste clasele YAMNet in alerte Keryke, folosind ALERT_MAP.
    Intoarce lista de alerte peste prag: [{tip, nivel, sursa, scor}, ...]
    """
    alerts = []
    seen = set()
    for c in classifications:
        if c["score"] < SCORE_THRESHOLD:
            continue
        # o clasa YAMNet -> o SINGURA alerta: alegem cheia cea mai
        # specifica (substring cel mai lung care se potriveste), ca sa nu
        # dublam (ex: "Emergency vehicle" contine si "Vehicle", dar vrem
        # doar alerta "urgenta", nu si "vehicul").
        matches = [
            (key, tip, nivel) for key, (tip, nivel) in ALERT_MAP.items()
            if key.lower() in c["class_name"].lower()
        ]
        if not matches:
            continue
        key, tip, nivel = max(matches, key=lambda m: len(m[0]))
        if tip in seen:
            continue
        seen.add(tip)
        alerts.append({
            "tip": tip,
            "nivel": nivel,
            "sursa_clasa": c["class_name"],
            "scor": c["score"],
        })
    # sortare: pericol > atentie > info
    ordine = {"pericol": 0, "atentie": 1, "info": 2}
    alerts.sort(key=lambda a: (ordine.get(a["nivel"], 9), -a["scor"]))
    return alerts


def load_wav_16k(path: str) -> np.ndarray:
    """Incarca un wav, converteste la mono 16kHz float32 [-1,1]."""
    import soundfile as sf
    data, sr = sf.read(path, dtype="float32")
    if data.ndim > 1:
        data = np.mean(data, axis=1)  # mono
    if sr != SAMPLE_RATE:
        # resample simplu prin interpolare liniara (pt test; pe Pi
        # foloseste scipy.signal.resample sau decimarea ta de la 48k)
        n_target = int(len(data) * SAMPLE_RATE / sr)
        data = np.interp(
            np.linspace(0, len(data), n_target, endpoint=False),
            np.arange(len(data)), data
        ).astype(np.float32)
    return data


def process_audio(clf: YamnetClassifier, waveform: np.ndarray) -> dict:
    """Functia pura reutilizabila (test local SAU callback ROS2)."""
    classifications = clf.classify(waveform)
    alerts = map_to_alerts(classifications)
    return {"top5": classifications, "alerts": alerts}


try:
    from .audio_devices import find_arecord_device
except ImportError:
    from audio_devices import find_arecord_device


class AudioEventRosNode(Node):
    """
    Nod ROS2: capteaza microfonul in ferestre continue de 1s, ruleaza
    YAMNet si publica fiecare alerta peste prag (vezi map_to_alerts) ca
    JSON pe /audio/alerts, ca sa apara pe dashboard alaturi de detectie
    si adancime.

    Captura foloseste `arecord` CLI prin subprocess, NU sounddevice/
    PyAudio direct -- pe microfonul AB13X, PortAudio (baza ambelor
    biblioteci) raporteaza gresit maxInputChannels=0, ceea ce blocheaza
    deschiderea stream-ului. `arecord` merge direct pe ALSA si evita bug-ul.

    Device-ul e gasit dinamic prin find_arecord_device() (cauta MIC_NAME,
    implicit "AB13X", in `arecord -l`), nu hardcodat -- daca nu il
    gaseste, cade pe device-ul implicit ALSA si loghează un avertisment.
    """

    WINDOW_S = 1.0

    def __init__(self):
        super().__init__("audio_event_node")
        self.clf = YamnetClassifier()
        self.alert_pub = self.create_publisher(String, "/audio/alerts", 10)
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("AudioEventRosNode pornit, ascult microfonul live (arecord).")

    def _listen_loop(self):
        import subprocess
        duration_s = max(1, int(round(self.WINDOW_S)))

        device = find_arecord_device(MIC_NAME)
        if device:
            self.get_logger().info(f"arecord foloseste device explicit: {device}")
        else:
            self.get_logger().warn(
                f"Nu gasesc cardul '{MIC_NAME}' in `arecord -l` -- folosesc device-ul "
                "implicit ALSA, risc sa prinda alta placa audio daca exista mai multe."
            )

        cmd = ["arecord", "-q", "-f", "S16_LE", "-r", str(SAMPLE_RATE),
               "-c", "1", "-t", "raw", "-d", str(duration_s)]
        if device:
            cmd += ["-D", device]

        while rclpy.ok():
            try:
                raw = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
                pcm = np.frombuffer(raw, dtype=np.int16)
                waveform = pcm.astype(np.float32) / 32768.0
                result = process_audio(self.clf, waveform)
                for alert in result["alerts"]:
                    msg = String()
                    msg.data = json.dumps(alert)
                    self.alert_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Eroare captura audio (arecord): {e}")
                time.sleep(1.0)


def main_ros(args=None):
    """Punct de intrare ROS2 (`ros2 run hive_perception audio_event_ros`)."""
    rclpy.init(args=args)
    node = AudioEventRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--wav", help="fisier wav de clasificat")
    parser.add_argument("--live", action="store_true", help="microfon live")
    parser.add_argument("--seconds", type=float, default=1.0,
                        help="lungime fereastra live (s)")
    args = parser.parse_args()

    clf = YamnetClassifier()

    if args.wav:
        wav = load_wav_16k(args.wav)
        result = process_audio(clf, wav)
        print("\nTop 5 clase detectate:")
        for c in result["top5"]:
            print(f"  {c['class_name']:35s} {c['score']:.3f}")
        print("\nAlerte Keryke:")
        if result["alerts"]:
            for a in result["alerts"]:
                print(f"  [{a['nivel'].upper()}] {a['tip']} "
                      f"(din '{a['sursa_clasa']}', scor {a['scor']:.2f})")
        else:
            print("  (niciun sunet relevant peste prag)")

    elif args.live:
        import sounddevice as sd
        print(f"[audio] ascult live, ferestre de {args.seconds}s. Ctrl+C oprire.")
        try:
            while True:
                n = int(SAMPLE_RATE * args.seconds)
                rec = sd.rec(n, samplerate=SAMPLE_RATE, channels=1, dtype="float32")
                sd.wait()
                result = process_audio(clf, rec.flatten())
                if result["alerts"]:
                    top = result["alerts"][0]
                    print(f"[{top['nivel'].upper()}] {top['tip']} "
                          f"(scor {top['scor']:.2f})")
        except KeyboardInterrupt:
            print("\n[audio] oprit.")
    else:
        print("Foloseste --wav FISIER sau --live")


if __name__ == "__main__":
    main()
