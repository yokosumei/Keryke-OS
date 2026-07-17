#!/usr/bin/env python3
"""
decision_container.py -- spatial_risk, narrator si tts intr-un singur
proces (MultiThreadedExecutor), separat de perception_container.

Cele trei sunt Python pur, fara framework ML greu -- consolidarea aici
nu economiseste RAM/CPU semnificativ (spre deosebire de
perception_container, unde exista runtime-uri grele duplicate reale).
Motivul e mai putine procese de gestionat, nu resurse.

Ramane INTENTIONAT separat de perception_container: daca perceptia
pica (ex. segfault torch/onnxruntime sub presiune de memorie), acest
proces ramane in viata -- narator/tts tot pot vorbi (ex. raspunsuri
Gemini prin brain_node -> /audio/speak), utilizatorul nu ramane in
tacere completa doar fiindca un model greu a picat.
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .spatial_risk_node import SpatialRiskNode
from .narrator_node import NarratorNode
from .tts_node import TtsNode


def main(args=None):
    rclpy.init(args=args)
    nodes = [SpatialRiskNode(), NarratorNode(), TtsNode()]
    executor = MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
