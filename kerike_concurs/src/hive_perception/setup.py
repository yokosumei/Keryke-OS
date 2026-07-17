import os
from glob import glob

from setuptools import setup

package_name = "hive_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yoko",
    maintainer_email="daria.antonia.stan@gmail.com",
    description="HIVE perception nodes for Keryke-OS smart cane",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imx500_bridge = hive_perception.imx500_bridge_node:main",
            "perception_container = hive_perception.perception_container:main",
            "decision_container = hive_perception.decision_container:main",
            "ble_bridge = hive_perception.ble_bridge_node:main",
            "haptic_vest = hive_perception.haptic_vest_node:main",
            "dashboard = hive_perception.dashboard_node:main",
            "narrator = hive_perception.narrator_node:main",
            "tts = hive_perception.tts_node:main",
            "wake = hive_perception.wake_node:main",
            "brain = hive_perception.brain_node:main",
            "spatial_risk  = hive_perception.spatial_risk_node:main",
            "yolo_detection = hive_perception.yolo_detection_node:main",
            "yolo_detection_ros = hive_perception.yolo_detection_node:main_ros",
            "yolo_segmentation = hive_perception.yolo_segmentation_node:main",
            "yolo_segmentation_ros = hive_perception.yolo_segmentation_node:main_ros",
            "audio_event = hive_perception.audio_event_node:main",
            "audio_event_ros = hive_perception.audio_event_node:main_ros",
            "depth_node = hive_perception.depth_node:main",
            "calibrate_depth = hive_perception.calibrate_depth:main",
            "test_acceptance = hive_perception.test_acceptance:main",
        ],
    },
)
