FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Pachete sistem
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-sensor-msgs \
    libgl1 \
    libglib2.0-0 \
    portaudio19-dev \
    libasound2-dev \
    libsndfile1 \
    alsa-utils \
    ffmpeg \
    espeak-ng \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# Python deps
# ncnn e listat explicit -- fara el, ultralytics il auto-instaleaza la
# RUNTIME la prima inferenta NCNN (yolo_detection_node.py cu
# KERYKE_YOLO_MODEL=yolov8n_ncnn_model), in mijlocul procesului deja
# pornit. Un pachet C-extension instalat asa nu e garantat utilizabil
# fara restart de interpretor -- exact ce a produs detectii oribile la
# prima rulare (auto-install + "Restart runtime ... to take effect").
RUN pip3 install --no-cache-dir \
    "numpy<2" \
    flask \
    opencv-python \
    requests \
    sounddevice \
    pyaudio \
    openwakeword \
    google-generativeai \
    ultralytics \
    ncnn \
    onnxruntime \
    pyyaml \
    soundfile \
    tflite-runtime \
    bleak \
    adafruit-blinka \
    adafruit-circuitpython-pca9685

# Descarcă modelele OpenWakeWord (hey_jarvis + dependencies)
RUN python3 -c "import openwakeword.utils; openwakeword.utils.download_models()"

# Fix pt colcon/ament_python: unul din pachetele de mai sus a tras dupa el
# un setuptools prea nou, care cheama canonicalize_version(strip_trailing_zero=...)
# -- argument pe care packaging-ul din imagine nu-l stie, si orice
# `colcon build` pica cu TypeError. Pinat ULTIMUL, ca sa nu fie reupgradat.
RUN pip3 install --no-cache-dir "setuptools==65.5.1"

WORKDIR /ws

# Auto-source ROS la fiecare bash interactiv (backup -- vezi si entrypoint.sh)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "source /ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

# entrypoint.sh sursa ROS + workspace-ul inainte de orice comanda -- sursa
# de adevar (nu doar .bashrc, care nu prinde toate sesiunile de shell).
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
