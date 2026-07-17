#!/usr/bin/env bash
# test_ai.sh -- testeaza pe rand nodurile AI standalone (yolo_detection,
# yolo_segmentation, audio_event) direct pe camera/microfonul Pi-ului,
# FARA Docker/ROS2 (nodurile astea nu sunt inca legate pe topicuri ROS,
# isi deschid singure camera -- vezi docstring-urile fiecarui fisier).
#
# Pi-ul e headless (SSH, fara monitor) -- optiunile video salveaza
# overlay-ul pe disc la /tmp/keryke_preview.jpg si pornesc un
# http.server ca sa-l vezi din browser de pe laptop, la:
#   http://<ip-pi>:8000/keryke_preview.jpg
#
# depth_node NU e aici -- e nod ROS2 adevarat, are nevoie de
# host/camera_publisher.py (pe host) + `ros2 run hive_perception
# imx500_bridge` (in container) ca sa aiba ce sa consume de pe
# /perception/image_raw.

set -e

NODE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../src/hive_perception/hive_perception" && pwd)"
PREVIEW_PATH="/tmp/keryke_preview.jpg"
HTTP_PORT=8000

HTTP_PID=""
cleanup() {
    if [ -n "$HTTP_PID" ]; then
        kill "$HTTP_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

start_preview_server() {
    local ip
    ip=$(hostname -I | awk '{print $1}')
    ( cd /tmp && python3 -m http.server "$HTTP_PORT" >/tmp/keryke_http.log 2>&1 ) &
    HTTP_PID=$!
    echo ""
    echo "Preview live la: http://${ip}:${HTTP_PORT}/keryke_preview.jpg"
    echo "(refresh manual in browser -- nu e un stream, e un JPEG suprascris ~2Hz)"
    echo ""
}

echo "Ce testam?"
echo "  1) YOLO detectie obiecte (models/yolov8n.pt)"
echo "  2) Segmentare drum -- mod HSV (prosop albastru, fara model)"
echo "  3) Segmentare drum -- mod YOLO (necesita yolov8n-seg.pt antrenat de tine in models/)"
echo "  4) YAMNet sunete de mediu, live din microfon (fara preview video)"
read -rp "Alege [1-4]: " choice

cd "$NODE_DIR"

case "$choice" in
    1)
        start_preview_server
        python3 yolo_detection_node.py --headless --preview-path "$PREVIEW_PATH"
        ;;
    2)
        start_preview_server
        python3 yolo_segmentation_node.py --mode hsv --headless --preview-path "$PREVIEW_PATH"
        ;;
    3)
        start_preview_server
        python3 yolo_segmentation_node.py --mode yolo --headless --preview-path "$PREVIEW_PATH"
        ;;
    4)
        python3 audio_event_node.py --live
        ;;
    *)
        echo "optiune invalida"
        exit 1
        ;;
esac
