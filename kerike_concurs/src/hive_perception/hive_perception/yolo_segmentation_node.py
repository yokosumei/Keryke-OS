#!/usr/bin/env python3
"""
yolo_segmentation_node.py  --  Modelul 2: segmentare drum + azimut

Doua moduri de obtinere a mastii de drum, alese cu --mode:

  --mode hsv   (DEMO, implicit): prag de culoare HSV pe prosopul albastru.
               Fiabil si rapid -- pentru demo unde drumul e o culoare
               distincta aleasa deliberat. NU foloseste retea neuronala,
               fiindca COCO n-are clasa "drum"/"prosop".

  --mode yolo  (STADIU REAL): YOLOv8-seg. ATENTIE -- modelul standard pe
               COCO NU are clasa drum/asfalt/trotuar. Ca sa mearga pe drum
               real ai nevoie de un model ANTRENAT CUSTOM (Cityscapes/ADE20K)
               sau folosesti podeaua din harta de adancime. Modul asta e
               lasat ca schelet: pui id-ul clasei tale de drum in ROAD_CLASS_ID
               dupa ce ai modelul custom.

In ambele cazuri, restul e identic: masca binara -> azimut din banda de
jos (drumul imediat din fata) -> unghi de servo pentru urmarire drum.

RULARE STANDALONE (azi, cu prosop albastru in fata camerei):
    python3 yolo_segmentation_node.py --mode hsv
    -> fereastra cu masca drumului colorata + directia. 'q' inchide.

RULARE HEADLESS (Pi fara monitor, peste SSH -- vezi scripts/test_ai.sh):
    python3 yolo_segmentation_node.py --mode hsv --headless
    -> salveaza overlay-ul pe disc la /tmp/keryke_preview.jpg (~2Hz),
       vizibil din browser de pe laptop via http.server.
"""

from __future__ import annotations

import argparse
import json
import time

import cv2
import numpy as np

try:
    from .perception_geometry import azimuth_from_mask_band, azimuth_to_servo_angle
except ImportError:
    from perception_geometry import azimuth_from_mask_band, azimuth_to_servo_angle

try:
    from .model_paths import resolve_model_path
except ImportError:
    from model_paths import resolve_model_path

# ROS2 e optional pentru modul standalone/headless (--mode hsv pe laptop,
# fara Docker/ROS2 instalat) -- doar main_ros()/SegmentationRosNode chiar
# au nevoie de el. Daca rclpy/cv_bridge nu sunt instalate, restul
# fisierului (testarea locala cu camera laptopului) tot merge.
try:
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage, CompressedImage
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    Node = object  # fallback ca definitia clasei de mai jos sa nu crape la import

# Segmentarea e semnal de siguranta lent (zona sigura da/nu), nu reactie
# rapida ca YOLO -- rularea ei pe FIECARE cadru de camera (cum era inainte)
# e cost irosit. 0.3s (~3.3Hz) e suficient pt un semnal binar de siguranta,
# acelasi tipar asincron ca depth_node.py (cache ultimul cadru, proceseaza
# pe timer propriu).
SEGMENTATION_PERIOD_S = 0.3

# Prag HSV pentru albastru (prosopul de demo). Ajusteaza la nuanta reala
# a prosopului tau sub lumina din sala -- deschide fereastra si regleaza.
BLUE_LOWER = np.array([90, 80, 50], dtype=np.uint8)
BLUE_UPPER = np.array([130, 255, 255], dtype=np.uint8)

# Pentru --mode yolo: id-ul clasei de drum in modelul TAU custom.
# La modelul COCO standard nu exista, deci ramane None (nu segmenteaza drum).
ROAD_CLASS_ID: int | None = None


def road_mask_hsv(frame: np.ndarray) -> np.ndarray:
    """Masca binara a drumului prin prag de culoare albastra (demo)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
    # curatare: inchide gauri mici, elimina zgomot
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return (mask > 0).astype(np.uint8)


class RoadSegmenter:
    def __init__(self, mode: str = "hsv", model_path: str | None = None,
                 imgsz: int = 320):
        self.mode = mode
        self.imgsz = imgsz
        self.model = None
        if mode == "yolo":
            self.model = self._load_yolo(
                model_path or resolve_model_path("yolov8n-seg.pt") or "yolov8n-seg.pt")

    def _load_yolo(self, model_path: str):
        try:
            from ultralytics import YOLO
            m = YOLO(model_path)
            print(f"[seg] model YOLO-seg incarcat: {model_path}")
            if ROAD_CLASS_ID is None:
                print("[seg] ATENTIE: ROAD_CLASS_ID=None -> modelul COCO "
                      "standard n-are clasa 'drum'. Masca va fi goala pana "
                      "pui un model custom cu clasa de drum.")
            return m
        except Exception as e:
            print(f"[seg] nu am putut incarca YOLO-seg ({e}). Trec pe HSV.")
            self.mode = "hsv"
            return None

    def road_mask(self, frame: np.ndarray) -> np.ndarray:
        """Intoarce masca binara (H,W) a drumului, dupa modul ales."""
        if self.mode == "hsv":
            return road_mask_hsv(frame)

        # mode == yolo
        if self.model is None or ROAD_CLASS_ID is None:
            return np.zeros(frame.shape[:2], dtype=np.uint8)

        results = self.model(frame, imgsz=self.imgsz, verbose=False)
        mask_total = np.zeros(frame.shape[:2], dtype=np.uint8)
        for r in results:
            if r.masks is None:
                continue
            for seg, box in zip(r.masks.data, r.boxes):
                if int(box.cls[0]) != ROAD_CLASS_ID:
                    continue
                m = seg.cpu().numpy()
                m = cv2.resize(m, (frame.shape[1], frame.shape[0]))
                mask_total |= (m > 0.5).astype(np.uint8)
        return mask_total


def walkable_status(mask: np.ndarray, band_azimuth: float | None) -> dict:
    """
    Traduce masca de drum in starea binara "esti pe zona sigura sau nu",
    gata de trimis ca alerta (dashboard/audio). Rostul segmentarii e
    STRICT asta -- un semnal de siguranta, nu ghidare de directie (aia
    e treaba YOLO object detection, vezi bulina din compose_frame() din
    dashboard_node.py). Cand banda din fata (imediat sub picioare) n-are
    niciun pixel walkable, alerta e fixa, fara nicio directie sugerata.
    """
    if band_azimuth is not None:
        return {"on_path": True, "azimuth_deg": band_azimuth, "message": None}

    return {
        "on_path": False,
        "azimuth_deg": None,
        "message": "Atentie! Nu ai pe unde sa mergi.",
    }


def draw_overlay(frame: np.ndarray, mask: np.ndarray,
                 azimuth_deg: float | None, status: dict | None = None) -> np.ndarray:
    """Coloreaza masca drumului peste feed -- semnal binar, fara bulina de
    directie (segmentarea nu ghideaza, doar avertizeaza)."""
    out = frame.copy()
    h, w = out.shape[:2]

    # verde = zona sigura (walkable), rosu = restul cadrului (nu e pe drum) --
    # raspunde direct la "arata-mi ce pixeli decideti voi ca sunt liberi"
    colored = np.zeros_like(out)
    colored[mask > 0] = (0, 200, 0)
    colored[mask == 0] = (0, 0, 160)
    out = cv2.addWeighted(out, 1.0, colored, 0.35, 0)

    # banda din fata (treimea de jos, unde masuram prezenta zonei sigure)
    band_top = int(h * 0.65)
    cv2.line(out, (0, band_top), (w, band_top), (100, 100, 100), 1)

    if azimuth_deg is None:
        message = (status or {}).get("message") or "NU AI PE UNDE SA MERGI"
        cv2.putText(out, message, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    else:
        cv2.putText(out, "zona sigura", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
    return out


def process_frame(seg: RoadSegmenter, frame: np.ndarray) -> tuple[np.ndarray, dict]:
    """Functia pura reutilizabila (bucla locala SAU callback ROS2)."""
    mask = seg.road_mask(frame)
    azimuth = azimuth_from_mask_band(mask)
    status = walkable_status(mask, azimuth)
    info = {
        "road_found": azimuth is not None,
        "azimuth_deg": azimuth,
        "servo_angle": azimuth_to_servo_angle(azimuth) if azimuth is not None else None,
        "walkable": status,
    }
    overlay = draw_overlay(frame, mask, azimuth, status)
    return overlay, info


class SegmentationRosNode(Node):
    """
    Nod ROS2: consuma /perception/image_raw, ruleaza segmentarea (implicit
    HSV pe prosopul albastru) pe un timer propriu (nu pe fiecare cadru --
    vezi SEGMENTATION_PERIOD_S) si publica starea "zona sigura" ca JSON pe
    /perception/walkable_status, plus overlay-ul deja colorat ca JPEG pe
    /perception/segmentation_overlay -- dashboard-ul il preia direct (bytes
    gata comprimati), nu-l mai recalculeaza separat (elimina o dublare
    reala de calcul HSV intre acest nod si dashboard_node.py).
    """

    def __init__(self):
        super().__init__("yolo_segmentation_node")
        self.bridge = CvBridge()
        self.seg = RoadSegmenter(mode="hsv")
        self._latest_frame = None
        self.status_pub = self.create_publisher(
            String, "/perception/walkable_status", 10)
        self.overlay_pub = self.create_publisher(
            CompressedImage, "/perception/segmentation_overlay", 1)
        # ReentrantCallbackGroup -- fara el, _on_image si _on_timer cad in
        # grupul implicit MutuallyExclusive al nodului si NU pot rula
        # concurent: cat timp _on_timer proceseaza (mai ales daca dureaza
        # mult sub contentie CPU cu YOLO), _on_image e blocat sa mai
        # actualizeze _latest_frame -- exact simptomul "acelasi cadru
        # ramane, mereu zice ca nu ai pe unde sa mergi" observat pe Pi.
        cb_group = ReentrantCallbackGroup()
        self.create_subscription(RosImage, "/perception/image_raw", self._on_image, 10,
                                 callback_group=cb_group)
        self.create_timer(SEGMENTATION_PERIOD_S, self._on_timer, callback_group=cb_group)
        self.get_logger().info(
            "SegmentationRosNode pornit (mod hsv, prosop albastru), "
            f"proceseaza la {1.0/SEGMENTATION_PERIOD_S:.1f}Hz, publica pe "
            "/perception/walkable_status si /perception/segmentation_overlay.")

    def _on_image(self, msg: RosImage) -> None:
        # Doar tine cel mai recent cadru -- procesarea reala ruleaza pe
        # timer, nu la rata bruta a camerei (vezi SEGMENTATION_PERIOD_S).
        self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _on_timer(self) -> None:
        if self._latest_frame is None:
            return
        t0 = time.monotonic()
        overlay, info = process_frame(self.seg, self._latest_frame)

        out = String()
        out.data = json.dumps(info["walkable"], ensure_ascii=False)
        self.status_pub.publish(out)

        ok, jpg = cv2.imencode(".jpg", overlay, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            comp = CompressedImage()
            comp.header.stamp = self.get_clock().now().to_msg()
            comp.format = "jpeg"
            comp.data = jpg.tobytes()
            self.overlay_pub.publish(comp)

        elapsed_ms = (time.monotonic() - t0) * 1000.0
        if elapsed_ms > SEGMENTATION_PERIOD_S * 1000.0:
            self.get_logger().warn(
                f"Segmentarea a durat {elapsed_ms:.0f}ms, mai mult decat "
                f"perioada tinta {SEGMENTATION_PERIOD_S*1000:.0f}ms -- "
                f"daca apare des, e contentie CPU cu YOLO, nu procesarea HSV insasi."
            )


def main_ros(args=None):
    """Punct de intrare ROS2 (`ros2 run hive_perception yolo_segmentation_ros`)."""
    if not _ROS_AVAILABLE:
        raise RuntimeError(
            "rclpy/cv_bridge nu sunt instalate in acest mediu -- main_ros() "
            "necesita ROS2 (ruleaza in containerul hive). Pentru test local "
            "fara ROS2, foloseste main() / --mode hsv."
        )
    rclpy.init(args=args)
    node = SegmentationRosNode()
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
    parser.add_argument("--mode", choices=["hsv", "yolo"], default="hsv")
    parser.add_argument("--model", default=None,
                         help="implicit: cauta yolov8n-seg.pt in models/ (Docker/repo/env)")
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--source", default="0")
    parser.add_argument("--headless", action="store_true",
                         help="fara fereastra grafica (Pi fara monitor, peste SSH) -- "
                              "salveaza overlay-ul periodic pe disc, la --preview-path")
    parser.add_argument("--preview-path", default="/tmp/keryke_preview.jpg",
                         help="unde salvez ultimul cadru cu overlay cand --headless")
    args = parser.parse_args()

    seg = RoadSegmenter(mode=args.mode, model_path=args.model, imgsz=args.imgsz)

    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"[seg] nu pot deschide sursa {args.source}")
        return

    if args.headless:
        print(f"[seg] rulez headless in mod '{seg.mode}'. Overlay salvat la "
              f"{args.preview_path} (~2Hz). Ctrl+C opreste.")
    else:
        print(f"[seg] rulez in mod '{seg.mode}'. Apasa 'q' ca sa inchizi.")

    last_save = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            overlay, info = process_frame(seg, frame)
            if info["road_found"]:
                print(f"drum az={info['azimuth_deg']:+.0f} servo={info['servo_angle']:.0f}")
            if args.headless:
                now = time.monotonic()
                if now - last_save > 0.5:
                    cv2.imwrite(args.preview_path, overlay)
                    last_save = now
            else:
                cv2.imshow("Keryke - segmentare drum", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
