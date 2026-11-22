#!/usr/bin/env python3
"""AGV kamera servisi (Pi HOST'ta calisir, Docker'da degil).

IMX500 AI kamera + SSD MobileNetV2 nesne tespiti, donanim MJPEG yayini.
  http://<Pi-IP>:8000/                 -> sade kamera sayfasi
  http://<Pi-IP>:8000/stream.mjpg      -> canli MJPEG
  http://<Pi-IP>:8000/snapshot.jpg     -> tek kare
  http://<Pi-IP>:8000/detections.json  -> {"t","fps","ai","detections":[{"label","score","bbox":[x,y,w,h]}]}
                                          bbox 0..1 araliginda (goruntu genisligi/yuksekligine gore)
Kamerasiz test:  python3 kamera_servis.py --fake
"""
import argparse
import io
import json
import os
import socket
import socketserver
import sys
import threading
import time
from http import server

import cv2
import numpy as np

MODEL = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"

COCO = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "-", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "-", "backpack", "umbrella", "-", "-",
        "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "-", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
        "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "-", "dining table",
        "-", "-", "toilet", "-", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
        "oven", "toaster", "sink", "refrigerator", "-", "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush"]


class FrameBuffer(io.BufferedIOBase):
    """Sadece EN SON kareyi tutar. Yavas istemci araya giren kareleri kacirir (gecikme birikmez)."""

    def __init__(self):
        super().__init__()
        self.frame, self.fid = None, 0
        self.cond = threading.Condition()

    def write(self, buf):
        with self.cond:
            self.frame = bytes(buf)
            self.fid += 1
            self.cond.notify_all()
        return len(buf)


out = FrameBuffer()
det_lock = threading.Lock()
det_state = {"t": 0.0, "fps": 0.0, "ai": False, "detections": []}


def set_dets(dets, fps, ai):
    with det_lock:
        det_state.update(t=round(time.time(), 3), fps=round(fps, 1), ai=ai, detections=dets)


def draw_boxes(img, dets, w, h, fps):
    for d in dets:
        x, y, bw, bh = d["bbox"]
        x0, y0 = int(x * w), int(y * h)
        x1, y1 = int((x + bw) * w), int((y + bh) * h)
        color = (0, 0, 255) if d["label"] == "person" else (0, 255, 0)
        if img.shape[2] == 4:
            color = color + (0,)
        cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
        cv2.putText(img, f"{d['label']} {d['score']:.2f}", (x0 + 4, max(y0 - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    white = (255, 255, 255) if img.shape[2] == 3 else (255, 255, 255, 0)
    cv2.putText(img, f"{fps:.0f} fps  {len(dets)} nesne", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, white, 2)


# --------------------------------------------------------------------------- gercek kamera
def run_camera(args):
    try:
        from picamera2 import Picamera2, MappedArray
        from picamera2.encoders import MJPEGEncoder
        from picamera2.outputs import FileOutput
    except Exception as e:  # noqa: BLE001
        print(f"HATA: picamera2 yuklenemedi ({e}). Bu servis Pi'nin kendisinde (Docker disinda) calismali.",
              file=sys.stderr)
        sys.exit(2)

    W, H = args.width, args.height
    imx500 = intr = None
    labels = COCO
    if not args.no_ai:
        try:
            from picamera2.devices import IMX500
            from picamera2.devices.imx500 import NetworkIntrinsics
            if not os.path.exists(args.model):
                raise FileNotFoundError(f"model yok: {args.model} (sudo apt install imx500-all)")
            imx500 = IMX500(args.model)
            intr = imx500.network_intrinsics or NetworkIntrinsics()
            intr.task = "object detection"
            if intr.labels is None:
                intr.labels = COCO
            intr.update_with_defaults()
            labels = list(intr.labels)
            if intr.ignore_dash_labels:
                labels = [lb for lb in labels if lb and lb != "-"]
        except Exception as e:  # noqa: BLE001
            print(f"UYARI: IMX500 AI baslatilamadi ({e}). Kamera AI'siz (sadece goruntu) aciliyor.",
                  file=sys.stderr)
            imx500 = intr = None

    try:
        picam2 = Picamera2(imx500.camera_num) if imx500 else Picamera2()
        fps_target = min(30, (intr.inference_rate if intr else 30) or 30)
        config = picam2.create_video_configuration(main={"size": (W, H)},
                                                   controls={"FrameRate": fps_target}, buffer_count=4)
    except Exception as e:  # noqa: BLE001
        print(f"HATA: Kamera acilamadi: {e}\n  -> Kablo/flex baglantisini kontrol et, "
              f"'rpicam-hello --list-cameras' ile kamerayi gor, baska bir program kamerayi kullaniyor olabilir.",
              file=sys.stderr)
        sys.exit(3)

    st = {"last": [], "t_prev": time.time(), "fps": 0.0}

    def parse(metadata):
        o = imx500.get_outputs(metadata, add_batch=True)
        if o is None:
            return None
        _, in_h = imx500.get_input_size()
        boxes, scores, classes = o[0][0], o[1][0], o[2][0]
        if intr.bbox_normalization:
            boxes = boxes / in_h
        if intr.bbox_order == "xy":
            boxes = boxes[:, [1, 0, 3, 2]]
        dets = []
        for box, s, c in zip(boxes, scores, classes):
            if s < args.threshold:
                continue
            ci = int(c)
            name = labels[ci] if 0 <= ci < len(labels) else f"id{ci}"
            if name == "-":
                continue
            x, y, w, h = imx500.convert_inference_coords(tuple(float(v) for v in box), metadata, picam2)
            dets.append({"label": name, "score": round(float(s), 3),
                         "bbox": [round(x / W, 4), round(y / H, 4), round(w / W, 4), round(h / H, 4)]})
        return dets

    def pre_cb(request):
        if imx500 is not None:
            d = parse(request.get_metadata())
            if d is not None:
                st["last"] = d
        now = time.time()
        st["fps"] = 0.9 * st["fps"] + 0.1 * (1.0 / max(now - st["t_prev"], 1e-3))
        st["t_prev"] = now
        set_dets(st["last"], st["fps"], imx500 is not None)
        with MappedArray(request, "main") as m:
            draw_boxes(m.array, st["last"], W, H, st["fps"])

    try:
        if imx500 is not None:
            imx500.show_network_fw_progress_bar()
        picam2.configure(config)
        picam2.pre_callback = pre_cb
        if intr is not None and intr.preserve_aspect_ratio:
            imx500.set_auto_aspect_ratio()
        picam2.start_recording(MJPEGEncoder(), FileOutput(out))
    except Exception as e:  # noqa: BLE001
        print(f"HATA: Kamera baslatilamadi: {e}", file=sys.stderr)
        sys.exit(3)
    return picam2


# --------------------------------------------------------------------------- sahte kamera
def run_fake(args):
    W, H = args.width, args.height

    def loop():
        t0 = time.time()
        t_prev, fps = t0, 0.0
        while True:
            t = time.time() - t0
            img = np.zeros((H, W, 3), np.uint8)
            img[:] = (40, 36, 32)
            for gx in range(0, W, 40):
                cv2.line(img, (gx, 0), (gx, H), (60, 56, 52), 1)
            for gy in range(0, H, 40):
                cv2.line(img, (0, gy), (W, gy), (60, 56, 52), 1)
            cv2.putText(img, "SAHTE KAMERA " + time.strftime("%H:%M:%S"), (8, H - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            px = 0.5 + 0.3 * np.sin(t * 0.7)
            cx = 0.5 + 0.35 * np.cos(t * 0.4)
            dets = [{"label": "person", "score": round(0.80 + 0.15 * float(np.sin(t)), 3),
                     "bbox": [round(px - 0.1, 4), 0.2, 0.2, 0.6]},
                    {"label": "chair", "score": 0.62, "bbox": [round(cx - 0.08, 4), 0.55, 0.16, 0.3]}]
            if int(t) % 6 < 3:
                dets.append({"label": "bottle", "score": 0.55, "bbox": [0.75, 0.1, 0.06, 0.15]})
            dets = [d for d in dets if d["score"] >= args.threshold]
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
            t_prev = now
            draw_boxes(img, dets, W, H, fps)
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                out.write(jpg.tobytes())
            set_dets(dets, fps, True)
            time.sleep(max(0.0, 1 / 15 - (time.time() - now)))

    threading.Thread(target=loop, daemon=True).start()


# --------------------------------------------------------------------------- HTTP
PAGE = b"""<!doctype html><html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>AGV Kamera</title></head>
<body style="margin:0;background:#111;color:#ccc;font:14px system-ui">
<img src="stream.mjpg" style="width:100%;display:block">
<p style="padding:8px">Tam panel (harita + kamera): bu adreste port 8080</p></body></html>"""


class Handler(server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"

    def log_message(self, *a):
        pass

    def _hdr(self, code, ctype, length=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, no-store, private")
        if length is not None:
            self.send_header("Content-Length", str(length))
        self.end_headers()

    def _send(self, code, ctype, body):
        self._hdr(code, ctype, len(body))
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", PAGE)
        elif path == "/detections.json":
            with det_lock:
                body = json.dumps(det_state).encode()
            self._send(200, "application/json", body)
        elif path == "/snapshot.jpg":
            with out.cond:
                if out.frame is None:
                    out.cond.wait(timeout=2)
                f = out.frame
            if f is None:
                self._send(503, "text/plain; charset=utf-8", "kare yok".encode())
            else:
                self._send(200, "image/jpeg", f)
        elif path == "/stream.mjpg":
            self._stream()
        else:
            self._send(404, "text/plain", b"yok")

    def _stream(self):
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        self._hdr(200, "multipart/x-mixed-replace; boundary=FRAME")
        last = -1
        try:
            while True:
                with out.cond:
                    out.cond.wait_for(lambda: out.fid != last, timeout=5)
                    if out.fid == last:
                        continue
                    f, last = out.frame, out.fid
                self.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(f)).encode() + b"\r\n\r\n" + f + b"\r\n")
                self.wfile.flush()
        except Exception:  # noqa: BLE001  (istemci koptu)
            pass


class Srv(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    ap = argparse.ArgumentParser(description="AGV kamera servisi (MJPEG + IMX500 nesne tespiti)")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--threshold", type=float, default=0.5, help="tespit esigi 0..1")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--no-ai", action="store_true", help="IMX500 modeli yukleme, sadece goruntu")
    ap.add_argument("--fake", action="store_true", help="kamerasiz test: sahte kare + sahte kutular")
    args = ap.parse_args()

    try:
        srv = Srv(("", args.port), Handler)
    except OSError as e:
        print(f"HATA: port {args.port} acilamadi ({e}). Eski bir kamera programi calisiyor olabilir.",
              file=sys.stderr)
        sys.exit(4)

    picam2 = None
    if args.fake:
        run_fake(args)
        print("SAHTE kamera modu (--fake)", flush=True)
    else:
        picam2 = run_camera(args)
    print(f"Kamera hazir: http://<Pi-IP>:{args.port}/stream.mjpg  (detections.json, snapshot.jpg)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if picam2 is not None:
            try:
                picam2.stop_recording()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    main()
