"""INTERFACES.md §7 — Gateway WebSocket JSON protokolü (saf Python, ROS'suz).

İçerik:
  - Sunucu->istemci mesaj oluşturucular (hello, telemetry, scan, map, image, detections, stations, ack, log, pong)
  - İstemci->sunucu mesaj doğrulama (parse_client_message) ve yetki kuralları (authorize)
  - OccupancyGrid -> gri PNG (+ base64)
  - Lidar taraması -> sabit frame noktaları, seyreltme
  - Kuaterniyon -> roll/pitch/yaw
  - RateLimiter / TokenBucket, TeleopWatchdog, NavTracker, ControlArbiter, StationStore

=== HARİTA PNG YÖNELİM SÖZLEŞMESİ ===
nav_msgs/OccupancyGrid'de data[0] = (origin.x, origin.y) köşesindeki hücre, satırlar +y yönünde artar
(yani grid satır 0 = EN DÜŞÜK y). PNG'de ise satır 0 = görüntünün ÜSTÜ.
Bu modül grid'i DİKEY ÇEVİRİR:  PNG satır 0 (üst) = grid'in son satırı = EN YÜKSEK y.
PNG piksel (sütun c, satır r) merkezinin dünya (map) koordinatı (origin.yaw = 0 için):
    x = origin.x + (c + 0.5) * resolution
    y = origin.y + (height - 1 - r + 0.5) * resolution
Yani istemci, PNG'yi düz (çevirmeden) çizip +x sağa, +y yukarı olan bir ekranda sol-alt köşeyi
(origin.x, origin.y)'ye koyarsa doğru hizalanır. (map_server'ın .pgm çıktısıyla aynı yönelim.)
Piksel değerleri: 0 = dolu (siyah), 254 = boş (beyaz), 205 = bilinmeyen.
"""
from __future__ import annotations

import base64
import math
import os
import re
import struct
import tempfile
import time
import zlib
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import json

PROTOCOL_VERSION = 1
ROBOT_NAME = "agv200"

ROLE_CONTROLLER = "controller"
ROLE_VIEWER = "viewer"

NAV_IDLE = "idle"
NAV_ACTIVE = "active"
NAV_SUCCEEDED = "succeeded"
NAV_FAILED = "failed"
NAV_CANCELED = "canceled"

# action_msgs/msg/GoalStatus sabitleri
GOAL_STATUS_UNKNOWN = 0
GOAL_STATUS_ACCEPTED = 1
GOAL_STATUS_EXECUTING = 2
GOAL_STATUS_CANCELING = 3
GOAL_STATUS_SUCCEEDED = 4
GOAL_STATUS_CANCELED = 5
GOAL_STATUS_ABORTED = 6

MAP_OCCUPIED = 0
MAP_FREE = 254
MAP_UNKNOWN = 205

MAX_STATION_NAME = 64
MAX_IMAGE_FPS = 10
MAX_SCAN_POINTS = 360


class ProtocolError(ValueError):
    """Geçersiz istemci mesajı. `cmd` ack'te kullanılacak tip."""

    def __init__(self, msg: str, cmd: str = "unknown"):
        super().__init__(msg)
        self.cmd = cmd


def dumps(obj: Dict[str, Any]) -> str:
    return json.dumps(obj, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def _r(v: Optional[float], nd: int = 3) -> Optional[float]:
    if v is None:
        return None
    v = float(v)
    return round(v, nd) if math.isfinite(v) else None


# ==============================================================================================
# Sunucu -> istemci mesajları
# ==============================================================================================
def msg_hello(role: str) -> Dict[str, Any]:
    return {"type": "hello", "version": PROTOCOL_VERSION, "robot": ROBOT_NAME, "role": role}


def msg_ack(cmd: str, ok: bool, msg: str = "") -> Dict[str, Any]:
    return {"type": "ack", "cmd": str(cmd), "ok": bool(ok), "msg": str(msg)}


def msg_log(level: str, msg: str) -> Dict[str, Any]:
    if level not in ("info", "warn", "error"):
        level = "info"
    return {"type": "log", "level": level, "msg": str(msg)}


def msg_pong(t: Any) -> Dict[str, Any]:
    return {"type": "pong", "t": t}


def pose_dict(pose: Optional[Sequence[float]]) -> Optional[Dict[str, float]]:
    if pose is None:
        return None
    x, y, yaw = pose
    return {"x": _r(x), "y": _r(y), "yaw": _r(yaw, 4)}


def msg_telemetry(t: float, status: Optional[Dict[str, Any]], mode: Optional[str],
                  pose: Optional[Sequence[float]], vel: Optional[Sequence[float]],
                  imu: Optional[Sequence[float]], nav_state: str,
                  nav_goal: Optional[Sequence[float]], frame: Optional[str] = None) -> Dict[str, Any]:
    """status: /agv/status JSON (dict) veya None. mode: /agv/mode (status'ta yoksa).
    imu: (roll, pitch, yaw, gz). vel: (vx, wz)."""
    st = status or {}
    m = st.get("mode") or mode or "UNKNOWN"
    vx, wz = vel if vel is not None else (0.0, 0.0)
    roll, pitch, yaw, gz = imu if imu is not None else (0.0, 0.0, 0.0, 0.0)
    batt = st.get("battery_v")
    mfd = st.get("min_front_dist")
    out = {
        "type": "telemetry",
        "t": round(float(t), 3),
        "mode": str(m),
        "estop": bool(st.get("estop", m == "ESTOP")),
        "obstacle_stop": bool(st.get("obstacle_stop", False)),
        # mode_manager reports -1.0 for "no valid reading / nothing in range" -> null
        "min_front_dist": _r(mfd) if isinstance(mfd, (int, float)) and mfd >= 0 else None,
        "battery_v": _r(batt, 2) if isinstance(batt, (int, float)) else None,
        "pose": pose_dict(pose),
        "vel": {"vx": _r(vx), "wz": _r(wz)},
        "imu": {"roll": _r(roll, 4), "pitch": _r(pitch, 4), "yaw": _r(yaw, 4), "gz": _r(gz, 4)},
        "nav": {"state": nav_state, "goal": pose_dict(nav_goal)},
    }
    # CONTRACT-NOTE: ek alanlar (sözleşmede yok, eklemeli/geriye uyumlu):
    #   "frame": pose'un frame'i ("map" | "odom" | null) — map TF'i yokken odom'a düşüldüğünü belirtir.
    #   "teleop_source": /agv/status'tan aynen aktarılır.
    out["frame"] = frame
    if "teleop_source" in st:
        out["teleop_source"] = st.get("teleop_source")
    return out


def msg_scan(t: float, pose: Optional[Sequence[float]], points: List[Tuple[float, float]],
             frame: Optional[str] = None) -> Dict[str, Any]:
    out = {"type": "scan", "t": round(float(t), 3), "pose": pose_dict(pose),
           "points": [[round(x, 3), round(y, 3)] for x, y in points]}
    out["frame"] = frame  # CONTRACT-NOTE: ek alan, bkz. msg_telemetry
    return out


def msg_map(t: float, resolution: float, width: int, height: int,
            origin: Sequence[float], png_b64: str) -> Dict[str, Any]:
    ox, oy, oyaw = origin
    return {"type": "map", "t": round(float(t), 3), "resolution": float(resolution),
            "width": int(width), "height": int(height),
            "origin": {"x": float(ox), "y": float(oy), "yaw": float(oyaw)}, "png_b64": png_b64}


def msg_image(t: float, jpeg_b64: str) -> Dict[str, Any]:
    return {"type": "image", "t": round(float(t), 3), "jpeg_b64": jpeg_b64}


def msg_detections(t: float, items: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {"type": "detections", "t": round(float(t), 3), "items": items}


def msg_stations(items: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {"type": "stations", "items": items}


def parse_detections_json(s: str) -> List[Dict[str, Any]]:
    """/detections/json -> items listesi (bozuk girdileri atar)."""
    try:
        raw = json.loads(s)
    except (TypeError, ValueError):
        return []
    out = []
    if not isinstance(raw, list):
        return out
    for d in raw:
        try:
            out.append({"label": str(d["label"]), "score": float(d["score"]),
                        "bbox": [float(v) for v in d["bbox"]][:4]})
        except (KeyError, TypeError, ValueError):
            continue
    return out


def parse_status_json(s: str) -> Optional[Dict[str, Any]]:
    try:
        d = json.loads(s)
    except (TypeError, ValueError):
        return None
    return d if isinstance(d, dict) else None


# ==============================================================================================
# İstemci -> sunucu doğrulama
# ==============================================================================================
CLIENT_TYPES = {
    "auth", "teleop", "set_mode", "estop", "goal", "goto_station", "cancel_goal", "station_add",
    "station_remove", "save_map", "subscribe", "request_control", "release_control", "ping",
}
# Viewer'ın da gönderebildikleri (estop yalnızca active=true için, bkz. authorize)
VIEWER_ALLOWED = {"auth", "estop", "subscribe", "request_control", "release_control", "ping", "cancel_goal"}

_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]{1,64}$")


def _num(msg: Dict[str, Any], key: str, cmd: str, default: Optional[float] = None) -> float:
    v = msg.get(key, default)
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ProtocolError(f"'{key}' sayı olmalı", cmd)
    v = float(v)
    if not math.isfinite(v):
        raise ProtocolError(f"'{key}' sonlu olmalı", cmd)
    return v


def _bool(msg: Dict[str, Any], key: str, cmd: str) -> bool:
    v = msg.get(key)
    if not isinstance(v, bool):
        raise ProtocolError(f"'{key}' bool olmalı", cmd)
    return v


def _station_name(msg: Dict[str, Any], cmd: str) -> str:
    v = msg.get("name")
    if not isinstance(v, str) or not v.strip():
        raise ProtocolError("'name' boş olmayan metin olmalı", cmd)
    v = v.strip()
    if len(v) > MAX_STATION_NAME:
        raise ProtocolError(f"'name' en fazla {MAX_STATION_NAME} karakter", cmd)
    return v


def parse_client_message(raw: Any, max_lin: float = 0.8, max_ang: float = 1.0) -> Dict[str, Any]:
    """Ham metni doğrular, kanonik dict döndürür. Hata -> ProtocolError (cmd alanıyla)."""
    if isinstance(raw, (bytes, bytearray)):
        raise ProtocolError("binary frame desteklenmez (JSON text bekleniyor)")
    try:
        msg = json.loads(raw)
    except (TypeError, ValueError):
        raise ProtocolError("geçersiz JSON")
    if not isinstance(msg, dict):
        raise ProtocolError("mesaj JSON nesnesi olmalı")
    typ = msg.get("type")
    if not isinstance(typ, str) or typ not in CLIENT_TYPES:
        raise ProtocolError(f"bilinmeyen type: {typ!r}", str(typ))

    if typ == "auth":
        tok = msg.get("token")
        if not isinstance(tok, str):
            raise ProtocolError("'token' metin olmalı", typ)
        return {"type": typ, "token": tok}
    if typ == "teleop":
        vx = _num(msg, "vx", typ)
        wz = _num(msg, "wz", typ)
        return {"type": typ, "vx": max(-max_lin, min(max_lin, vx)), "wz": max(-max_ang, min(max_ang, wz))}
    if typ == "set_mode":
        m = msg.get("mode")
        if m not in ("MANUAL", "AUTO"):
            raise ProtocolError("'mode' MANUAL veya AUTO olmalı", typ)
        return {"type": typ, "mode": m}
    if typ == "estop":
        return {"type": typ, "active": _bool(msg, "active", typ)}
    if typ == "goal":
        return {"type": typ, "x": _num(msg, "x", typ), "y": _num(msg, "y", typ),
                "yaw": _num(msg, "yaw", typ, 0.0)}
    if typ == "goto_station":
        return {"type": typ, "name": _station_name(msg, typ)}
    if typ == "station_add":
        return {"type": typ, "name": _station_name(msg, typ), "x": _num(msg, "x", typ),
                "y": _num(msg, "y", typ), "yaw": _num(msg, "yaw", typ, 0.0)}
    if typ == "station_remove":
        return {"type": typ, "name": _station_name(msg, typ)}
    if typ == "save_map":
        name = msg.get("name", "")
        if not isinstance(name, str) or not _NAME_RE.match(name):
            raise ProtocolError("'name' sadece harf/rakam/_/- içermeli (1-64)", typ)
        return {"type": typ, "name": name}
    if typ == "subscribe":
        out: Dict[str, Any] = {"type": typ}
        for k in ("image", "scan", "map"):
            if k in msg:
                out[k] = _bool(msg, k, typ)
        if "image_fps" in msg:
            fps = _num(msg, "image_fps", typ)
            out["image_fps"] = int(max(0, min(MAX_IMAGE_FPS, round(fps))))
        return out
    if typ == "ping":
        t = msg.get("t")
        if t is not None and (isinstance(t, bool) or not isinstance(t, (int, float))):
            raise ProtocolError("'t' sayı olmalı", typ)
        return {"type": typ, "t": t}
    return {"type": typ}  # cancel_goal, request_control, release_control


def authorize(msg: Dict[str, Any], role: str) -> Tuple[bool, str]:
    """Rol bazlı yetki. Controller her şeyi yapabilir; viewer sadece güvenlik/izleme mesajlarını."""
    if role == ROLE_CONTROLLER:
        return True, ""
    typ = msg["type"]
    if typ == "estop":
        if msg["active"]:
            return True, ""
        return False, "ESTOP serbest bırakma (false) sadece controller'a açık"
    if typ in VIEWER_ALLOWED:
        return True, ""
    return False, f"'{typ}' sadece controller'a açık (request_control gönderin)"


# ==============================================================================================
# Matematik
# ==============================================================================================
def quat_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """(roll, pitch, yaw) radyan, ZYX."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float,
                   sensor_pose: Optional[Sequence[float]] = None,
                   max_points: int = MAX_SCAN_POINTS) -> List[Tuple[float, float]]:
    """LaserScan -> 2D noktalar. sensor_pose = (x, y, yaw): lazerin hedef frame'deki 2D pozu
    (None -> lazer frame'inde döner). Geçersiz ölçümler atılır, sonra ≤ max_points'e eşit adımla seyreltilir."""
    px, py, pyaw = sensor_pose if sensor_pose is not None else (0.0, 0.0, 0.0)
    rmin = max(range_min, 1e-3)
    valid = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < rmin or r > range_max:
            continue
        valid.append((i, r))
    if not valid:
        return []
    step = max(1, math.ceil(len(valid) / max(1, max_points)))
    c, s = math.cos(pyaw), math.sin(pyaw)
    out = []
    for i, r in valid[::step]:
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        out.append((px + c * lx - s * ly, py + s * lx + c * ly))
    return out[:max_points]


# ==============================================================================================
# Harita -> PNG
# ==============================================================================================
def occupancy_to_gray(width: int, height: int, data: Sequence[int],
                      free_thresh: int = 25, occupied_thresh: int = 65):
    """OccupancyGrid data -> numpy uint8 (height, width), DİKEY ÇEVRİLMİŞ (satır 0 = en yüksek y).
    -1 -> 205, <= free_thresh -> 254, >= occupied_thresh -> 0, arası -> 205 (map_saver trinary ile aynı)."""
    import numpy as np

    grid = np.asarray(data, dtype=np.int16)
    if grid.size != width * height:
        raise ValueError(f"map data size {grid.size} != {width}x{height}")
    grid = grid.reshape(height, width)
    img = np.full(grid.shape, MAP_UNKNOWN, dtype=np.uint8)
    known = grid >= 0
    img[known & (grid <= free_thresh)] = MAP_FREE
    img[known & (grid >= occupied_thresh)] = MAP_OCCUPIED
    return np.ascontiguousarray(img[::-1, :])


def encode_png_gray(img) -> bytes:
    """8-bit gri numpy (h, w) -> PNG byte'ları (stdlib zlib, Pillow gerekmez)."""
    import numpy as np

    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape
    raw = np.empty((h, w + 1), dtype=np.uint8)
    raw[:, 0] = 0  # filtre tipi None
    raw[:, 1:] = img

    def chunk(tag: bytes, payload: bytes) -> bytes:
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw.tobytes(), 6)) + chunk(b"IEND", b""))


def map_to_png_b64(width: int, height: int, data: Sequence[int]) -> str:
    return base64.b64encode(encode_png_gray(occupancy_to_gray(width, height, data))).decode("ascii")


def map_fingerprint(width: int, height: int, resolution: float, origin: Sequence[float], data) -> int:
    """Harita değişti mi tespiti için ucuz parmak izi."""
    import numpy as np

    arr = np.asarray(data, dtype=np.int8)
    h = zlib.crc32(arr.tobytes())
    h = zlib.crc32(struct.pack(">IIdddd", width, height, resolution, *origin), h)
    return h


# ==============================================================================================
# Hız sınırlayıcılar
# ==============================================================================================
class RateLimiter:
    """Anahtar bazlı minimum aralık: allow(key, hz) -> en fazla hz kez/saniye True."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._last: Dict[Any, float] = {}

    def allow(self, key: Any, hz: float) -> bool:
        if hz <= 0:
            return False
        now = self._clock()
        last = self._last.get(key)
        if last is not None and now - last < (1.0 / hz) * 0.999:
            return False
        self._last[key] = now
        return True

    def forget(self, key: Any) -> None:
        self._last.pop(key, None)


class TokenBucket:
    """Gelen mesaj taşkın koruması (istemci başına)."""

    def __init__(self, rate: float, burst: float, clock=time.monotonic):
        self.rate = float(rate)
        self.burst = float(burst)
        self.tokens = float(burst)
        self._clock = clock
        self._t = clock()

    def take(self, n: float = 1.0) -> bool:
        now = self._clock()
        self.tokens = min(self.burst, self.tokens + (now - self._t) * self.rate)
        self._t = now
        if self.tokens >= n:
            self.tokens -= n
            return True
        return False


# ==============================================================================================
# Teleop watchdog
# ==============================================================================================
class TeleopWatchdog:
    """Son teleop'tan `timeout` sn sonra TEK SEFER sıfır yayınlanmasını sağlar."""

    def __init__(self, timeout: float = 0.5, clock=time.monotonic):
        self.timeout = timeout
        self._clock = clock
        self._last: Optional[float] = None
        self.active = False

    def feed(self) -> None:
        self._last = self._clock()
        self.active = True

    def stop(self) -> bool:
        """Hemen durdur (controller ayrıldı / estop). Aktifse True (sıfır yayınlanmalı)."""
        was = self.active
        self.active = False
        self._last = None
        return was

    def tick(self) -> bool:
        """True dönerse çağıran sıfır Twist yayınlamalı (bir kez)."""
        if self.active and self._last is not None and self._clock() - self._last > self.timeout:
            self.active = False
            return True
        return False


# ==============================================================================================
# Nav durumu
# ==============================================================================================
def nav_state_from_status(status: int) -> str:
    if status == GOAL_STATUS_SUCCEEDED:
        return NAV_SUCCEEDED
    if status == GOAL_STATUS_CANCELED:
        return NAV_CANCELED
    if status in (GOAL_STATUS_ACCEPTED, GOAL_STATUS_EXECUTING, GOAL_STATUS_CANCELING):
        return NAV_ACTIVE
    return NAV_FAILED  # ABORTED, UNKNOWN


class NavTracker:
    """Nav hedef durum makinesi. Her yeni hedef bir token alır; eski hedeflerin geç gelen
    sonuçları (Nav2 preempt) token uyuşmadığı için yok sayılır. Thread-safe değil: çağıran kilitler."""

    def __init__(self):
        self.state = NAV_IDLE
        self.goal: Optional[Tuple[float, float, float]] = None
        self._token = 0

    def start(self, goal: Tuple[float, float, float]) -> int:
        self._token += 1
        self.goal = tuple(goal)  # type: ignore[assignment]
        self.state = NAV_ACTIVE
        return self._token

    @property
    def token(self) -> int:
        return self._token

    def rejected(self, token: int) -> bool:
        if token != self._token:
            return False
        self.state = NAV_FAILED
        return True

    def finished(self, token: int, status: int) -> Optional[str]:
        """Sonuç işlendi ise yeni durumu, eski hedefe aitse None döndürür."""
        if token != self._token:
            return None
        self.state = nav_state_from_status(status)
        return self.state

    def canceled_locally(self) -> None:
        self._token += 1
        self.state = NAV_CANCELED


# ==============================================================================================
# Controller hakemi
# ==============================================================================================
class ControlArbiter:
    """İlk bağlanan (controller yokken bağlanan) istemci controller olur, diğerleri viewer.
    request_control: controller boşsa verilir; doluysa `allow_takeover` ise devralınır.
    Controller ayrılınca kimse otomatik terfi ETMEZ (güvenlik): yeni bağlanan veya request_control eden alır."""

    def __init__(self, allow_takeover: bool = True):
        self.allow_takeover = allow_takeover
        self.controller: Any = None
        self.clients: List[Any] = []

    def join(self, cid: Any) -> str:
        self.clients.append(cid)
        if self.controller is None:
            self.controller = cid
            return ROLE_CONTROLLER
        return ROLE_VIEWER

    def leave(self, cid: Any) -> bool:
        """Ayrılan controller ise True."""
        if cid in self.clients:
            self.clients.remove(cid)
        if self.controller == cid:
            self.controller = None
            return True
        return False

    def role(self, cid: Any) -> str:
        return ROLE_CONTROLLER if cid == self.controller and cid is not None else ROLE_VIEWER

    def request(self, cid: Any) -> Tuple[bool, Any, str]:
        """(ok, önceki_controller_veya_None, mesaj)"""
        if self.controller == cid:
            return True, None, "zaten controller"
        if self.controller is None:
            self.controller = cid
            return True, None, "controller oldunuz"
        if self.allow_takeover:
            prev = self.controller
            self.controller = cid
            return True, prev, "kontrol devralındı"
        return False, None, "başka bir controller bağlı"

    def release(self, cid: Any) -> Tuple[bool, str]:
        if self.controller != cid:
            return False, "controller değilsiniz"
        self.controller = None
        return True, "kontrol bırakıldı"


# ==============================================================================================
# İstemci abonelik ayarları
# ==============================================================================================
@dataclass
class ClientSettings:
    image: bool = True
    image_fps: int = MAX_IMAGE_FPS
    scan: bool = True
    map: bool = True

    def apply(self, sub: Dict[str, Any]) -> None:
        for k in ("image", "scan", "map"):
            if k in sub:
                setattr(self, k, bool(sub[k]))
        if "image_fps" in sub:
            self.image_fps = int(max(0, min(MAX_IMAGE_FPS, sub["image_fps"])))

    @property
    def effective_image_fps(self) -> int:
        return self.image_fps if self.image else 0


# ==============================================================================================
# İstasyonlar
# ==============================================================================================
class StationStore:
    """İstasyonları YAML dosyasında tutar (varsayılan ~/.agv/stations.yaml).

    Dosya biçimi:
        stations:
          - {name: dock, x: 0.0, y: 0.0, yaw: 0.0}
    Ekleme aynı isim varsa günceller. Yazma atomik (geçici dosya + os.replace).
    """

    def __init__(self, path: str = "~/.agv/stations.yaml"):
        self.path = os.path.expanduser(path)
        self._items: List[Dict[str, Any]] = []
        self.load()

    def load(self) -> None:
        self._items = []
        if not os.path.exists(self.path):
            return
        import yaml

        with open(self.path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        raw = doc.get("stations", []) if isinstance(doc, dict) else doc
        for s in raw or []:
            try:
                self._upsert({"name": str(s["name"]), "x": float(s["x"]), "y": float(s["y"]),
                              "yaw": float(s.get("yaw", 0.0))})
            except (KeyError, TypeError, ValueError):
                continue

    def _save(self) -> None:
        import yaml

        d = os.path.dirname(self.path) or "."
        os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(prefix=".stations.", suffix=".yaml", dir=d)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                yaml.safe_dump({"stations": self._items}, f, allow_unicode=True, sort_keys=False)
            os.replace(tmp, self.path)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    def _upsert(self, st: Dict[str, Any]) -> None:
        for i, s in enumerate(self._items):
            if s["name"] == st["name"]:
                self._items[i] = st
                return
        self._items.append(st)

    def add(self, name: str, x: float, y: float, yaw: float) -> Dict[str, Any]:
        st = {"name": name, "x": round(float(x), 4), "y": round(float(y), 4),
              "yaw": round(normalize_angle(float(yaw)), 4)}
        self._upsert(st)
        self._save()
        return st

    def remove(self, name: str) -> bool:
        n = len(self._items)
        self._items = [s for s in self._items if s["name"] != name]
        if len(self._items) != n:
            self._save()
            return True
        return False

    def get(self, name: str) -> Optional[Dict[str, Any]]:
        for s in self._items:
            if s["name"] == name:
                return dict(s)
        return None

    def items(self) -> List[Dict[str, Any]]:
        return [dict(s) for s in self._items]

    def to_json(self) -> str:
        """/agv/stations gövdesi."""
        return json.dumps(self.items(), separators=(",", ":"), ensure_ascii=False)
