"""§7 WebSocket sunucusu (asyncio, ROS'suz).

Mimari:
  - GatewayServer asyncio döngüsünde koşar (ana thread).
  - ROS tarafı `Backend` arayüzünü uygular (gateway_node.RosBackend); rclpy executor ayrı thread'de döner.
  - Sunucu -> backend: düz metot çağrıları (rclpy publish/thread-safe Buffer okuma). Bloklayabilecek
    işler (save_map) backend içinde kendi thread'inde yapılır.
  - Backend -> sunucu: `server.post_threadsafe(msg, to=...)` (loop.call_soon_threadsafe) ile.
  - Periyodik yayınlar sunucuda: telemetry 10 Hz, scan 5 Hz, map değişince ≤0.5 Hz, görüntü istemci fps'iyle,
    teleop watchdog 20 Hz kontrol.

Bu modül websockets >= 9.1 (Ubuntu 22.04 apt) ile >= 14 (yeni asyncio API) arasında uyumlu yazılmıştır.
"""
from __future__ import annotations

import asyncio
import base64
import itertools
import logging
import time
from typing import Any, Dict, List, Optional, Tuple

from . import protocol as P

log = logging.getLogger("agv_gateway")


class Backend:
    """ROS (veya test için sahte) arka uç arayüzü. Tüm metotlar asyncio thread'inden çağrılır
    ve hızlı dönmelidir."""

    # ---- okumalar ----
    def get_telemetry(self) -> Dict[str, Any]:
        raise NotImplementedError

    def get_scan(self) -> Optional[Dict[str, Any]]:
        """{"t", "pose", "points", "frame"} veya None (yeni tarama yoksa da None dönebilir)."""
        raise NotImplementedError

    def get_map(self) -> Optional[Dict[str, Any]]:
        """{"seq", "width", "height", "resolution", "origin":(x,y,yaw), "data"} veya None."""
        raise NotImplementedError

    def get_image(self) -> Optional[Tuple[int, float, bytes]]:
        """(seq, t, jpeg) veya None."""
        raise NotImplementedError

    def get_detections(self) -> Optional[Tuple[int, float, List[Dict[str, Any]]]]:
        raise NotImplementedError

    def current_mode(self) -> Optional[str]:
        return None

    # ---- komutlar ----
    def publish_teleop(self, vx: float, wz: float) -> None:
        raise NotImplementedError

    def publish_estop(self, active: bool) -> None:
        raise NotImplementedError

    def publish_set_mode(self, mode: str) -> None:
        raise NotImplementedError

    def send_goal(self, x: float, y: float, yaw: float) -> Tuple[bool, str]:
        raise NotImplementedError

    def cancel_goal(self) -> Tuple[bool, str]:
        raise NotImplementedError

    def nav_state(self) -> Tuple[str, Optional[Tuple[float, float, float]]]:
        raise NotImplementedError

    def save_map(self, name: str, reply) -> None:
        """Arka planda çalışır, bitince reply(ok: bool, msg: str) çağırır (herhangi bir thread'den)."""
        raise NotImplementedError

    def stations_changed(self, json_str: str) -> None:
        """/agv/stations latched yayını."""
        raise NotImplementedError


class Client:
    _ids = itertools.count(1)

    def __init__(self, ws, settings: P.ClientSettings):
        self.id = next(Client._ids)
        self.ws = ws
        self.settings = settings
        self.queue: asyncio.Queue = asyncio.Queue(maxsize=256)
        self.bucket = P.TokenBucket(rate=60.0, burst=120.0)
        self.last_image_seq = -1
        self.last_image_sent = 0.0
        self.last_map_seq = -1
        self.last_flood_warn = 0.0
        self.authed = False
        try:
            self.addr = "%s:%s" % tuple(ws.remote_address[:2])
        except Exception:
            self.addr = "?"

    def __repr__(self):
        return f"client#{self.id}({self.addr})"


class GatewayServer:
    def __init__(self, backend: Backend, stations: P.StationStore, host: str = "0.0.0.0",
                 port: int = 8765, token: str = "", teleop_timeout: float = 0.5,
                 max_lin: float = 0.8, max_ang: float = 1.0, telemetry_hz: float = 10.0,
                 scan_hz: float = 5.0, map_period: float = 2.0, allow_takeover: bool = True,
                 estop_cancels_goal: bool = True, auth_timeout: float = 5.0):
        self.backend = backend
        self.stations = stations
        self.host, self.port = host, port
        self.token = token or ""
        self.max_lin, self.max_ang = max_lin, max_ang
        self.telemetry_hz, self.scan_hz, self.map_period = telemetry_hz, scan_hz, map_period
        self.estop_cancels_goal = estop_cancels_goal
        self.auth_timeout = auth_timeout
        self.arbiter = P.ControlArbiter(allow_takeover=allow_takeover)
        self.watchdog = P.TeleopWatchdog(timeout=teleop_timeout)
        self.clients: Dict[int, Client] = {}
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self._server = None
        self._tasks: List[asyncio.Task] = []
        self._map_cache: Optional[Tuple[int, Dict[str, Any]]] = None  # (seq, map msg)
        self._last_det_seq = -1
        self._stop = None

    # ------------------------------------------------------------------ yaşam döngüsü
    async def start(self):
        import websockets

        self.loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        self._server = await websockets.serve(self._handler, self.host, self.port,
                                              max_size=1 << 20, ping_interval=10, ping_timeout=10)
        try:
            self.port = self._server.sockets[0].getsockname()[1]
        except Exception:
            pass
        self._tasks = [asyncio.ensure_future(c) for c in (
            self._telemetry_loop(), self._scan_loop(), self._map_loop(), self._image_loop(),
            self._detections_loop(), self._watchdog_loop())]
        self.backend.stations_changed(self.stations.to_json())
        log.info("gateway ws://%s:%d", self.host, self.port)

    async def stop(self):
        for t in self._tasks:
            t.cancel()
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
        if self.watchdog.stop():
            self.backend.publish_teleop(0.0, 0.0)

    async def serve_forever(self):
        await self.start()
        try:
            await self._stop.wait()
        finally:
            await self.stop()

    def request_stop(self):
        if self.loop and self._stop:
            self.loop.call_soon_threadsafe(self._stop.set)

    # ------------------------------------------------------------------ gönderim yardımcıları
    def _enqueue(self, c: Client, obj: Dict[str, Any], droppable: bool = False) -> None:
        if droppable and c.queue.qsize() > 4:
            return  # yavaş istemci: görüntü/scan/harita atlanır
        try:
            c.queue.put_nowait(P.dumps(obj))
        except asyncio.QueueFull:
            log.warning("%s kuyruğu doldu, bağlantı kapatılıyor", c)
            asyncio.ensure_future(c.ws.close(1013, "send queue full"))

    def broadcast(self, obj: Dict[str, Any], droppable: bool = False, pred=None) -> None:
        for c in list(self.clients.values()):
            if c.authed and (pred is None or pred(c)):
                self._enqueue(c, obj, droppable)

    def post_threadsafe(self, obj: Dict[str, Any], to: Optional[int] = None) -> None:
        """Backend thread'lerinden güvenli gönderim. to=None -> herkese."""
        if self.loop is None:
            return

        def _do():
            if to is None:
                self.broadcast(obj)
            elif to in self.clients:
                self._enqueue(self.clients[to], obj)
        self.loop.call_soon_threadsafe(_do)

    def log_all(self, level: str, msg: str) -> None:
        self.post_threadsafe(P.msg_log(level, msg))

    async def _sender(self, c: Client):
        try:
            while True:
                s = await c.queue.get()
                await c.ws.send(s)
        except asyncio.CancelledError:
            raise
        except Exception:
            pass  # bağlantı kapandı; handler temizler

    # ------------------------------------------------------------------ bağlantı
    async def _handler(self, ws, path=None):  # path: websockets < 10 uyumluluğu
        c = Client(ws, P.ClientSettings())
        self.clients[c.id] = c
        sender = asyncio.ensure_future(self._sender(c))
        role = None
        try:
            if self.token:
                if not await self._authenticate(c):
                    return
            c.authed = True
            role = self.arbiter.join(c.id)
            log.info("%s bağlandı, rol=%s", c, role)
            self._enqueue(c, P.msg_hello(role))
            self._enqueue(c, P.msg_stations(self.stations.items()))
            if self._map_cache is not None and c.settings.map:
                c.last_map_seq = self._map_cache[0]
                self._enqueue(c, self._map_cache[1])
            async for raw in ws:
                await self._on_message(c, raw)
        except Exception as e:  # ConnectionClosed vb.
            if not _is_conn_closed(e):
                log.exception("%s handler hatası", c)
        finally:
            self.clients.pop(c.id, None)
            if role is not None and self.arbiter.leave(c.id):
                log.info("controller %s ayrıldı", c)
                if self.watchdog.stop():
                    self.backend.publish_teleop(0.0, 0.0)
                self.broadcast(P.msg_log("warn", "controller ayrıldı; request_control ile kontrol alınabilir"))
            # Kuyrukta kalanları göndermesi için kısa süre tanı
            await asyncio.sleep(0)
            sender.cancel()
            log.info("%s ayrıldı", c)

    async def _authenticate(self, c: Client) -> bool:
        try:
            raw = await asyncio.wait_for(c.ws.recv(), timeout=self.auth_timeout)
            msg = P.parse_client_message(raw)
            if msg["type"] != "auth":
                raise P.ProtocolError("önce auth gönderilmeli", "auth")
            if msg["token"] != self.token:
                raise P.ProtocolError("geçersiz token", "auth")
        except (asyncio.TimeoutError, P.ProtocolError) as e:
            reason = "auth zaman aşımı" if isinstance(e, asyncio.TimeoutError) else str(e)
            try:
                await c.ws.send(P.dumps(P.msg_ack("auth", False, reason)))
                await c.ws.close(4001, "unauthorized")
            except Exception:
                pass
            return False
        self._enqueue(c, P.msg_ack("auth", True, "ok"))
        return True

    # ------------------------------------------------------------------ mesaj işleme
    def _ack(self, c: Client, cmd: str, ok: bool, msg: str = "") -> None:
        self._enqueue(c, P.msg_ack(cmd, ok, msg))

    async def _on_message(self, c: Client, raw) -> None:
        if not c.bucket.take():
            now = time.monotonic()
            if now - c.last_flood_warn > 1.0:
                c.last_flood_warn = now
                self._enqueue(c, P.msg_log("warn", "mesaj hızı çok yüksek, bazı mesajlar atlandı"))
            return
        try:
            msg = P.parse_client_message(raw, self.max_lin, self.max_ang)
        except P.ProtocolError as e:
            self._ack(c, e.cmd, False, str(e))
            return
        typ = msg["type"]
        role = self.arbiter.role(c.id)
        ok, why = P.authorize(msg, role)
        if not ok:
            self._ack(c, typ, False, why)
            return
        handler = getattr(self, "_cmd_" + typ)
        try:
            res = handler(c, msg)
            if asyncio.iscoroutine(res):
                await res
        except Exception as e:
            log.exception("komut hatası %s", typ)
            self._ack(c, typ, False, f"iç hata: {e}")

    # --- komutlar
    def _cmd_auth(self, c, msg):
        self._ack(c, "auth", True, "zaten doğrulandı")

    def _cmd_ping(self, c, msg):
        self._enqueue(c, P.msg_pong(msg.get("t")))

    def _cmd_teleop(self, c, msg):
        # teleop için ack yok (≥10 Hz akış); watchdog beslenir.
        self.watchdog.feed()
        self.backend.publish_teleop(msg["vx"], msg["wz"])

    def _cmd_estop(self, c, msg):
        active = msg["active"]
        if active:
            if self.watchdog.stop():
                self.backend.publish_teleop(0.0, 0.0)
            self.backend.publish_estop(True)
            if self.estop_cancels_goal and self.backend.nav_state()[0] == P.NAV_ACTIVE:
                self.backend.cancel_goal()
            self.broadcast(P.msg_log("warn", f"ESTOP etkin ({c})"))
            self._ack(c, "estop", True, "ESTOP etkin")
        else:
            self.backend.publish_estop(False)
            self._ack(c, "estop", True, "ESTOP serbest bırakma isteği gönderildi; ardından set_mode gönderin")

    def _cmd_set_mode(self, c, msg):
        self.backend.publish_set_mode(msg["mode"])
        self._ack(c, "set_mode", True, f"{msg['mode']} isteği gönderildi")

    def _goal(self, c, cmd, x, y, yaw):
        mode = self.backend.current_mode()
        ok, m = self.backend.send_goal(x, y, yaw)
        if ok and mode and mode != "AUTO":
            m += f" (uyarı: mod {mode}; hareket için AUTO gerekli)"
        self._ack(c, cmd, ok, m)

    def _cmd_goal(self, c, msg):
        self._goal(c, "goal", msg["x"], msg["y"], msg["yaw"])

    def _cmd_goto_station(self, c, msg):
        st = self.stations.get(msg["name"])
        if st is None:
            self._ack(c, "goto_station", False, f"istasyon yok: {msg['name']}")
            return
        self._goal(c, "goto_station", st["x"], st["y"], st["yaw"])

    def _cmd_cancel_goal(self, c, msg):
        ok, m = self.backend.cancel_goal()
        self._ack(c, "cancel_goal", ok, m)

    def _stations_updated(self):
        self.backend.stations_changed(self.stations.to_json())
        self.broadcast(P.msg_stations(self.stations.items()))

    def _cmd_station_add(self, c, msg):
        try:
            st = self.stations.add(msg["name"], msg["x"], msg["y"], msg["yaw"])
        except OSError as e:
            self._ack(c, "station_add", False, f"kaydedilemedi: {e}")
            return
        self._stations_updated()
        self._ack(c, "station_add", True, f"istasyon kaydedildi: {st['name']}")

    def _cmd_station_remove(self, c, msg):
        try:
            ok = self.stations.remove(msg["name"])
        except OSError as e:
            self._ack(c, "station_remove", False, f"kaydedilemedi: {e}")
            return
        if ok:
            self._stations_updated()
        self._ack(c, "station_remove", ok, "silindi" if ok else f"istasyon yok: {msg['name']}")

    def _cmd_save_map(self, c, msg):
        cid = c.id

        def reply(ok, m):
            self.post_threadsafe(P.msg_ack("save_map", ok, m), to=cid)
        self._enqueue(c, P.msg_log("info", f"harita kaydediliyor: {msg['name']}"))
        self.backend.save_map(msg["name"], reply)

    def _cmd_subscribe(self, c, msg):
        was_map = c.settings.map
        c.settings.apply(msg)
        if c.settings.map and not was_map and self._map_cache is not None:
            c.last_map_seq = self._map_cache[0]
            self._enqueue(c, self._map_cache[1], droppable=False)
        s = c.settings
        self._ack(c, "subscribe", True,
                  f"image={s.image} image_fps={s.image_fps} scan={s.scan} map={s.map}")

    def _cmd_request_control(self, c, msg):
        ok, prev, m = self.arbiter.request(c.id)
        if ok and prev is not None:
            if self.watchdog.stop():
                self.backend.publish_teleop(0.0, 0.0)
            pc = self.clients.get(prev)
            if pc is not None:
                self._enqueue(pc, P.msg_hello(P.ROLE_VIEWER))
                self._enqueue(pc, P.msg_log("warn", f"kontrol {c} tarafından devralındı"))
        if ok:
            self._enqueue(c, P.msg_hello(P.ROLE_CONTROLLER))
        self._ack(c, "request_control", ok, m)

    def _cmd_release_control(self, c, msg):
        ok, m = self.arbiter.release(c.id)
        if ok:
            if self.watchdog.stop():
                self.backend.publish_teleop(0.0, 0.0)
            self._enqueue(c, P.msg_hello(P.ROLE_VIEWER))
        self._ack(c, "release_control", ok, m)

    # ------------------------------------------------------------------ periyodik görevler
    async def _periodic(self, hz: float, fn):
        period = 1.0 / hz
        while True:
            t0 = time.monotonic()
            try:
                r = fn()
                if asyncio.iscoroutine(r):
                    await r
            except asyncio.CancelledError:
                raise
            except Exception:
                log.exception("periyodik görev hatası (%s)", getattr(fn, "__name__", fn))
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _any(self, pred=None) -> bool:
        return any(c.authed and (pred is None or pred(c)) for c in self.clients.values())

    async def _telemetry_loop(self):
        def tick():
            if self._any():
                self.broadcast(self.backend.get_telemetry())
        await self._periodic(self.telemetry_hz, tick)

    async def _scan_loop(self):
        def tick():
            if not self._any(lambda c: c.settings.scan):
                return
            s = self.backend.get_scan()
            if s is not None:
                self.broadcast(P.msg_scan(s["t"], s["pose"], s["points"], s.get("frame")),
                               droppable=True, pred=lambda c: c.settings.scan)
        await self._periodic(self.scan_hz, tick)

    async def _map_loop(self):
        async def tick():
            m = self.backend.get_map()
            if m is None:
                return
            if self._map_cache is None or self._map_cache[0] != m["seq"]:
                # PNG kodlama CPU işi: döngüyü bloklamamak için thread havuzunda
                png_b64 = await asyncio.get_running_loop().run_in_executor(
                    None, P.map_to_png_b64, m["width"], m["height"], m["data"])
                self._map_cache = (m["seq"], P.msg_map(time.time(), m["resolution"], m["width"],
                                                       m["height"], m["origin"], png_b64))
            seq, mm = self._map_cache
            for c in list(self.clients.values()):
                if c.authed and c.settings.map and c.last_map_seq != seq:
                    c.last_map_seq = seq
                    self._enqueue(c, mm)
        await self._periodic(1.0 / self.map_period, tick)

    async def _image_loop(self):
        cache: Dict[int, str] = {}

        def tick():
            if not self._any(lambda c: c.settings.effective_image_fps > 0):
                return
            img = self.backend.get_image()
            if img is None:
                return
            seq, t, jpeg = img
            now = time.monotonic()
            for c in list(self.clients.values()):
                fps = c.settings.effective_image_fps
                if not c.authed or fps <= 0 or c.last_image_seq == seq:
                    continue
                if now - c.last_image_sent < (1.0 / fps) * 0.95:
                    continue
                if seq not in cache:
                    cache.clear()
                    cache[seq] = P.dumps(P.msg_image(t, base64.b64encode(jpeg).decode("ascii")))
                if c.queue.qsize() > 4:
                    continue
                c.last_image_seq = seq
                c.last_image_sent = now
                try:
                    c.queue.put_nowait(cache[seq])
                except asyncio.QueueFull:
                    pass
        await self._periodic(20.0, tick)

    async def _detections_loop(self):
        def tick():
            if not self._any():
                return
            d = self.backend.get_detections()
            if d is None:
                return
            seq, t, items = d
            if seq != self._last_det_seq:
                self._last_det_seq = seq
                self.broadcast(P.msg_detections(t, items))  # küçük mesaj: atlanmaz
        await self._periodic(10.0, tick)

    async def _watchdog_loop(self):
        def tick():
            if self.watchdog.tick():
                self.backend.publish_teleop(0.0, 0.0)
                self.broadcast(P.msg_log("warn", "teleop zaman aşımı: hız sıfırlandı"),
                               pred=lambda c: self.arbiter.role(c.id) == P.ROLE_CONTROLLER)
        await self._periodic(20.0, tick)


def _is_conn_closed(e: Exception) -> bool:
    try:
        import websockets
        return isinstance(e, websockets.exceptions.ConnectionClosed)
    except Exception:
        return False
