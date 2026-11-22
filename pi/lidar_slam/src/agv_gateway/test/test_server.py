"""GatewayServer'ı sahte backend ile gerçek WebSocket üzerinden uçtan uca test eder (ROS'suz)."""
import asyncio
import json
import os
import sys
import time

import pytest

websockets = pytest.importorskip("websockets")
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..")))
from agv_gateway import protocol as P  # noqa: E402
from agv_gateway.server import Backend, GatewayServer  # noqa: E402

JPEG = b"\xff\xd8\xff\xe0fakejpeg\xff\xd9"


class FakeBackend(Backend):
    def __init__(self):
        self.calls = []
        self.nav = P.NavTracker()
        self.map = {"seq": 1, "width": 2, "height": 2, "resolution": 0.05, "origin": (0, 0, 0),
                    "data": [0, 100, -1, 0]}
        self.img_seq = 0
        self.saved = []

    def get_telemetry(self):
        return P.msg_telemetry(time.time(), {"mode": "MANUAL", "estop": False}, None, (0, 0, 0),
                               (0, 0), (0, 0, 0, 0), self.nav.state, self.nav.goal, "map")

    def get_scan(self):
        return {"t": time.time(), "pose": (0, 0, 0), "points": [(1.0, 0.0)], "frame": "map"}

    def get_map(self):
        return self.map

    def get_image(self):
        self.img_seq += 1
        return (self.img_seq, time.time(), JPEG)

    def get_detections(self):
        return (1, time.time(), [{"label": "person", "score": 0.9, "bbox": [0, 0, 0.5, 0.5]}])

    def current_mode(self):
        return "MANUAL"

    def publish_teleop(self, vx, wz):
        self.calls.append(("teleop", vx, wz))

    def publish_estop(self, a):
        self.calls.append(("estop", a))

    def publish_set_mode(self, m):
        self.calls.append(("set_mode", m))

    def send_goal(self, x, y, yaw):
        self.nav.start((x, y, yaw))
        self.calls.append(("goal", x, y, yaw))
        return True, "ok"

    def cancel_goal(self):
        if self.nav.state != P.NAV_ACTIVE:
            return False, "aktif hedef yok"
        self.nav.canceled_locally()
        self.calls.append(("cancel",))
        return True, "iptal"

    def nav_state(self):
        return self.nav.state, self.nav.goal

    def save_map(self, name, reply):
        import threading
        self.saved.append(name)
        threading.Timer(0.05, reply, args=(True, f"saved {name}")).start()

    def stations_changed(self, s):
        self.calls.append(("stations", s))


class WS:
    """Mesajları tipe göre toplayan test istemcisi."""

    def __init__(self, ws):
        self.ws = ws
        self.msgs = []

    async def send(self, **m):
        await self.ws.send(json.dumps(m))

    async def wait(self, pred, timeout=2.0):
        t_end = time.monotonic() + timeout
        for m in self.msgs:
            if pred(m):
                self.msgs.remove(m)
                return m
        while time.monotonic() < t_end:
            try:
                raw = await asyncio.wait_for(self.ws.recv(), timeout=t_end - time.monotonic())
            except asyncio.TimeoutError:
                break
            m = json.loads(raw)
            if pred(m):
                return m
            self.msgs.append(m)
        raise AssertionError(f"beklenen mesaj gelmedi; alınanlar: {[x.get('type') for x in self.msgs][-20:]}")

    async def ack(self, cmd, timeout=2.0):
        return await self.wait(lambda m: m.get("type") == "ack" and m.get("cmd") == cmd, timeout)

    async def typ(self, t, timeout=2.0):
        return await self.wait(lambda m: m.get("type") == t, timeout)


async def _connect(port):
    return WS(await websockets.connect(f"ws://127.0.0.1:{port}", max_size=8 << 20))


def run(coro):
    return asyncio.run(asyncio.wait_for(coro, 20))


@pytest.fixture
def mk(tmp_path):
    def _mk(**kw):
        be = FakeBackend()
        srv = GatewayServer(be, P.StationStore(str(tmp_path / "stations.yaml")), host="127.0.0.1", port=0, **kw)
        return be, srv
    return _mk


def test_roles_teleop_watchdog_estop(mk):
    be, srv = mk(allow_takeover=False)

    async def main():
        await srv.start()
        a = await _connect(srv.port)
        h = await a.typ("hello")
        assert h == {"type": "hello", "version": 1, "robot": "agv200", "role": "controller"}
        st = await a.typ("stations")
        assert st["items"] == []
        b = await _connect(srv.port)
        assert (await b.typ("hello"))["role"] == "viewer"

        # viewer teleop -> red
        await b.send(type="teleop", vx=0.3, wz=0)
        assert (await b.ack("teleop"))["ok"] is False
        # controller teleop -> yayın, ack yok
        for _ in range(3):
            await a.send(type="teleop", vx=0.3, wz=0.1)
            await asyncio.sleep(0.05)
        await asyncio.sleep(0.1)
        assert ("teleop", 0.3, 0.1) in be.calls
        # watchdog: 0.5 s sonra tek bir sıfır
        await asyncio.sleep(0.8)
        zeros = [c for c in be.calls if c == ("teleop", 0.0, 0.0)]
        assert len(zeros) == 1
        await a.typ("log")

        # viewer ESTOP true serbest, false yasak
        await b.send(type="estop", active=True)
        assert (await b.ack("estop"))["ok"] is True
        await b.send(type="estop", active=False)
        assert (await b.ack("estop"))["ok"] is False
        await a.send(type="estop", active=False)
        assert (await a.ack("estop"))["ok"] is True
        assert ("estop", True) in be.calls and ("estop", False) in be.calls

        # takeover kapalı: viewer kontrol alamaz
        await b.send(type="request_control")
        assert (await b.ack("request_control"))["ok"] is False
        await a.send(type="release_control")
        assert (await a.ack("release_control"))["ok"] is True
        assert (await a.typ("hello"))["role"] == "viewer"
        await b.send(type="request_control")
        assert (await b.ack("request_control"))["ok"] is True
        assert (await b.typ("hello"))["role"] == "controller"

        # telemetry akıyor
        tel = await a.typ("telemetry")
        assert tel["mode"] == "MANUAL" and "nav" in tel
        await a.ws.close()
        await b.ws.close()
        await srv.stop()
    run(main())


def test_controller_disconnect_stops_teleop(mk):
    be, srv = mk()

    async def main():
        await srv.start()
        a = await _connect(srv.port)
        await a.typ("hello")
        b = await _connect(srv.port)
        await b.typ("hello")
        await a.send(type="teleop", vx=0.5, wz=0)
        await asyncio.sleep(0.1)
        await a.ws.close()
        await asyncio.sleep(0.2)
        assert be.calls[-1] == ("teleop", 0.0, 0.0) or ("teleop", 0.0, 0.0) in be.calls
        # takeover açık: b request_control
        await b.send(type="request_control")
        assert (await b.ack("request_control"))["ok"]
        await b.ws.close()
        await srv.stop()
    run(main())


def test_goals_stations_save_map_ping(mk):
    be, srv = mk()

    async def main():
        await srv.start()
        a = await _connect(srv.port)
        await a.typ("hello")
        await a.send(type="station_add", name="dock", x=1.0, y=2.0, yaw=0.5)
        assert (await a.ack("station_add"))["ok"]
        st = await a.wait(lambda m: m["type"] == "stations" and m["items"])
        assert st["items"][0]["name"] == "dock"
        assert any(c[0] == "stations" and "dock" in c[1] for c in be.calls)

        await a.send(type="goto_station", name="dock")
        ack = await a.ack("goto_station")
        assert ack["ok"] and "AUTO" in ack["msg"]  # mod MANUAL uyarısı
        assert ("goal", 1.0, 2.0, 0.5) in be.calls
        await a.send(type="goto_station", name="yok")
        assert not (await a.ack("goto_station"))["ok"]

        await a.send(type="goal", x=3, y=4, yaw=0)
        assert (await a.ack("goal"))["ok"]
        tel = await a.wait(lambda m: m["type"] == "telemetry" and m["nav"]["state"] == "active")
        assert tel["nav"]["goal"]["x"] == 3.0
        # estop aktif hedefi iptal eder
        await a.send(type="estop", active=True)
        await a.ack("estop")
        assert ("cancel",) in be.calls
        await a.send(type="cancel_goal")
        assert not (await a.ack("cancel_goal"))["ok"]

        await a.send(type="station_remove", name="dock")
        assert (await a.ack("station_remove"))["ok"]
        await a.send(type="save_map", name="fab1")
        ack = await a.ack("save_map")
        assert ack["ok"] and "fab1" in ack["msg"]
        await a.send(type="save_map", name="bad name!")
        assert not (await a.ack("save_map"))["ok"]

        await a.send(type="ping", t=12.5)
        assert (await a.typ("pong"))["t"] == 12.5
        await a.ws.send("garbage")
        assert (await a.ack("unknown"))["ok"] is False
        await a.ws.close()
        await srv.stop()
    run(main())


def test_streams_and_subscribe(mk):
    be, srv = mk()

    async def main():
        await srv.start()
        a = await _connect(srv.port)
        await a.typ("hello")
        m = await a.typ("map", timeout=4)
        assert m["width"] == 2 and m["png_b64"]
        sc = await a.typ("scan")
        assert sc["points"] == [[1.0, 0.0]]
        img = await a.typ("image")
        import base64
        assert base64.b64decode(img["jpeg_b64"]) == JPEG
        det = await a.typ("detections")
        assert det["items"][0]["label"] == "person"

        # image_fps=2 ile ~1 s'de en fazla ~3 kare
        await a.send(type="subscribe", image=True, image_fps=2, scan=False, map=False)
        await a.ack("subscribe")
        a.msgs.clear()
        t_end = time.monotonic() + 1.5
        n_img = n_scan = 0
        while time.monotonic() < t_end:
            try:
                raw = await asyncio.wait_for(a.ws.recv(), timeout=0.2)
            except asyncio.TimeoutError:
                continue
            t = json.loads(raw)["type"]
            n_img += t == "image"
            n_scan += t == "scan"
        assert 2 <= n_img <= 4, n_img
        assert n_scan <= 1  # en fazla kuyrukta kalmış bir tane

        await a.send(type="subscribe", image=False)
        await a.ack("subscribe")
        a.msgs.clear()
        await asyncio.sleep(0.2)
        t_end = time.monotonic() + 0.8
        while time.monotonic() < t_end:
            try:
                raw = await asyncio.wait_for(a.ws.recv(), timeout=0.2)
            except asyncio.TimeoutError:
                continue
            assert json.loads(raw)["type"] != "image"
        await a.ws.close()
        await srv.stop()
    run(main())


def test_auth_token(mk):
    be, srv = mk(token="s3cret", auth_timeout=1.0)

    async def main():
        await srv.start()
        bad = await _connect(srv.port)
        await bad.send(type="auth", token="wrong")
        ack = await bad.ack("auth")
        assert ack["ok"] is False
        with pytest.raises(websockets.exceptions.ConnectionClosed):
            await asyncio.wait_for(bad.ws.recv(), 2)

        noauth = await _connect(srv.port)
        await noauth.send(type="teleop", vx=0.1, wz=0)
        assert (await noauth.ack("auth"))["ok"] is False

        good = await _connect(srv.port)
        await good.send(type="auth", token="s3cret")
        assert (await good.ack("auth"))["ok"]
        assert (await good.typ("hello"))["role"] == "controller"  # yetkisizler controller olmadı
        await good.ws.close()
        await srv.stop()
    run(main())


def test_flood_protection(mk):
    be, srv = mk()

    async def main():
        await srv.start()
        a = await _connect(srv.port)
        await a.typ("hello")
        for i in range(300):
            await a.send(type="ping", t=i)
        log = await a.wait(lambda m: m["type"] == "log" and "hız" in m["msg"], timeout=3)
        assert log["level"] == "warn"
        await a.ws.close()
        await srv.stop()
    run(main())
