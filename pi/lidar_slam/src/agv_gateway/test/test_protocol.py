import base64
import io
import json
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(__file__), "..")))
from agv_gateway import protocol as P  # noqa: E402


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


# ------------------------------------------------------------------ parse / authorize
def test_parse_teleop_clamps():
    m = P.parse_client_message('{"type":"teleop","vx":5,"wz":-9}', max_lin=0.8, max_ang=1.0)
    assert m == {"type": "teleop", "vx": 0.8, "wz": -1.0}


@pytest.mark.parametrize("raw,cmd", [
    ("not json", "unknown"),
    ("[1,2]", "unknown"),
    ('{"type":"fly"}', "fly"),
    ('{"type":"teleop","vx":"1","wz":0}', "teleop"),
    ('{"type":"teleop","vx":true,"wz":0}', "teleop"),
    ('{"type":"teleop","vx":NaN,"wz":0}', "teleop"),
    ('{"type":"set_mode","mode":"ESTOP"}', "set_mode"),
    ('{"type":"estop","active":1}', "estop"),
    ('{"type":"goal","x":1}', "goal"),
    ('{"type":"station_add","name":"","x":0,"y":0}', "station_add"),
    ('{"type":"save_map","name":"../etc"}', "save_map"),
    ('{"type":"subscribe","image":"yes"}', "subscribe"),
])
def test_parse_errors(raw, cmd):
    with pytest.raises(P.ProtocolError) as e:
        P.parse_client_message(raw)
    assert e.value.cmd == cmd


def test_parse_binary_rejected():
    with pytest.raises(P.ProtocolError):
        P.parse_client_message(b"\x00")


def test_parse_valid_messages():
    assert P.parse_client_message('{"type":"goal","x":1,"y":2}')["yaw"] == 0.0
    assert P.parse_client_message('{"type":"subscribe","image_fps":50}')["image_fps"] == 10
    assert P.parse_client_message('{"type":"subscribe","image":false}') == {"type": "subscribe", "image": False}
    assert P.parse_client_message('{"type":"ping","t":1.5}') == {"type": "ping", "t": 1.5}
    assert P.parse_client_message('{"type":"station_add","name":" dock ","x":1,"y":2,"yaw":3}')["name"] == "dock"
    assert P.parse_client_message('{"type":"save_map","name":"fabrika_1"}')["name"] == "fabrika_1"


def test_authorize_rules():
    V, C = P.ROLE_VIEWER, P.ROLE_CONTROLLER
    assert P.authorize({"type": "estop", "active": True}, V)[0]
    assert not P.authorize({"type": "estop", "active": False}, V)[0]
    assert P.authorize({"type": "estop", "active": False}, C)[0]
    for t in ("teleop", "set_mode", "goal", "goto_station", "station_add", "station_remove", "save_map"):
        assert not P.authorize({"type": t}, V)[0]
        assert P.authorize({"type": t}, C)[0]
    for t in ("subscribe", "ping", "request_control", "release_control"):
        assert P.authorize({"type": t}, V)[0]


# ------------------------------------------------------------------ builders
def test_hello_and_ack():
    assert P.msg_hello("viewer") == {"type": "hello", "version": 1, "robot": "agv200", "role": "viewer"}
    assert P.msg_ack("goal", True, "x") == {"type": "ack", "cmd": "goal", "ok": True, "msg": "x"}
    assert P.msg_log("bogus", "m")["level"] == "info"


def test_telemetry_schema():
    st = {"mode": "AUTO", "estop": False, "obstacle_stop": True, "min_front_dist": 0.512345,
          "teleop_source": "app", "battery_v": 24.123}
    t = P.msg_telemetry(1.0, st, None, (1, 2, 0.5), (0.3, 0.1), (0.01, 0.02, 0.5, 0.1),
                        "active", (3, 4, 0), frame="map")
    s = P.dumps(t)  # NaN olmamalı
    d = json.loads(s)
    assert d["type"] == "telemetry" and d["mode"] == "AUTO" and d["obstacle_stop"] is True
    assert d["min_front_dist"] == 0.512 and d["battery_v"] == 24.12
    assert d["pose"] == {"x": 1.0, "y": 2.0, "yaw": 0.5}
    assert d["vel"] == {"vx": 0.3, "wz": 0.1}
    assert set(d["imu"]) == {"roll", "pitch", "yaw", "gz"}
    assert d["nav"] == {"state": "active", "goal": {"x": 3.0, "y": 4.0, "yaw": 0.0}}
    assert d["teleop_source"] == "app"


def test_telemetry_defaults_without_status():
    d = P.msg_telemetry(1.0, None, "ESTOP", None, None, None, "idle", None)
    assert d["mode"] == "ESTOP" and d["estop"] is True and d["pose"] is None
    assert d["battery_v"] is None and d["min_front_dist"] is None
    assert d["nav"]["goal"] is None
    d2 = P.msg_telemetry(1.0, {"mode": "MANUAL", "battery_v": None, "min_front_dist": float("inf")},
                         None, None, None, None, "idle", None)
    assert d2["battery_v"] is None and d2["min_front_dist"] is None
    P.dumps(d2)


def test_detections_json_parse():
    s = '[{"label":"person","score":0.9,"bbox":[0.1,0.2,0.3,0.4]},{"bad":1}]'
    assert P.parse_detections_json(s) == [{"label": "person", "score": 0.9, "bbox": [0.1, 0.2, 0.3, 0.4]}]
    assert P.parse_detections_json("garbage") == []


# ------------------------------------------------------------------ math
def test_quat_to_euler():
    qx, qy, qz, qw = P.yaw_to_quat(1.2)
    r, p, y = P.quat_to_euler(qx, qy, qz, qw)
    assert r == pytest.approx(0) and p == pytest.approx(0) and y == pytest.approx(1.2)
    # 90° roll
    s = math.sin(math.pi / 4)
    r, p, y = P.quat_to_euler(s, 0, 0, s)
    assert r == pytest.approx(math.pi / 2)


def test_scan_to_points_transform_and_decimate():
    # 4 ışın: 0, 90, 180, 270 derece, hepsi 1 m; lazer (1,0)'da 90° dönük
    pts = P.scan_to_points([1, 1, 1, 1], 0.0, math.pi / 2, 0.1, 10.0, (1.0, 0.0, math.pi / 2))
    assert pts[0] == pytest.approx((1.0, 1.0))
    assert pts[1] == pytest.approx((0.0, 0.0))
    # geçersizler atılır
    pts = P.scan_to_points([float("inf"), float("nan"), 0.0, 20.0, 1.0], 0, 0.1, 0.1, 10.0)
    assert len(pts) == 1
    # seyreltme
    pts = P.scan_to_points([2.0] * 1440, -math.pi, 2 * math.pi / 1440, 0.1, 10.0)
    assert len(pts) <= 360 and len(pts) >= 300
    assert P.scan_to_points([], 0, 0.1, 0.1, 10) == []


# ------------------------------------------------------------------ map PNG
def test_map_png_orientation_and_values():
    from PIL import Image
    # 3 genişlik x 2 yükseklik. grid satır 0 (en düşük y) = [0, 100, -1], satır 1 (yüksek y) = [100, 50, 0]
    data = [0, 100, -1, 100, 50, 0]
    b64 = P.map_to_png_b64(3, 2, data)
    img = Image.open(io.BytesIO(base64.b64decode(b64)))
    assert img.mode == "L" and img.size == (3, 2)
    import numpy as np
    px = np.asarray(img).ravel().tolist()
    # PNG satır 0 (üst) = grid satır 1 (en yüksek y)
    assert px[:3] == [0, 205, 254]
    assert px[3:] == [254, 0, 205]


def test_map_png_size_mismatch():
    with pytest.raises(ValueError):
        P.occupancy_to_gray(3, 3, [0] * 8)


def test_map_png_large_roundtrip():
    import numpy as np
    from PIL import Image
    rng = np.random.default_rng(1)
    w, h = 400, 300
    data = rng.choice([-1, 0, 100], size=w * h).tolist()
    img = np.asarray(Image.open(io.BytesIO(base64.b64decode(P.map_to_png_b64(w, h, data)))))
    grid = np.array(data).reshape(h, w)
    expected = np.where(grid < 0, 205, np.where(grid >= 65, 0, 254))[::-1]
    assert (img == expected).all()


def test_map_msg_and_fingerprint():
    m = P.msg_map(1, 0.05, 3, 2, (-1.0, -2.0, 0.0), "AAAA")
    assert m["origin"] == {"x": -1.0, "y": -2.0, "yaw": 0.0} and m["type"] == "map"
    a = P.map_fingerprint(3, 2, 0.05, (0, 0, 0), [0, 1, 2, 3, 4, 5])
    b = P.map_fingerprint(3, 2, 0.05, (0, 0, 0), [0, 1, 2, 3, 4, 6])
    assert a != b


# ------------------------------------------------------------------ limiters / watchdog
def test_rate_limiter():
    clk = FakeClock()
    rl = P.RateLimiter(clk)
    assert rl.allow("a", 10)
    assert not rl.allow("a", 10)
    clk.t += 0.1
    assert rl.allow("a", 10)
    assert rl.allow("b", 10)
    assert not rl.allow("c", 0)


def test_token_bucket():
    clk = FakeClock()
    tb = P.TokenBucket(rate=10, burst=3, clock=clk)
    assert all(tb.take() for _ in range(3))
    assert not tb.take()
    clk.t += 0.11
    assert tb.take()


def test_teleop_watchdog_fires_once():
    clk = FakeClock()
    wd = P.TeleopWatchdog(0.5, clk)
    assert not wd.tick()
    wd.feed()
    clk.t += 0.4
    assert not wd.tick()
    clk.t += 0.2
    assert wd.tick()
    assert not wd.tick()  # sadece bir kez
    wd.feed()
    assert wd.stop() and not wd.stop()


# ------------------------------------------------------------------ nav / arbiter
def test_nav_tracker_preempt():
    n = P.NavTracker()
    t1 = n.start((1, 1, 0))
    t2 = n.start((2, 2, 0))
    assert n.finished(t1, P.GOAL_STATUS_CANCELED) is None  # eski hedef
    assert n.state == "active"
    assert n.finished(t2, P.GOAL_STATUS_SUCCEEDED) == "succeeded"
    t3 = n.start((3, 3, 0))
    assert n.finished(t3, P.GOAL_STATUS_ABORTED) == "failed"
    t4 = n.start((4, 4, 0))
    n.canceled_locally()
    assert n.state == "canceled"
    assert n.finished(t4, P.GOAL_STATUS_CANCELED) is None
    t5 = n.start((5, 5, 0))
    assert n.rejected(t5) and n.state == "failed"


def test_arbiter():
    a = P.ControlArbiter(allow_takeover=False)
    assert a.join(1) == "controller"
    assert a.join(2) == "viewer"
    assert a.request(2)[0] is False
    assert a.release(2)[0] is False
    assert a.release(1)[0] is True
    assert a.request(2)[0] is True and a.role(2) == "controller"
    assert a.leave(2) is True and a.controller is None
    assert a.join(3) == "controller"
    b = P.ControlArbiter(allow_takeover=True)
    b.join(1)
    b.join(2)
    ok, prev, _ = b.request(2)
    assert ok and prev == 1 and b.role(1) == "viewer"


def test_client_settings():
    s = P.ClientSettings()
    s.apply({"image": False, "image_fps": 3})
    assert s.effective_image_fps == 0 and s.image_fps == 3
    s.apply({"image": True})
    assert s.effective_image_fps == 3


# ------------------------------------------------------------------ stations
def test_station_store(tmp_path):
    path = tmp_path / "sub" / "stations.yaml"
    s = P.StationStore(str(path))
    assert s.items() == []
    s.add("dock", 1, 2, 3)
    s.add("line_a", -1, 0.5, 7.0)  # yaw normalize edilir
    s.add("dock", 5, 5, 0)  # günceller
    s2 = P.StationStore(str(path))
    assert [x["name"] for x in s2.items()] == ["dock", "line_a"]
    assert s2.get("dock")["x"] == 5
    assert abs(s2.get("line_a")["yaw"]) < math.pi
    assert json.loads(s2.to_json())[0]["name"] == "dock"
    assert s2.remove("dock") and not s2.remove("dock")
    assert P.StationStore(str(path)).get("dock") is None


def test_station_store_tolerates_garbage(tmp_path):
    path = tmp_path / "st.yaml"
    path.write_text("stations:\n  - {name: ok, x: 1, y: 2}\n  - {name: bad}\n")
    s = P.StationStore(str(path))
    assert s.items() == [{"name": "ok", "x": 1.0, "y": 2.0, "yaw": 0.0}]
