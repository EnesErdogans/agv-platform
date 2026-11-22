#!/usr/bin/env python3
"""ws_cli_test — gateway için küçük tezgah test istemcisi (ROS gerektirmez, sadece `websockets`).

Örnekler:
  python3 ws_cli_test.py --url ws://192.168.1.50:8765                 # hello + telemetri yazdır
  python3 ws_cli_test.py --teleop 0.2 0.0 --duration 3                # 3 s boyunca 10 Hz teleop
  python3 ws_cli_test.py --estop on        /  --estop off --mode MANUAL
  python3 ws_cli_test.py --goal 1.0 2.0 0  /  --goto dock  /  --cancel
  python3 ws_cli_test.py --interactive                                # w/a/s/d/x/space, e=estop, r=serbest, q=çık
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time


def summarize(m: dict) -> str:
    t = m.get("type")
    if t == "telemetry":
        p = m.get("pose") or {}
        return (f"TEL mode={m.get('mode')} estop={m.get('estop')} obs={m.get('obstacle_stop')} "
                f"front={m.get('min_front_dist')} bat={m.get('battery_v')} "
                f"pose=({p.get('x')},{p.get('y')},{p.get('yaw')}) vel={m.get('vel')} nav={m.get('nav', {}).get('state')}")
    if t == "image":
        return f"IMAGE {len(m.get('jpeg_b64', '')) * 3 // 4} B"
    if t == "map":
        return f"MAP {m.get('width')}x{m.get('height')} res={m.get('resolution')} png={len(m.get('png_b64', ''))} b64"
    if t == "scan":
        return f"SCAN {len(m.get('points', []))} nokta frame={m.get('frame')}"
    return json.dumps(m, ensure_ascii=False)


async def run(a):
    import websockets

    async with websockets.connect(a.url, max_size=8 << 20) as ws:
        if a.token:
            await ws.send(json.dumps({"type": "auth", "token": a.token}))
        await ws.send(json.dumps({"type": "subscribe", "image": a.image, "image_fps": a.image_fps,
                                  "scan": a.scan, "map": a.map}))
        counts: dict = {}
        last_print: dict = {}

        async def reader():
            async for raw in ws:
                m = json.loads(raw)
                t = m.get("type")
                counts[t] = counts.get(t, 0) + 1
                periodic = t in ("telemetry", "image", "scan", "detections")
                if periodic and time.monotonic() - last_print.get(t, 0) < a.print_period:
                    continue
                last_print[t] = time.monotonic()
                print(summarize(m), flush=True)

        rt = asyncio.ensure_future(reader())
        await asyncio.sleep(0.3)

        if a.request_control:
            await ws.send(json.dumps({"type": "request_control"}))
        if a.estop is not None:
            await ws.send(json.dumps({"type": "estop", "active": a.estop == "on"}))
        if a.mode:
            await ws.send(json.dumps({"type": "set_mode", "mode": a.mode}))
        if a.goal:
            await ws.send(json.dumps({"type": "goal", "x": a.goal[0], "y": a.goal[1], "yaw": a.goal[2]}))
        if a.goto:
            await ws.send(json.dumps({"type": "goto_station", "name": a.goto}))
        if a.cancel:
            await ws.send(json.dumps({"type": "cancel_goal"}))
        if a.ping:
            await ws.send(json.dumps({"type": "ping", "t": time.time()}))

        if a.interactive:
            await interactive(ws, a)
        elif a.teleop:
            vx, wz = a.teleop
            t_end = time.monotonic() + a.duration
            while time.monotonic() < t_end:
                await ws.send(json.dumps({"type": "teleop", "vx": vx, "wz": wz}))
                await asyncio.sleep(0.1)
            print("teleop bitti (gateway 0.5 s içinde sıfırlayacak)")
            await asyncio.sleep(1.0)
        else:
            await asyncio.sleep(a.duration)
        rt.cancel()
        print("alınan mesaj sayıları:", counts)


async def interactive(ws, a):
    import termios
    import tty

    keys = {"w": (a.speed, 0.0), "s": (-a.speed, 0.0), "a": (0.0, a.turn), "d": (0.0, -a.turn),
            "x": (0.0, 0.0), " ": (0.0, 0.0)}
    cmd = [0.0, 0.0]
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    loop = asyncio.get_running_loop()
    q: asyncio.Queue = asyncio.Queue()
    loop.add_reader(fd, lambda: q.put_nowait(sys.stdin.read(1)))
    print("w/s ileri-geri, a/d dön, x/boşluk dur, e=ESTOP, r=ESTOP bırak, m=MANUAL, o=AUTO, q=çık")
    try:
        tty.setcbreak(fd)
        while True:
            try:
                k = await asyncio.wait_for(q.get(), timeout=0.1)
            except asyncio.TimeoutError:
                k = None
            if k == "q":
                break
            if k in keys:
                cmd = list(keys[k])
            elif k == "e":
                await ws.send(json.dumps({"type": "estop", "active": True}))
                cmd = [0.0, 0.0]
            elif k == "r":
                await ws.send(json.dumps({"type": "estop", "active": False}))
            elif k == "m":
                await ws.send(json.dumps({"type": "set_mode", "mode": "MANUAL"}))
            elif k == "o":
                await ws.send(json.dumps({"type": "set_mode", "mode": "AUTO"}))
            await ws.send(json.dumps({"type": "teleop", "vx": cmd[0], "wz": cmd[1]}))
    finally:
        loop.remove_reader(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(argv=None):
    p = argparse.ArgumentParser(description="AGV gateway WebSocket test istemcisi")
    p.add_argument("--url", default="ws://127.0.0.1:8765")
    p.add_argument("--token", default="")
    p.add_argument("--duration", type=float, default=5.0)
    p.add_argument("--print-period", type=float, default=1.0, help="periyodik mesajları en fazla bu sıklıkta yaz")
    p.add_argument("--image", action="store_true", help="görüntü akışını aç (varsayılan kapalı)")
    p.add_argument("--image-fps", type=int, default=2)
    p.add_argument("--scan", action="store_true")
    p.add_argument("--map", action="store_true")
    p.add_argument("--request-control", action="store_true")
    p.add_argument("--teleop", nargs=2, type=float, metavar=("VX", "WZ"))
    p.add_argument("--estop", choices=["on", "off"])
    p.add_argument("--mode", choices=["MANUAL", "AUTO"])
    p.add_argument("--goal", nargs=3, type=float, metavar=("X", "Y", "YAW"))
    p.add_argument("--goto")
    p.add_argument("--cancel", action="store_true")
    p.add_argument("--ping", action="store_true")
    p.add_argument("--interactive", action="store_true")
    p.add_argument("--speed", type=float, default=0.2)
    p.add_argument("--turn", type=float, default=0.5)
    a = p.parse_args(argv)
    try:
        asyncio.run(run(a))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
