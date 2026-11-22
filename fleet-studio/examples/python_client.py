#!/usr/bin/env python3
"""
AGV-UGV Fleet Studio WebSocket API — Python örneği

Kendi algoritmanı Python'da yazıp simüle (ya da gerçek) araca karşı dene.
Bu örnek LiDAR ile engelden kaçan basit bir gezgin çalıştırır.

    pip install websockets
    python3 python_client.py agv200

Araç panelde "Manuel" modda olmalıdır (komutlar manuel sürüş kanalından gider;
0.6 sn komut gelmezse araç durur).
"""
import asyncio
import json
import math
import sys

import websockets

URL = "ws://127.0.0.1:8765/ws"
VEHICLE = sys.argv[1] if len(sys.argv) > 1 else "agv200"


def sector_min(lidar, lo_deg, hi_deg):
    """Aracın önüne göre [lo, hi] derece aralığındaki en yakın mesafe (m)."""
    best = float("inf")
    for i, r in enumerate(lidar["ranges"]):
        if r <= lidar["rangeMin"]:
            continue                      # 0 = dönüş yok
        a = lidar["theta"] + lidar["angleMin"] + i * lidar["angleInc"]
        a = math.degrees(math.atan2(math.sin(a), math.cos(a)))
        if lo_deg <= a <= hi_deg:
            best = min(best, r)
    return best


async def main():
    async with websockets.connect(URL, max_size=None) as ws:
        await ws.send(json.dumps({"op": "mode", "id": VEHICLE, "mode": "manual"}))
        lidar = None
        async for raw in ws:
            msg = json.loads(raw)
            if msg["op"] != "state":
                continue
            me = next((v for v in msg["vehicles"] if v["id"] == VEHICLE), None)
            if me is None:
                continue
            if "lidar" in me["sens"]:     # LiDAR yalnızca yeni tarama geldiğinde gönderilir
                lidar = me["sens"]["lidar"]
            if lidar is None:
                continue

            front = sector_min(lidar, -25, 25)
            left, right = sector_min(lidar, 25, 90), sector_min(lidar, -90, -25)
            if front < 0.9:
                v, w = 0.0, (1.2 if left > right else -1.2)
            else:
                v, w = min(0.8, 0.3 + 0.25 * front), 0.15 * (min(left, 5) - min(right, 5))
            await ws.send(json.dumps({"op": "teleop", "id": VEHICLE, "v": v, "w": w}))
            print(f"\rx={me['x']:6.2f} y={me['y']:6.2f}  ön={front:5.2f} m  v={v:4.2f} w={w:5.2f}", end="")


if __name__ == "__main__":
    asyncio.run(main())
