#!/usr/bin/env python3
"""
Raspberry Pi köprüsü: AGV-UGV Fleet Studio (UDP)  <->  Arduino Mega (USB seri)

ROS kullanmadan, Pi üzerinden aracı panelden sürmek için. Pi'de çalıştırın:

    pip install pyserial
    python3 pi_udp_serial_bridge.py --serial /dev/ttyACM0

Panelde araç: Bağlantı tipi = UDP, Araç IP = Pi'nin adı/IP'si (ör. agv.local),
Araç portu = 4210, Dinleme portu = 4211.

Mega'da firmware/arduino_mega_agv çalışmalıdır. Enkodersiz araçta Mega "T" satırında
enkoderi 0 gönderir; panelde aracın "Tik / tur" değerini 0 yapın.
"""
import argparse
import json
import socket
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--port", type=int, default=4210, help="komutların dinlendiği UDP portu")
    ap.add_argument("--tel-port", type=int, default=4211, help="panelin dinlediği UDP portu")
    ap.add_argument("--no-encoders", action="store_true", help="enkoder alanını gönderme")
    args = ap.parse_args()

    ser = serial.Serial(args.serial, args.baud, timeout=0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    sock.setblocking(False)
    print(f"Köprü hazır: UDP :{args.port} <-> {args.serial} @ {args.baud}")

    panel = None
    buf = b""
    last_tel = 0.0
    tel = {"t": "tel"}
    while True:
        # Panel -> Mega
        try:
            data, addr = sock.recvfrom(512)
            msg = json.loads(data.decode())
            if msg.get("t") == "cmd":
                panel = (addr[0], args.tel_port)
                v = int(float(msg.get("v", 0)) * 1000)
                w = int(float(msg.get("w", 0)) * 1000)
                ser.write(f"V,{v},{w}\n".encode())
        except BlockingIOError:
            pass
        except (ValueError, UnicodeDecodeError):
            pass

        # Mega -> Panel
        buf += ser.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            parts = line.decode(errors="ignore").strip().split(",")
            if len(parts) >= 3 and parts[0] == "T":
                try:
                    nums = [int(x) for x in parts[1:]]
                except ValueError:
                    continue
                tel = {"t": "tel"}
                if not args.no_encoders:
                    tel["enc"] = nums[0:2]
                if len(nums) > 2 and nums[2] > 0:
                    tel["bat"] = nums[2] / 1000.0
                if len(nums) > 3:
                    tel["bumper"] = nums[3]
                if len(nums) > 4:
                    tel["us"] = [n / 1000.0 for n in nums[4:]]

        now = time.time()
        if panel and now - last_tel >= 0.05:      # 20 Hz (aynı zamanda "bağlıyım" sinyali)
            last_tel = now
            sock.sendto(json.dumps(tel).encode(), panel)
        time.sleep(0.005)


if __name__ == "__main__":
    main()
