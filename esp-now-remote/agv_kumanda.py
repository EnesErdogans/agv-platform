"""
AGV Kumanda - PC tarafi
Gamepad (veya klavye) -> USB Seri (COM6) -> TX ESP32 -> ESP-NOW -> RX ESP32 -> Mega

Kurulum:  pip install pygame pyserial
Calistir: python agv_kumanda.py              (varsayilan COM6)
          python agv_kumanda.py --port COM7
          python agv_kumanda.py --test       (seri port olmadan, sadece ekranda)
          python agv_kumanda.py --map        (kolun eksen/tus numaralarini gormek icin)

KONTROLLER (gamepad)
  Sol analog yukari/asagi : ileri / geri
  Sag analog sag/sol      : donus
  START (Options/Menu)    : ARM ac/kapat   (ESTOP sonrasi: once kilidi acar, bir daha basinca ARM)
  B  (PS: Daire)          : ACIL STOP
  RB / LB  (R1 / L1)      : hiz kademesi arttir / azalt  (%25 - %50 - %75 - %100)

KONTROLLER (klavye, pencere aktifken)
  W/S veya Yukari/Asagi   : ileri / geri
  A/D veya Sol/Sag        : donus
  Enter                   : ARM ac/kapat
  Bosluk                  : ACIL STOP
  1 2 3 4                 : hiz kademesi
  Esc                     : cikis (once durur)
"""

import argparse
import sys
import time

import pygame

try:
    import serial
except ImportError:
    serial = None

# ---------------------------------------------------------------------------
# AYARLAR
# ---------------------------------------------------------------------------
DEFAULT_PORT = "COM6"
BAUD = 115200
SEND_HZ = 50
DEADZONE = 0.10          # analog olu bolge
EXPO = 0.35              # 0 = dogrusal, 1 = cok yumusak merkez
TURN_GAIN = 0.60         # donus hassasiyeti (yerinde donus icin)
SPEED_LEVELS = [0.25, 0.50, 0.75, 1.00]
START_LEVEL = 0          # ilk acilista %25 (200 kg arac - dikkat!)

# Klasik joystick API'si icin yedek eslesme (Xbox duzeni). --map ile kontrol et.
JOY_AXIS_THROTTLE = 1    # sol analog Y
JOY_AXIS_TURN = 2        # sag analog X (bazi kollarda 3)
JOY_BTN_ESTOP = 1        # B
JOY_BTN_ARM = 7          # Start
JOY_BTN_SPEED_DN = 4     # LB
JOY_BTN_SPEED_UP = 5     # RB

FLAG_ARM = 0x01
FLAG_ESTOP = 0x02

# ---------------------------------------------------------------------------
# SDL GameController API: Xbox / PS4 / PS5 / cogu kol icin standart eslesme
# ---------------------------------------------------------------------------
try:
    from pygame._sdl2 import controller as sdl_ctrl
    HAS_SDL_CTRL = True
except Exception:  # eski pygame
    sdl_ctrl = None
    HAS_SDL_CTRL = False


def apply_deadzone_expo(v: float) -> float:
    if abs(v) < DEADZONE:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    v = (abs(v) - DEADZONE) / (1.0 - DEADZONE)
    v = (1 - EXPO) * v + EXPO * v ** 3
    return s * min(v, 1.0)


def arcade_mix(throttle: float, turn: float):
    """Tank surus karisimi. throttle, turn: -1..1 -> sol, sag: -1..1"""
    turn *= TURN_GAIN
    left = throttle + turn
    right = throttle - turn
    m = max(1.0, abs(left), abs(right))
    return left / m, right / m


class Pad:
    """Gamepad'i tek arayuzle okur (once GameController, olmazsa Joystick)."""

    def __init__(self):
        self.ctrl = None
        self.joy = None
        self.name = "yok"
        self.connect()

    def connect(self):
        self.ctrl = self.joy = None
        self.name = "yok"
        if pygame.joystick.get_count() == 0:
            return
        if HAS_SDL_CTRL:
            sdl_ctrl.init()
            for i in range(pygame.joystick.get_count()):
                if sdl_ctrl.is_controller(i):
                    self.ctrl = sdl_ctrl.Controller(i)
                    self.name = f"{self.ctrl.name} (standart eslesme)"
                    return
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.name = f"{self.joy.get_name()} (joystick eslesmesi)"

    @property
    def ok(self):
        return self.ctrl is not None or self.joy is not None

    def axes(self):
        """(throttle, turn) -1..1, ileri pozitif"""
        if self.ctrl is not None:
            ly = self.ctrl.get_axis(pygame.CONTROLLER_AXIS_LEFTY) / 32767.0
            rx = self.ctrl.get_axis(pygame.CONTROLLER_AXIS_RIGHTX) / 32767.0
            return -ly, rx
        if self.joy is not None:
            n = self.joy.get_numaxes()
            ly = self.joy.get_axis(JOY_AXIS_THROTTLE) if n > JOY_AXIS_THROTTLE else 0.0
            rx = self.joy.get_axis(JOY_AXIS_TURN) if n > JOY_AXIS_TURN else 0.0
            return -ly, rx
        return 0.0, 0.0

    def button_event(self, ev):
        """Olaydan anlamli komut cikar: 'arm', 'estop', 'up', 'down' veya None"""
        if self.ctrl is not None and ev.type == pygame.CONTROLLERBUTTONDOWN:
            return {
                pygame.CONTROLLER_BUTTON_START: "arm",
                pygame.CONTROLLER_BUTTON_B: "estop",
                pygame.CONTROLLER_BUTTON_RIGHTSHOULDER: "up",
                pygame.CONTROLLER_BUTTON_LEFTSHOULDER: "down",
            }.get(ev.button)
        if self.joy is not None and ev.type == pygame.JOYBUTTONDOWN:
            return {
                JOY_BTN_ARM: "arm",
                JOY_BTN_ESTOP: "estop",
                JOY_BTN_SPEED_UP: "up",
                JOY_BTN_SPEED_DN: "down",
            }.get(ev.button)
        return None


class Link:
    """TX ESP32 ile seri baglanti, otomatik yeniden baglanir."""

    def __init__(self, port, test=False):
        self.port = port
        self.test = test
        self.ser = None
        self.rx_buf = b""
        self.status = {"link": 0, "rssi": 0, "pc": 0}
        self.last_status_t = 0.0
        self.last_try = 0.0
        self.err = ""

    def ensure(self):
        if self.test or self.ser is not None:
            return
        if time.time() - self.last_try < 1.0:
            return
        self.last_try = time.time()
        if serial is None:
            self.err = "pyserial yok: pip install pyserial"
            return
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0, write_timeout=0.05)
            self.err = ""
        except Exception as e:
            self.ser = None
            self.err = f"{self.port} acilamadi: {e}"

    def send(self, left, right, flags):
        line = f"{left},{right},{flags}\n".encode()
        if self.test or self.ser is None:
            return
        try:
            self.ser.write(line)
        except Exception as e:
            self.err = f"yazma hatasi: {e}"
            self.close()

    def poll(self):
        if self.test or self.ser is None:
            return
        try:
            data = self.ser.read(256)
        except Exception as e:
            self.err = f"okuma hatasi: {e}"
            self.close()
            return
        if not data:
            return
        self.rx_buf += data
        while b"\n" in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b"\n", 1)
            text = raw.decode(errors="ignore").strip()
            if text.startswith("#S"):
                for part in text[2:].split():
                    if "=" in part:
                        k, v = part.split("=", 1)
                        try:
                            self.status[k] = int(v)
                        except ValueError:
                            pass
                self.last_status_t = time.time()
            elif text.startswith("#"):
                print("[TX]", text)
        if len(self.rx_buf) > 1024:
            self.rx_buf = b""

    @property
    def link_ok(self):
        return (time.time() - self.last_status_t < 1.0) and self.status.get("link", 0) == 1

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


def map_mode():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("Kol bulunamadi. USB/Bluetooth baglantisini kontrol et.")
        return
    j = pygame.joystick.Joystick(0)
    j.init()
    print(f"Kol: {j.get_name()}  eksen={j.get_numaxes()} tus={j.get_numbuttons()}")
    print("Cubuklari oynat / tuslara bas. Ctrl+C ile cik.")
    try:
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.JOYBUTTONDOWN:
                    print(f"TUS {ev.button}")
                elif ev.type == pygame.JOYAXISMOTION and abs(ev.value) > 0.5:
                    print(f"EKSEN {ev.axis} = {ev.value:+.2f}")
                elif ev.type == pygame.JOYHATMOTION:
                    print(f"HAT {ev.hat} = {ev.value}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description="AGV gamepad kumanda")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--test", action="store_true", help="seri port olmadan calis")
    ap.add_argument("--map", action="store_true", help="kol tus/eksen numaralarini goster")
    args = ap.parse_args()

    if args.map:
        map_mode()
        return

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((520, 330))
    pygame.display.set_caption("AGV Kumanda")
    font = pygame.font.SysFont("consolas", 20)
    big = pygame.font.SysFont("consolas", 34, bold=True)
    clock = pygame.time.Clock()

    pad = Pad()
    link = Link(args.port, test=args.test)

    armed = False
    estop = False
    level = START_LEVEL
    left_pwm = right_pwm = 0
    running = True

    def press(cmd):
        nonlocal armed, estop, level
        if cmd == "estop":
            estop, armed = True, False
        elif cmd == "arm":
            if estop:
                estop = False          # kilit acildi, ARM hala kapali
            else:
                armed = not armed
        elif cmd == "up":
            level = min(level + 1, len(SPEED_LEVELS) - 1)
        elif cmd == "down":
            level = max(level - 1, 0)

    print(f"Kol: {pad.name}")
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED,
                             getattr(pygame, "CONTROLLERDEVICEADDED", -1),
                             getattr(pygame, "CONTROLLERDEVICEREMOVED", -2)):
                armed = False   # kol degisti -> guvenli tarafta kal
                pad.connect()
                print(f"Kol: {pad.name}")
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_SPACE:
                    press("estop")
                elif ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    press("arm")
                elif pygame.K_1 <= ev.key <= pygame.K_4:
                    level = ev.key - pygame.K_1
            else:
                cmd = pad.button_event(ev)
                if cmd:
                    press(cmd)

        # --- girisler
        throttle, turn = pad.axes()
        throttle = apply_deadzone_expo(throttle)
        turn = apply_deadzone_expo(turn)

        keys = pygame.key.get_pressed()
        kt = (keys[pygame.K_w] or keys[pygame.K_UP]) - (keys[pygame.K_s] or keys[pygame.K_DOWN])
        kr = (keys[pygame.K_d] or keys[pygame.K_RIGHT]) - (keys[pygame.K_a] or keys[pygame.K_LEFT])
        if kt or kr:                     # klavye kullaniliyorsa onu al
            throttle, turn = float(kt), float(kr)

        l, r = arcade_mix(throttle, turn)
        scale = 255 * SPEED_LEVELS[level]
        if armed and not estop:
            left_pwm, right_pwm = int(round(l * scale)), int(round(r * scale))
        else:
            left_pwm = right_pwm = 0

        flags = (FLAG_ARM if armed else 0) | (FLAG_ESTOP if estop else 0)

        link.ensure()
        link.send(left_pwm, right_pwm, flags)
        link.poll()

        # --- ekran
        screen.fill((20, 22, 26))
        if estop:
            state, col = "ACIL STOP", (230, 60, 60)
        elif armed:
            state, col = "ARM - SURUS", (60, 210, 90)
        else:
            state, col = "BEKLEMEDE", (230, 180, 50)
        screen.blit(big.render(state, True, col), (20, 15))

        if args.test:
            ltxt, lcol = "TEST MODU (seri yok)", (150, 150, 150)
        elif link.ser is None:
            ltxt, lcol = link.err or "seri port bekleniyor", (230, 60, 60)
        elif link.link_ok:
            ltxt, lcol = f"AGV bagli  RSSI {link.status.get('rssi', 0)} dBm", (60, 210, 90)
        else:
            ltxt, lcol = "AGV'den cevap yok (RX kapali / menzil disi)", (230, 60, 60)

        lines = [
            (f"Port : {args.port}", (200, 200, 200)),
            (f"Link : {ltxt}", lcol),
            (f"Kol  : {pad.name[:40]}", (200, 200, 200)),
            (f"Hiz  : %{int(SPEED_LEVELS[level] * 100)}", (200, 200, 200)),
            (f"Sol  : {left_pwm:+4d}    Sag : {right_pwm:+4d}", (255, 255, 255)),
            ("START/Enter=ARM  B/Bosluk=STOP  RB-LB/1-4=hiz", (130, 130, 130)),
        ]
        y = 75
        for txt, c in lines:
            screen.blit(font.render(txt, True, c), (20, y))
            y += 34
        pygame.display.flip()
        clock.tick(SEND_HZ)

    # cikarken guvenli durus
    for _ in range(10):
        link.send(0, 0, FLAG_ESTOP)
        time.sleep(0.02)
    link.close()
    pygame.quit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
