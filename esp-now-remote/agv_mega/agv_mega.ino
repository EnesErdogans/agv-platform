/*
 * AGV Kumanda - Motor Kontrol  |  Arduino Mega 2560
 *
 * RX ESP32 (GPIO25) --> Serial1 RX1 (D19)
 * Gelen satir: "$L,R,F*CC\n"   (L,R: -255..255, F: bit0 ARM, bit1 ESTOP, bit2 LINK)
 *
 * 2 x BTS7960 (IBT-2) surucu. Her surucu: RPWM, LPWM, R_EN, L_EN.
 *
 * GUVENLIK
 *  - 200 ms gecerli satir gelmezse  -> motorlar 0 (rampasiz, aninda)
 *  - ESTOP gelirse                   -> aninda 0, EN pinleri LOW, ARM kilitlenir
 *  - ESTOP sonrasi tekrar surmek icin: ESTOP birakilmali + ARM 0'a dusup tekrar 1 olmali
 *  - Normal surus rampali (200 kg yuk icin ani kalkis/fren dislileri ve sasiyi yorar)
 *
 * D13 LED:  sonuk = veri yok | yavas yanip sonme = bagli, ARM yok
 *           hizli yanip sonme = ESTOP kilitli | surekli yanik = ARM, suruse hazir
 */

// ---------------------------------------------------------------------------
// PINLER  (kendi kablolamana gore degistir)
// ---------------------------------------------------------------------------
// Sol surucu
#define L_RPWM  5    // ileri  (Timer3)
#define L_LPWM  6    // geri   (Timer4)
#define L_REN   22
#define L_LEN   23
// Sag surucu
#define R_RPWM  7    // ileri  (Timer4)
#define R_LPWM  8    // geri   (Timer4)
#define R_REN   24
#define R_LEN   25

#define LED_PIN 13

// ---------------------------------------------------------------------------
// AYARLAR
// ---------------------------------------------------------------------------
#define LINK_BAUD        115200
#define CMD_TIMEOUT_MS   200
#define RAMP_UP_PER_MS   0.6f    // hizlanma: 0 -> 255 yaklasik 425 ms
#define RAMP_DOWN_PER_MS 1.5f    // yavaslama: 255 -> 0 yaklasik 170 ms
#define MIN_PWM          0       // motor bu degerin altinda donmuyorsa artir (ornek 25)
#define INVERT_LEFT      false   // sol motor ters donuyorsa true yap
#define INVERT_RIGHT     false   // sag motor ters donuyorsa true yap
#define PWM_HIGH_FREQ    1       // 1: ~3.9 kHz (daha az vizilti), 0: varsayilan ~490 Hz

#define FLAG_ARM   0x01
#define FLAG_ESTOP 0x02
#define FLAG_LINK  0x04

// ---------------------------------------------------------------------------
// DURUM
// ---------------------------------------------------------------------------
int      targetL = 0, targetR = 0;
uint8_t  flags = 0;
uint32_t lastCmdMs = 0;
float    curL = 0, curR = 0;
bool     estopLatched = false;
bool     armSeenLow = true;     // ESTOP sonrasi ARM'in 0'a dustugunu gorduk mu

char     buf[40];
uint8_t  bufLen = 0;
bool     inFrame = false;

uint32_t goodFrames = 0, badFrames = 0;

// ---------------------------------------------------------------------------
void setupPwmFreq() {
#if PWM_HIGH_FREQ
  // Timer3 (D5) ve Timer4 (D6,D7,D8): prescaler 8 -> ~3.9 kHz
  TCCR3B = (TCCR3B & 0xF8) | 0x02;
  TCCR4B = (TCCR4B & 0xF8) | 0x02;
#endif
}

void driverEnable(bool en) {
  digitalWrite(L_REN, en); digitalWrite(L_LEN, en);
  digitalWrite(R_REN, en); digitalWrite(R_LEN, en);
}

void writeMotor(uint8_t pinFwd, uint8_t pinRev, int v, bool invert) {
  if (invert) v = -v;
  v = constrain(v, -255, 255);
  int mag = abs(v);
  if (mag > 0 && mag < MIN_PWM) mag = MIN_PWM;
  if (v > 0)      { analogWrite(pinRev, 0); analogWrite(pinFwd, mag); }
  else if (v < 0) { analogWrite(pinFwd, 0); analogWrite(pinRev, mag); }
  else            { analogWrite(pinFwd, 0); analogWrite(pinRev, 0); }
}

void stopNow() {
  curL = curR = 0;
  writeMotor(L_RPWM, L_LPWM, 0, false);
  writeMotor(R_RPWM, R_LPWM, 0, false);
}

// ---------------------------------------------------------------------------
// "$L,R,F*CC" cozumleme
// ---------------------------------------------------------------------------
uint8_t hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;
}

void handleFrame(char *s) {
  char *star = strchr(s, '*');
  if (!star || strlen(star) < 3) { badFrames++; return; }
  uint8_t hi = hexVal(star[1]), lo = hexVal(star[2]);
  if (hi == 0xFF || lo == 0xFF) { badFrames++; return; }
  uint8_t want = (hi << 4) | lo, cs = 0;
  for (char *p = s; p < star; p++) cs ^= (uint8_t)*p;
  if (cs != want) { badFrames++; return; }
  *star = '\0';

  int l, r, f;
  if (sscanf(s, "%d,%d,%d", &l, &r, &f) != 3) { badFrames++; return; }
  targetL = constrain(l, -255, 255);
  targetR = constrain(r, -255, 255);
  flags   = (uint8_t)f;
  lastCmdMs = millis();
  goodFrames++;
}

void readLink() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '$') { inFrame = true; bufLen = 0; continue; }
    if (!inFrame) continue;
    if (c == '\n' || c == '\r') {
      buf[bufLen] = '\0';
      handleFrame(buf);
      inFrame = false;
    } else if (bufLen < sizeof(buf) - 1) {
      buf[bufLen++] = c;
    } else {
      inFrame = false; badFrames++;
    }
  }
}

float rampTo(float cur, int target, float dtMs) {
  // ayni yonde buyume -> RAMP_UP, kuculme / yon degisimi -> RAMP_DOWN
  bool speedingUp = (target > 0 && cur >= 0 && target > cur) ||
                    (target < 0 && cur <= 0 && target < cur);
  float step = (speedingUp ? RAMP_UP_PER_MS : RAMP_DOWN_PER_MS) * dtMs;
  if (cur < target) return min(cur + step, (float)target);
  if (cur > target) return max(cur - step, (float)target);
  return cur;
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);       // USB debug
  Serial1.begin(LINK_BAUD);   // RX ESP32

  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(L_REN, OUTPUT);  pinMode(L_LEN, OUTPUT);
  pinMode(R_REN, OUTPUT);  pinMode(R_LEN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  setupPwmFreq();
  stopNow();
  driverEnable(false);
  Serial.println("Mega hazir");
}

void loop() {
  readLink();

  static uint32_t lastTick = millis(), lastDbg = 0;
  uint32_t now = millis();
  float dt = (float)(now - lastTick);
  if (dt < 5) return;           // ~200 Hz kontrol dongusu
  lastTick = now;

  bool fresh = (lastCmdMs != 0) && (now - lastCmdMs < CMD_TIMEOUT_MS);
  bool link  = fresh && (flags & FLAG_LINK);
  bool arm   = link && (flags & FLAG_ARM);
  bool estop = (flags & FLAG_ESTOP) && fresh;

  // ESTOP kilidi
  if (estop) { estopLatched = true; armSeenLow = false; }
  if (estopLatched && !estop) {
    if (!arm) armSeenLow = true;            // ARM'i birakti
    if (armSeenLow && arm) estopLatched = false;  // tekrar ARM verdi -> kilit acildi
  }

  bool drive = link && arm && !estopLatched;

  if (!drive) {
    stopNow();
    driverEnable(!estopLatched && link);    // ESTOP'ta surucu tamamen kapali
  } else {
    driverEnable(true);
    curL = rampTo(curL, targetL, dt);
    curR = rampTo(curR, targetR, dt);
    writeMotor(L_RPWM, L_LPWM, (int)curL, INVERT_LEFT);
    writeMotor(R_RPWM, R_LPWM, (int)curR, INVERT_RIGHT);
  }

  // LED
  bool led;
  if (!link)              led = false;
  else if (estopLatched)  led = (now / 100) % 2;
  else if (!arm)          led = (now / 500) % 2;
  else                    led = true;
  digitalWrite(LED_PIN, led);

  if (now - lastDbg >= 500) {
    lastDbg = now;
    Serial.print("link="); Serial.print(link);
    Serial.print(" arm="); Serial.print(arm);
    Serial.print(" estop="); Serial.print(estopLatched);
    Serial.print(" tgt="); Serial.print(targetL); Serial.print(','); Serial.print(targetR);
    Serial.print(" out="); Serial.print((int)curL); Serial.print(','); Serial.print((int)curR);
    Serial.print(" ok="); Serial.print(goodFrames);
    Serial.print(" bad="); Serial.println(badFrames);
  }
}
