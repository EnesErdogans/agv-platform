/*
 * AGV-UGV Fleet Studio — Arduino Mega 2560 motor kontrolcüsü (diferansiyel / paletli)
 *
 * Seri protokol (115200 baud, satır sonu '\n'):
 *   PC / Pi -> Mega :  V,<v_mm_s>,<w_mrad_s>   hız komutu (ileri hız, dönüş hızı)
 *                      S                       dur
 *                      R                       enkoder sayaçlarını sıfırla
 *   Mega -> PC / Pi :  T,<encL>,<encR>,<vbat_mV>,<bumper>,<us_mm>   (20 Hz)
 *
 * 500 ms komut gelmezse motorlar durur (bağlantı koparsa araç kaçmaz).
 *
 * ENKODERSİZ kullanım (ör. 37 mm DC motor + L298N): HAS_ENCODERS 0 yapın.
 * Bu durumda hız açık çevrim PWM'e çevrilir ve panelde aracın
 * "Tik / tur" değeri 0 olmalıdır (konum komuttan tahmin edilir).
 *
 * Pinleri ve mekanik sabitleri kendi aracınıza göre düzenleyin.
 */

#define HAS_ENCODERS   1      // 0: enkoder yok (açık çevrim)
#define HAS_ULTRASONIC 0      // 1: ön HC-SR04 takılı
#define HAS_BUMPER     0      // 1: tampon anahtarı takılı (GND'ye çeker)

// ---- Mekanik sabitler (paneldeki Tasarım sekmesiyle aynı olmalı) ----
const float WHEEL_RADIUS_M = 0.100;   // teker / palet tamburu yarıçapı
const float TRACK_WIDTH_M  = 0.500;   // iz genişliği
const long  TICKS_PER_REV  = 2048;    // teker turu başına enkoder tiki (redüktör dahil)
const float MAX_WHEEL_MPS  = 1.30;    // tam PWM'de teker çizgisel hızı (açık çevrim ölçeği)

// ---- Pinler: L298N / BTS7960 benzeri (PWM + iki yön pini) ----
const uint8_t L_PWM = 5,  L_IN1 = 22, L_IN2 = 23;
const uint8_t R_PWM = 6,  R_IN1 = 24, R_IN2 = 25;
const uint8_t ENC_L_A = 2, ENC_L_B = 4;    // A kanalı kesme pini (Mega: 2, 3, 18, 19, 20, 21)
const uint8_t ENC_R_A = 3, ENC_R_B = 7;
const uint8_t US_TRIG = 30, US_ECHO = 31;
const uint8_t BUMPER_PIN = 32;
const uint8_t VBAT_PIN = A0;               // gerilim bölücü çıkışı
const float   VBAT_DIVIDER = 11.0;         // (R1 + R2) / R2  ör. 100k + 10k

// ---- PI hız kontrolü (yalnız enkoderli) ----
const float KP = 180.0, KI = 420.0;        // PWM / (m/s), PWM / (m/s·s)

volatile long encL = 0, encR = 0;
float targetL = 0, targetR = 0;            // m/s
float integL = 0, integR = 0;
long  lastEncL = 0, lastEncR = 0;
unsigned long lastCmdMs = 0, lastCtrlMs = 0, lastTelMs = 0;
char buf[48]; uint8_t len = 0;

void isrL() { if (digitalRead(ENC_L_B)) encL--; else encL++; }
void isrR() { if (digitalRead(ENC_R_B)) encR++; else encR--; }

void drive(uint8_t pwmPin, uint8_t in1, uint8_t in2, int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(in1, pwm > 0); digitalWrite(in2, pwm < 0);
  analogWrite(pwmPin, abs(pwm));
}

void setTwist(float v, float w) {          // v: m/s, w: rad/s
  targetL = v - w * TRACK_WIDTH_M / 2.0;
  targetR = v + w * TRACK_WIDTH_M / 2.0;
  lastCmdMs = millis();
}

void handleLine(char *s) {
  if (s[0] == 'V') {
    char *p = strchr(s, ','); if (!p) return;
    long v = atol(p + 1); p = strchr(p + 1, ','); if (!p) return;
    long w = atol(p + 1);
    setTwist(v / 1000.0, w / 1000.0);
  } else if (s[0] == 'S') { setTwist(0, 0); integL = integR = 0; }
  else if (s[0] == 'R') { noInterrupts(); encL = encR = 0; interrupts(); lastEncL = lastEncR = 0; }
}

long readUltrasonicMm() {
#if HAS_ULTRASONIC
  digitalWrite(US_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH); delayMicroseconds(10); digitalWrite(US_TRIG, LOW);
  unsigned long us = pulseIn(US_ECHO, HIGH, 12000UL);   // ~2 m zaman aşımı
  return us ? (long)(us * 0.1715) : 0;                  // 0 = yankı yok
#else
  return 0;
#endif
}

void setup() {
  Serial.begin(115200);
  pinMode(L_PWM, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
#if HAS_ENCODERS
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);
#endif
#if HAS_ULTRASONIC
  pinMode(US_TRIG, OUTPUT); pinMode(US_ECHO, INPUT);
#endif
#if HAS_BUMPER
  pinMode(BUMPER_PIN, INPUT_PULLUP);
#endif
  Serial.println(F("AGV-UGV Mega hazir"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (len) { buf[len] = 0; handleLine(buf); len = 0; } }
    else if (len < sizeof(buf) - 1) buf[len++] = c;
  }

  unsigned long now = millis();
  if (now - lastCmdMs > 500) { targetL = targetR = 0; integL = integR = 0; }   // güvenlik zaman aşımı

  if (now - lastCtrlMs >= 20) {                                                // 50 Hz kontrol
    float dt = (now - lastCtrlMs) / 1000.0; lastCtrlMs = now;
    int pwmL, pwmR;
#if HAS_ENCODERS
    noInterrupts(); long eL = encL, eR = encR; interrupts();
    const float mPerTick = 2.0 * PI * WHEEL_RADIUS_M / TICKS_PER_REV;
    float vL = (eL - lastEncL) * mPerTick / dt, vR = (eR - lastEncR) * mPerTick / dt;
    lastEncL = eL; lastEncR = eR;
    float errL = targetL - vL, errR = targetR - vR;
    integL = constrain(integL + errL * dt, -0.5, 0.5); integR = constrain(integR + errR * dt, -0.5, 0.5);
    pwmL = (int)(targetL / MAX_WHEEL_MPS * 255 + KP * errL + KI * integL);
    pwmR = (int)(targetR / MAX_WHEEL_MPS * 255 + KP * errR + KI * integR);
    if (targetL == 0 && targetR == 0) { pwmL = pwmR = 0; integL = integR = 0; }
#else
    pwmL = (int)(targetL / MAX_WHEEL_MPS * 255);
    pwmR = (int)(targetR / MAX_WHEEL_MPS * 255);
#endif
#if HAS_BUMPER
    if (!digitalRead(BUMPER_PIN)) { if (pwmL > 0) pwmL = 0; if (pwmR > 0) pwmR = 0; }  // temas: ileri gitme
#endif
    drive(L_PWM, L_IN1, L_IN2, pwmL); drive(R_PWM, R_IN1, R_IN2, pwmR);
  }

  if (now - lastTelMs >= 50) {                                                 // 20 Hz telemetri
    lastTelMs = now;
    noInterrupts(); long eL = encL, eR = encR; interrupts();
    long vbat = (long)(analogRead(VBAT_PIN) * (5000.0 / 1023.0) * VBAT_DIVIDER);
    int bumper = 0;
#if HAS_BUMPER
    bumper = !digitalRead(BUMPER_PIN);
#endif
    Serial.print(F("T,")); Serial.print(eL); Serial.print(','); Serial.print(eR); Serial.print(',');
    Serial.print(vbat); Serial.print(','); Serial.print(bumper);
#if HAS_ULTRASONIC
    Serial.print(','); Serial.print(readUltrasonicMm());
#endif
    Serial.print('\n');
  }
}
