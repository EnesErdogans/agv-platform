/*
 * AGV-UGV Fleet Studio — ESP32 Wi-Fi (UDP) araç kontrolcüsü
 *
 * Panel -> ESP32 (port 4210):  {"t":"cmd","v":0.50,"vy":0,"w":0.20}     v: m/s, w: rad/s
 * ESP32 -> Panel (port 4211):  {"t":"tel","bat":7.9}                    (5 Hz; enkoder varsa "enc":[sol,sag] ekleyin)
 *
 * Panelde: Bağlantı tipi = UDP, Araç IP = ESP32'nin IP'si, Araç portu 4210, Dinleme portu 4211.
 * Hiç telemetri göndermeyecekseniz panelde "Telemetri yok" kutusunu işaretleyin.
 * 500 ms komut gelmezse motorlar durur. ESP32 Arduino çekirdeği 2.x / 3.x ile derlenir.
 */
#include <WiFi.h>
#include <WiFiUdp.h>

const char *WIFI_SSID = "AG_ADI";
const char *WIFI_PASS = "AG_SIFRESI";
const uint16_t CMD_PORT = 4210, TEL_PORT = 4211;

const float TRACK_WIDTH_M = 0.16;     // iz genişliği
const float MAX_WHEEL_MPS = 0.50;     // tam PWM'de teker hızı (ölçüp yazın)

// DRV8833 / L298N / TB6612: her motor için iki PWM girişi
const int L_A = 25, L_B = 26, R_A = 27, R_B = 14;
const int VBAT_PIN = 34;              // gerilim bölücü (isteğe bağlı)
const float VBAT_DIVIDER = 3.0;

WiFiUDP udp;
IPAddress panelIp; bool havePanel = false;
unsigned long lastCmdMs = 0, lastTelMs = 0;
char pkt[160];

void motor(int a, int b, float u) {            // u: -1..1
  int pwm = (int)(constrain(fabs(u), 0.0, 1.0) * 255);
  analogWrite(a, u > 0 ? pwm : 0); analogWrite(b, u < 0 ? pwm : 0);
}
void setTwist(float v, float w) {
  motor(L_A, L_B, (v - w * TRACK_WIDTH_M / 2) / MAX_WHEEL_MPS);
  motor(R_A, R_B, (v + w * TRACK_WIDTH_M / 2) / MAX_WHEEL_MPS);
}
float field(const char *s, const char *key) {  // çok basit JSON alan okuyucu: "key":sayi
  const char *p = strstr(s, key); if (!p) return 0;
  p = strchr(p, ':'); return p ? atof(p + 1) : 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(L_A, OUTPUT); pinMode(L_B, OUTPUT); pinMode(R_A, OUTPUT); pinMode(R_B, OUTPUT);
  setTwist(0, 0);
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print('.'); }
  Serial.print("\nIP: "); Serial.println(WiFi.localIP());
  udp.begin(CMD_PORT);
}

void loop() {
  int n = udp.parsePacket();
  if (n > 0) {
    int len = udp.read(pkt, sizeof(pkt) - 1); pkt[max(len, 0)] = 0;
    if (strstr(pkt, "\"cmd\"")) {
      setTwist(field(pkt, "\"v\""), field(pkt, "\"w\""));
      lastCmdMs = millis(); panelIp = udp.remoteIP(); havePanel = true;
    }
  }
  unsigned long now = millis();
  if (now - lastCmdMs > 500) setTwist(0, 0);     // güvenlik zaman aşımı
  if (havePanel && now - lastTelMs >= 200) {
    lastTelMs = now;
    float vbat = analogRead(VBAT_PIN) * (3.3 / 4095.0) * VBAT_DIVIDER;
    udp.beginPacket(panelIp, TEL_PORT);
    udp.printf("{\"t\":\"tel\",\"bat\":%.2f}", vbat);
    udp.endPacket();
  }
}
