/*
 * AGV Kumanda - TX (Verici)  |  YD-ESP32-23 (ESP32-S3)  |  COM6
 *
 * PC (Python) --USB Seri--> TX --ESP-NOW--> RX ESP32 --UART--> Arduino Mega
 *
 * PC'den gelen satir formati:  "L,R,F\n"
 *   L, R : -255..255   (sol / sag teker PWM)
 *   F    : bit0 = ARM (surus izni), bit1 = ESTOP (acil durdurma)
 *
 * PC'ye giden durum satiri (5 Hz):  "#S link=1 rssi=-52 pc=1"
 *   link : RX'ten son 300 ms icinde onay (ACK) geldi mi
 *   rssi : RX'in olctugu sinyal gucu (dBm)
 *   pc   : PC'den son 300 ms icinde komut geldi mi
 *
 * Guvenlik: PC'den 300 ms komut gelmezse RX'e 0,0 ve ARM=0 gonderilir.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// ---------------------------------------------------------------------------
// AYARLAR
// ---------------------------------------------------------------------------
static uint8_t RX_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // RX ESP32 MAC adresini yaz (RX acilinca Seri Monitor'de yazar)

#define WIFI_CHANNEL     1       // RX ile AYNI olmali
#define SERIAL_BAUD      115200
#define SEND_PERIOD_MS   20      // 50 Hz ESP-NOW gonderimi
#define PC_TIMEOUT_MS    300     // PC sessiz kalirsa dur
#define ACK_TIMEOUT_MS   300     // RX'ten onay gelmezse link=0
#define STATUS_PERIOD_MS 200     // PC'ye durum satiri

#define FLAG_ARM   0x01
#define FLAG_ESTOP 0x02
#define PKT_MAGIC  0xA5
#define ACK_MAGIC  0x5A

// ---------------------------------------------------------------------------
// PAKETLER (RX ile birebir ayni olmali)
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint16_t seq;
  int16_t  left;
  int16_t  right;
  uint8_t  flags;
  uint8_t  crc;
} ControlPacket;

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint16_t lastSeq;
  int8_t   rssi;
  uint8_t  crc;
} AckPacket;

static uint8_t calcCrc(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    c = (uint8_t)((c << 1) | (c >> 7));
  }
  return c;
}

// ---------------------------------------------------------------------------
// DURUM
// ---------------------------------------------------------------------------
static int16_t  cmdLeft = 0, cmdRight = 0;
static uint8_t  cmdFlags = 0;
static uint32_t lastPcMs = 0;
static uint16_t seq = 0;
static uint32_t sendErr = 0;

static volatile uint32_t lastAckMs = 0;
static volatile int8_t   ackRssi = 0;

static char    lineBuf[48];
static uint8_t lineLen = 0;

// ---------------------------------------------------------------------------
// ESP-NOW alma (RX'ten gelen ACK)
// Arduino-ESP32 3.x ile callback imzasi degisti, ikisi de destekleniyor.
// ---------------------------------------------------------------------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len != (int)sizeof(AckPacket)) return;
  AckPacket a;
  memcpy(&a, data, sizeof(a));
  if (a.magic != ACK_MAGIC) return;
  if (a.crc != calcCrc((const uint8_t *)&a, sizeof(a) - 1)) return;
  ackRssi = a.rssi;
  lastAckMs = millis();
}

// ---------------------------------------------------------------------------
// PC'den satir okuma
// ---------------------------------------------------------------------------
static void handleLine(char *s) {
  int l, r, f;
  if (sscanf(s, "%d,%d,%d", &l, &r, &f) == 3) {
    cmdLeft  = (int16_t)constrain(l, -255, 255);
    cmdRight = (int16_t)constrain(r, -255, 255);
    cmdFlags = (uint8_t)(f & (FLAG_ARM | FLAG_ESTOP));
    lastPcMs = millis();
  }
}

static void readPc() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;  // tasma: satiri at
    }
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("#ERR esp_now_init basarisiz");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RX_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("#ERR RX peer eklenemedi");
    while (true) delay(1000);
  }

  Serial.print("#TX hazir. Kendi MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  readPc();

  static uint32_t lastSend = 0, lastStatus = 0;
  uint32_t now = millis();

  if (now - lastSend >= SEND_PERIOD_MS) {
    lastSend = now;
    bool pcOk = (lastPcMs != 0) && (now - lastPcMs < PC_TIMEOUT_MS);

    ControlPacket p;
    p.magic = PKT_MAGIC;
    p.seq   = seq++;
    p.left  = pcOk ? cmdLeft  : 0;
    p.right = pcOk ? cmdRight : 0;
    p.flags = pcOk ? cmdFlags : 0;   // PC yoksa ARM duser
    p.crc   = calcCrc((const uint8_t *)&p, sizeof(p) - 1);

    if (esp_now_send(RX_MAC, (const uint8_t *)&p, sizeof(p)) != ESP_OK) sendErr++;
  }

  if (now - lastStatus >= STATUS_PERIOD_MS) {
    lastStatus = now;
    uint32_t ack = lastAckMs;
    uint32_t t = millis();
    bool link = (ack != 0) && (t - ack < ACK_TIMEOUT_MS);
    bool pcOk = (lastPcMs != 0) && (t - lastPcMs < PC_TIMEOUT_MS);
    Serial.printf("#S link=%d rssi=%d pc=%d err=%lu\n",
                  link ? 1 : 0, link ? (int)ackRssi : 0, pcOk ? 1 : 0,
                  (unsigned long)sendErr);
  }
}
