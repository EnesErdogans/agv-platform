/*
 * AGV Kumanda - RX (Alici)  |  ESP32 DevKitC V4 (ESP32-D0WD)
 *
 * TX ESP32 --ESP-NOW--> RX --UART2 (GPIO25 TX)--> Arduino Mega RX1 (D19)
 *
 * Mega'ya giden satir (50 Hz):  "$L,R,F*CC\n"
 *   L, R : -255..255
 *   F    : bit0 ARM, bit1 ESTOP, bit2 LINK (RX taze paket aliyor mu)
 *   CC   : '$' ile '*' arasindaki karakterlerin XOR'u (2 haneli hex)
 *
 * Guvenlik: 250 ms boyunca gecerli paket gelmezse Mega'ya 0,0 ve LINK=0 gider.
 * Her gecerli pakete TX'e ACK (RSSI ile) doner.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// ---------------------------------------------------------------------------
// AYARLAR
// ---------------------------------------------------------------------------
static uint8_t TX_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // TX ESP32-S3 MAC adresini yaz (TX acilinca Seri Monitor'de yazar)

#define WIFI_CHANNEL     1        // TX ile AYNI olmali
#define MEGA_TX_PIN      25       // ESP32 TX  -> Mega D19 (RX1)
#define MEGA_RX_PIN      26       // ESP32 RX  <- Mega D18 (TX1)  (gerilim bolucu sart!)
#define MEGA_BAUD        115200
#define FORWARD_MS       20       // Mega'ya 50 Hz
#define LINK_TIMEOUT_MS  250
#define LED_PIN          2        // DevKitC'de LED yoksa sorun degil

#define FLAG_ARM   0x01
#define FLAG_ESTOP 0x02
#define FLAG_LINK  0x04
#define PKT_MAGIC  0xA5
#define ACK_MAGIC  0x5A

// ---------------------------------------------------------------------------
// PAKETLER (TX ile birebir ayni)
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
// DURUM (callback ile loop arasinda paylasilan)
// ---------------------------------------------------------------------------
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int16_t  rxLeft = 0, rxRight = 0;
static volatile uint8_t  rxFlags = 0;
static volatile uint16_t rxSeq = 0;
static volatile int8_t   rxRssi = 0;
static volatile uint32_t lastPktMs = 0;
static volatile bool     ackPending = false;
static volatile uint32_t pktCount = 0, badCount = 0;

// ---------------------------------------------------------------------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
  int8_t rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  int8_t rssi = 0;  // 2.x cekirdekte RSSI bu callback'te yok
#endif
  if (memcmp(mac, TX_MAC, 6) != 0) { badCount++; return; }   // sadece bizim TX
  if (len != (int)sizeof(ControlPacket)) { badCount++; return; }

  ControlPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != PKT_MAGIC) { badCount++; return; }
  if (p.crc != calcCrc((const uint8_t *)&p, sizeof(p) - 1)) { badCount++; return; }

  portENTER_CRITICAL(&mux);
  rxLeft    = constrain(p.left,  -255, 255);
  rxRight   = constrain(p.right, -255, 255);
  rxFlags   = p.flags & (FLAG_ARM | FLAG_ESTOP);
  rxSeq     = p.seq;
  rxRssi    = rssi;
  lastPktMs = millis();
  ackPending = true;
  pktCount++;
  portEXIT_CRITICAL(&mux);
}

// ---------------------------------------------------------------------------
static void sendToMega(int16_t l, int16_t r, uint8_t f) {
  char body[24];
  int n = snprintf(body, sizeof(body), "%d,%d,%u", l, r, f);
  uint8_t cs = 0;
  for (int i = 0; i < n; i++) cs ^= (uint8_t)body[i];
  char line[32];
  snprintf(line, sizeof(line), "$%s*%02X\n", body, cs);
  Serial2.print(line);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN);
  pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERR esp_now_init basarisiz");
    while (true) { sendToMega(0, 0, 0); delay(20); }
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, TX_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);   // ACK gondermek icin

  Serial.print("RX hazir. Kendi MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  static uint32_t lastFwd = 0, lastDbg = 0;
  uint32_t now = millis();

  // ACK'i callback disinda gonder (callback icinde esp_now_send onerilmez)
  bool doAck = false;
  AckPacket a;
  portENTER_CRITICAL(&mux);
  if (ackPending) {
    ackPending = false;
    doAck = true;
    a.lastSeq = rxSeq;
    a.rssi    = rxRssi;
  }
  portEXIT_CRITICAL(&mux);
  if (doAck) {
    a.magic = ACK_MAGIC;
    a.crc   = calcCrc((const uint8_t *)&a, sizeof(a) - 1);
    esp_now_send(TX_MAC, (const uint8_t *)&a, sizeof(a));
  }

  if (now - lastFwd >= FORWARD_MS) {
    lastFwd = now;
    int16_t l, r; uint8_t f; uint32_t last;
    portENTER_CRITICAL(&mux);
    l = rxLeft; r = rxRight; f = rxFlags; last = lastPktMs;
    portEXIT_CRITICAL(&mux);

    bool link = (last != 0) && (millis() - last < LINK_TIMEOUT_MS);
    if (link) {
      sendToMega(l, r, f | FLAG_LINK);
    } else {
      sendToMega(0, 0, 0);   // baglanti yok: dur, ARM yok
    }
    digitalWrite(LED_PIN, link ? HIGH : ((now / 250) % 2));
  }

  if (now - lastDbg >= 1000) {
    lastDbg = now;
    Serial.printf("pkt/s=%lu bad=%lu L=%d R=%d F=%u rssi=%d\n",
                  (unsigned long)pktCount, (unsigned long)badCount,
                  rxLeft, rxRight, rxFlags, rxRssi);
    pktCount = 0;
  }
}
