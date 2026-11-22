# AGV Kumanda – Kurulum Notları

```
Gamepad ─USB/BT─> PC (agv_kumanda.py) ─COM6─> TX ESP32-S3 ─ESP-NOW─> RX ESP32 ─UART2─> Mega ─> 2x BTS7960
                                       <─ durum (link, RSSI) ───────── ACK ──┘
```

## Kablolama

**RX ESP32 → Mega**
| ESP32 | Mega | Not |
|---|---|---|
| GPIO25 (TX2) | D19 (RX1) | Direkt bağlanır (3.3V çıkışı Mega 5V'ta HIGH okunur) |
| GPIO26 (RX2) | D18 (TX1) | **Opsiyonel.** Bağlayacaksan araya gerilim bölücü koy (ör. 1k + 2k), Mega 5V basar, ESP32 3.3V'luk |
| GND | GND | **Mutlaka ortak toprak** |

**Mega → BTS7960** (pinler `agv_mega.ino` başında, değiştirilebilir)
| | RPWM (ileri) | LPWM (geri) | R_EN | L_EN |
|---|---|---|---|---|
| Sol | D5 | D6 | D22 | D23 |
| Sağ | D7 | D8 | D24 | D25 |

BTS7960'ların VCC'si Mega 5V'a, GND'leri Mega GND'ye. Motor beslemesi (B+/B−) ayrı batarya hattından.

## Yükleme sırası
1. **RX** (`agv_rx`): Kart = *ESP32 Dev Module*. Yükle, Seri Monitör'de `RX hazir. Kendi MAC: XX:XX:...` görmelisin. Bu adresi `agv_tx.ino` içindeki `RX_MAC` dizisine yaz.
2. **TX** (`agv_tx`): Önce TX'i bir kez yükleyip Seri Monitör'deki MAC adresini `agv_rx.ino` içindeki `TX_MAC`'e yaz ve RX'i tekrar yükle. Kart = *ESP32S3 Dev Module*, **USB CDC On Boot: Enabled** (USB portundan bağlıysan). COM6.
   Seri Monitör'de `#S link=1 rssi=-40 ...` gelmesi lazım (RX açıkken).
3. **Mega** (`agv_mega`): Kart = *Arduino Mega 2560*.
4. Seri Monitör'leri **kapat**, sonra: `pip install pygame pyserial` → `python agv_kumanda.py`

## İlk test (TEKERLER HAVADA!)
1. Aracı takoza al, tekerler yere değmesin.
2. Python'u aç → ekranda "AGV bagli" yeşil olmalı.
3. START → ARM. Sol analog ileri: iki teker de ileri dönmeli.
   - Bir teker ters dönüyorsa: `INVERT_LEFT` / `INVERT_RIGHT` = `true`.
   - Sağ çubuk sağa: sol teker ileri, sağ teker geri olmalı. Ters ise `TURN_GAIN` işaretini değiştir (`-0.60`).
4. B tuşu → anında durmalı, Mega'nın D13 LED'i hızlı yanıp sönmeli.
5. RX ESP32'nin fişini çek → motorlar 0.2 sn içinde durmalı.
6. Python'u kapat → motorlar durmalı.

## Güvenlik katmanları
| Ne olursa | Kim durdurur | Süre |
|---|---|---|
| Python kapanır / kol çıkar | Python ESTOP gönderir / ARM düşer | anında |
| USB kablosu çıkar | TX | 300 ms |
| ESP-NOW kopar / menzil dışı | RX | 250 ms |
| RX–Mega kablosu kopar | Mega | 200 ms |
| B tuşu / Boşluk | Mega kilitler, BTS EN pinleri LOW | anında |

ESTOP kilidini açmak: START (kilit açılır, araç hâlâ beklemede) → START tekrar (ARM).

## Ayar noktaları
- `agv_kumanda.py`: `SPEED_LEVELS`, `START_LEVEL` (ilk açılış %25), `DEADZONE`, `EXPO`, `TURN_GAIN`
- `agv_mega.ino`: `RAMP_UP_PER_MS`, `RAMP_DOWN_PER_MS` (200 kg için kalkış rampası), `MIN_PWM`, `PWM_HIGH_FREQ`
- Kol tuşları yanlış eşleşirse: `python agv_kumanda.py --map` ile numaraları gör, dosyanın başındaki `JOY_*` değerlerini düzelt.
- Kanal: TX ve RX'te `WIFI_CHANNEL` aynı olmalı (varsayılan 1).

> **Fiziksel acil stop:** Yazılımsal stop'a ek olarak motor batarya hattına mantar tipi acil stop butonu/kontaktör koy. 200 kg'lık araçta bu şart.
