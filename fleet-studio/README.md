# AGV-UGV Fleet Studio

AGV (iç lojistik) ve UGV (arazi) araçların için **simülatör + gerçek araç kontrol paneli**.
Aynı haritada simüle ve gerçek araçlar birlikte çalışır; aynı arayüz, aynı görev sistemi,
aynı kod API'si ikisini de sürer.

## Kurulum

| Platform | Dosya | Not |
|---|---|---|
| Windows 10/11 (x64) | `AGV-UGV-Fleet-Studio-Setup-1.1.0.exe` | İmzasız olduğu için SmartScreen "Yine de çalıştır" isteyebilir |
| Linux (x64) | `AGV-UGV-Fleet-Studio-1.1.0.AppImage` | `chmod +x` yapıp çalıştır |
| Kaynak koddan | `baslat.bat` / `./baslat.sh` | Node.js 18+ gerekir; ilk çalıştırmada `npm install` yapar |

Arayüzsüz (sunucu) kullanım: `npm run server` → tarayıcıda `http://127.0.0.1:8765`.
Ağdaki tablet / telefondan açmak için `node server/index.js --lan` (bkz. Güvenlik).

Kurulum dosyasını kendin üretmek için: `npm run dist:win` (Linux / macOS / Windows'ta çalışır, wine gerektirmez)
ve `npm run dist:linux`.

## Neler var

**Simülasyon**
- Sürüş modelleri: diferansiyel, paletli / skid-steer, Ackermann, mecanum. İvme sınırı, motor gecikmesi,
  yüke bağlı dinamik (200 kg yükle fren mesafesi uzar), palet dönüş kayması.
- Sensörler: 2B LiDAR (menzil, FOV, ışın sayısı, gürültü, Hz), ultrasonik, kamera (görüş alanındaki istasyon /
  araç / engel tespitleri), şerit (çizgi) sensörü, enkoder, IMU + pusula, GPS, tampon.
- Enkodersiz araç: "Tik / tur = 0" → odometri komuttan açık çevrim hesaplanır ve gerçek konumdan sapar
  (haritadaki kesikli çerçeve = odometri hayaleti).
- Saha: fabrika içi (duvar, raf, istasyon, şerit, hareketli engel) + dış arazi (çim, çakıl, kum, çamur).
  Zemin hızı, kaymayı ve enerji tüketimini etkiler; "Arazi kabiliyeti" yüksek araç daha az etkilenir.
  Kapalı alanda GPS fix vermez.
- Batarya: zemine, yüke ve hıza bağlı tüketim; şarj istasyonunda dolum.
- Simülasyon hızı 0.5×–8×, duraklat / sıfırla.

**Kontrol paneli**
- Filo listesi, canlı harita (LiDAR, yol, iz, sensör katmanları), olay günlüğü, filo geneli ve araç başı acil durdurma.
- Manuel: ekran joystick'i + klavye (W A S D, mecanum için Q E).
- Otonom: A* yol planlama (zemin maliyetli), pure-pursuit takip, LiDAR / ultrasonik güvenlik katmanı
  (fren mesafesine göre yavaşlama, durma), yol kapanınca yeniden planlama.
- Görev: istasyona git, bekle, yük al / bırak; döngü.
- Sensörler sekmesi: telemetri, LiDAR kutupsal görünüm, hız grafiği, tüm sensör değerleri.
- Tasarım sekmesi: boyut, kütle, teker, hız / ivme, batarya, enkoder, sensör yerleşimi. Anında hesap:
  teker devri, gerekli tork, fren mesafesi, enkoder çözünürlüğü, tahmini çalışma süresi. JSON içe / dışa aktarım.
- Harita düzenleyici: duvar, engel (Shift = hareketli), istasyon / devriye noktası / şarj, şerit, zemin bölgesi.
- LiDAR haritalama: Sensörler sekmesinden başlat, aracı gezdir; taramalar doluluk ızgarasına (5 cm/hücre) işlenir.
  **Saha** katmanını kapatınca yalnızca aracın çıkardığı harita görünür. Konum kaynağı olarak gerçek konum ya da
  odometri seçilebilir (odometride kayma haritayı bozar; SLAM'in neden gerektiğini gösterir). Çıktı: PGM + YAML
  (ROS 2 `map_server` / Nav2 biçimi) ve PNG. Gerçek araçta da çalışır (sürücüden gelen LaserScan + konum ile).
- Çoklu harita: üst çubuktaki listeden harita seç; **Harita ▾** menüsünden sıfırdan yeni harita oluştur (kapalı alan /
  açık arazi / düz), çoğalt, yeniden adlandır, boyutunu değiştir, içeriğini temizle, sil, JSON içe / dışa aktar.
  Her harita araçların kendi başlangıç konumlarını ve gizli araç listesini ayrı tutar.
- Araç yönetimi: filo kartındaki **Gizle** aracı o haritada devre dışı bırakır (simüle edilmez, çizilmez, engel sayılmaz),
  **Sil** filodan kaldırır. Filo başlığındaki **⋯**: tümünü göster / gizle, yalnız gerçek araçları bırak, tümünü sil.

**Kod testi** (üç yol)
1. **Kod sekmesi** — JavaScript betiği: `loop(agv, dt)` 20 Hz çağrılır. 7 hazır örnek (çizgi izleyen, duvar takibi,
   GPS devriye, kamera ile takip…). Aynı betik gerçek araçta da çalışır.
2. **WebSocket API** — herhangi bir dilden: `examples/python_client.py`.
3. **ROS 2** — simüle aracı rosbridge üzerinden `/scan`, `/odom`, `/tf`, `/cmd_vel` olarak açar; Nav2,
   slam_toolbox ya da kendi düğümün simülasyona karşı çalışır. Bkz. `ros2/README.md`.

**Gerçek araç sürücüleri**

| Sürücü | Araç tarafı | Dosya |
|---|---|---|
| ROS 2 (rosbridge) | Raspberry Pi + ROS 2 Humble (Docker) | `ros2/` |
| Seri port | Arduino Mega, USB ile bu bilgisayara bağlı | `firmware/arduino_mega_agv/` |
| UDP | ESP32 (Wi-Fi) | `firmware/esp32_udp_agv/` |
| UDP | Raspberry Pi → USB → Arduino Mega (ROS'suz) | `examples/pi_udp_serial_bridge.py` |

Gerçek araç eklemek: **+ Araç** → "GERÇEK — …" şablonu → **Tasarım** sekmesinde ölçüleri gir (teker yarıçapı,
iz genişliği, tik / tur odometri için kullanılır) → **Bağlantı** → **Bağlan** → üstteki **Konum** aracıyla aracın
haritadaki yerini bir kez işaretle.

## Betik API'si

```js
function setup(agv) { }           // isteğe bağlı, bir kez
function loop(agv, dt) { }        // 20 Hz; agv / ugv / robot aynı nesne
```

| Okuma | |
|---|---|
| `pose`, `odom` | `{x, y, theta}` — konum kestirimi / ham odometri |
| `velocity` | `{v, vy, w}` |
| `lidar` | `{ranges[], angleMin, angleInc, range, rangeMin}`; `0` = dönüş yok |
| `lidarMin(a, b)` | aracın önüne göre a°..b° diliminde en yakın mesafe |
| `ultrasonic[]`, `line`, `camera[]`, `imu`, `gps`, `encoders`, `bumper` | sensörler |
| `battery`, `payload`, `terrain`, `stations`, `navigating`, `time`, `spec` | durum |

| Komut | |
|---|---|
| `setVelocity(v, w, vy)` | m/s, rad/s |
| `goTo('A')` / `goTo(x, y, theta)` | dahili planlayıcıyla git |
| `stop()`, `setPayload(kg)`, `log(...)` | |

Güvenlik katmanı betik komutlarına da uygulanır (Tasarım → Güvenlik'ten kapatılabilir).

## WebSocket API (`ws://127.0.0.1:8765/ws`)

```jsonc
// istemci → sunucu
{"op":"teleop","id":"agv200","v":0.4,"w":0.2,"vy":0}   // Manuel modda; 0.6 sn'de bir yenilenmeli
{"op":"mode","id":"agv200","mode":"manual|auto|script"}
{"op":"goto","id":"agv200","station":"B"}              // ya da "x","y","theta"
{"op":"mission","id":"agv200","mission":{"loop":true,"steps":[{"type":"goto","station":"A"},{"type":"wait","sec":5}]}}
{"op":"estop","id":"all","on":true}
// sunucu → istemci (15 Hz)
{"op":"state","t":12.3,"vehicles":[{"id":"agv200","x":..,"y":..,"th":..,"v":..,"soc":..,"sens":{"lidar":{..},"us":[..],"gps":{..}}}]}
```

## Proje yapısı

```
server/
  index.js            HTTP + WebSocket sunucusu
  fleet.js            araç yöneticisi: modlar, görev, güvenlik katmanı, betik çalıştırıcı
  sim/                harita, geometri
  drivers/sim.js      simüle araç: kinematik, sensör modelleri, batarya
  drivers/real.js     rosbridge / seri / UDP sürücüleri, ROS'a açma
  nav/                A* planlayıcı, pure-pursuit takipçi
web/                  arayüz (saf HTML / JS, bağımlılık yok)
electron/main.js      masaüstü kabuğu
config/defaults.js    varsayılan saha, filo ve şablonlar
firmware/ ros2/ examples/
test/run.js           otomatik testler (npm test)
```

Proje (tüm haritalar + filo + betikler) **Kaydet** ile kullanıcı veri klasöründeki `fleet.json` dosyasına yazılır.

## Sınırlar ve dürüst notlar

- Simülasyon 2B ve kinematik tabanlıdır: eğim, süspansiyon, devrilme, teker-zemin temas fiziği yoktur.
  Zemin etkisi hız / kayma / enerji katsayılarıyla modellenir. Kontrol mantığı, navigasyon ve sensör işleme
  kodu için uygundur; motor seçimi gibi kararlar için değerler ilk tahmindir.
- LiDAR haritalama, konumu bilinen araçla haritalamadır (mapping with known poses); tarama eşleştirme ya da döngü
  kapatma yapan bir SLAM değildir. Gerçek araçta harita kalitesi konum kestiriminin kalitesi kadardır.
- Kamera görüntü üretmez; görüş alanındaki nesneleri tespit listesi olarak verir (AI kamera çıktısına benzer).
- Simülasyonda otonom sürüş gerçek konumu kullanır (lokalizasyon var sayılır). Gerçek araçta konum odometriden
  ya da verdiğin `pose` konusundan (AMCL) gelir; odometri kayar.
- Sürücüler ve firmware örnekleri sahte uçlarla (test rosbridge / UDP) doğrulandı; **gerçek donanımda denenmedi**.
  İlk denemeyi tekerler havadayken, düşük hız sınırıyla yap.
- Paneldeki acil durdurma yazılımsaldır; araçta donanımsal acil stop butonunun yerini tutmaz.
- Güvenlik: sunucu varsayılan olarak yalnızca bu bilgisayardan erişilebilir. `--lan` ile ağa açarsan ağdaki
  herkes araçları sürebilir ve betik çalıştırabilir; yalnızca güvendiğin ağda kullan.
