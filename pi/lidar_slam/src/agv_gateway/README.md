# agv_gateway

Araç üstünde WebSocket sunucusu (`ws://<agv_ip>:8765`), yer istasyonu ROS'suz bağlanır. Protokol: `INTERFACES.md` §7.

## Çalıştırma
```bash
sudo apt install python3-websockets      # Docker imajında da gerekli (ros-humble-nav2-msgs, tf2-ros ile birlikte)
ros2 launch agv_gateway gateway.launch.py                 # şifresiz
ros2 launch agv_gateway gateway.launch.py token:=gizli    # istemci önce {"type":"auth","token":"gizli"} gönderir
```
Parametreler `config/gateway.yaml`: `port, host, token, teleop_timeout(0.5), max_lin_vel, max_ang_vel, telemetry_hz(10),
scan_hz(5), map_period(2.0), stations_file(~/.agv/stations.yaml), map_dir(~/.agv/maps), map_frame, odom_frame,
base_frame, nav_action(navigate_to_pose), allow_takeover(true), estop_cancels_goal(true), status_timeout(2.0)`.

## Davranış özeti
- **Roller:** controller yokken bağlanan controller olur, diğerleri viewer. `request_control` (allow_takeover=true ise
  devralır, eski controller'a `hello role=viewer` gider), `release_control`. Controller ayrılırsa kimse otomatik terfi etmez, teleop sıfırlanır.
- **Teleop:** sadece controller → `/cmd_vel_teleop` + `/agv/teleop_source`="app". 0.5 s gelmezse **bir kez** sıfır yayınlanır.
  Hızlar `max_lin_vel/max_ang_vel` ile kırpılır.
- **ESTOP:** `active:true` herkes (teleop sıfırlanır, aktif nav hedefi iptal edilir); `active:false` sadece controller.
  ESTOP'tan çıkış: `estop false` + `set_mode` (mode_manager kuralı).
- **Hedefler:** Nav2 `NavigateToPose` aksiyonu; durum `idle/active/succeeded/failed/canceled` telemetride. Mod AUTO değilse ack'te uyarı.
- **İstasyonlar:** `~/.agv/stations.yaml` (atomik yazma), `/agv/stations` latched JSON, değişince tüm istemcilere `stations`.
- **save_map:** `/slam_toolbox/serialize_map` (→ `<map_dir>/<ad>.posegraph/.data`) + `/map_saver/save_map` servisi varsa o,
  yoksa `ros2 run nav2_map_server map_saver_cli -f <map_dir>/<ad>` → `ack save_map ok/msg`.
- **Harita PNG yönü:** PNG satır 0 (üst) = **en yüksek y**. Piksel (c, r) merkezi:
  `x = origin.x + (c+0.5)*res`, `y = origin.y + (height-1-r+0.5)*res`. PNG'yi çevirmeden, sol-alt köşesi origin'de olacak şekilde
  (+y yukarı) çizin. Değerler: 0 dolu, 254 boş, 205 bilinmeyen.
- **Scan:** `map→laser_link` TF ile map frame'ine, yoksa odom'a, o da yoksa lazer frame'inde; ≤360 nokta. Mesajda ek `frame` alanı var.
- **Pose:** TF `map→base_footprint`, yoksa `odom→base_footprint` (telemetride ek `frame` alanı), o da yoksa `null`.
- **Görüntü:** `/camera/image_raw/compressed` (jpeg) base64, istemci başına `image_fps` (≤10). Yavaş istemcide görüntü/scan atlanır.
- **Taşkın koruması:** istemci başına ~60 mesaj/s (burst 120).

## Kod yapısı
| Dosya | İçerik |
|---|---|
| `agv_gateway/protocol.py` | Saf Python: mesaj oluşturucular, doğrulama, yetki, harita→PNG, scan dönüşümü, rate limiter, watchdog, nav durum makinesi, istasyon deposu |
| `agv_gateway/server.py` | asyncio + websockets sunucu (ROS'suz, `Backend` arayüzü) |
| `agv_gateway/gateway_node.py` | rclpy backend; executor ayrı thread'de, aksiyon/servis çağrıları GuardCondition ile ROS thread'ine taşınır |
| `agv_gateway/tools/ws_cli_test.py` | Tezgah test istemcisi |

## Test
```bash
cd src/agv_gateway && python3 -m pytest -q test        # protokol + sahte backend ile gerçek WebSocket testleri
ros2 run agv_gateway ws_cli_test --url ws://<ip>:8765 --scan --image          # dinle
ros2 run agv_gateway ws_cli_test --teleop 0.2 0 --duration 3                  # sür
ros2 run agv_gateway ws_cli_test --estop on   |  --estop off --mode MANUAL  |  --interactive
```
