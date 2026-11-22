# ROS 2 Humble ile kullanım

Panel, ROS 2 ile **rosbridge** (WebSocket, port 9090) üzerinden konuşur; bilgisayarına ROS kurman gerekmez.

## 1) Gerçek aracı panele bağlamak

Araçtaki Raspberry Pi'de (Docker):

```bash
cd ros2
docker compose up -d --build      # rosbridge (9090) + RPLIDAR A1M8 (/scan)
```

Panelde **+ Araç → GERÇEK — ROS 2 Humble**, sonra **Bağlantı** sekmesi:

| Alan | Değer |
|---|---|
| rosbridge adresi | `ws://agv.local:9090` (ya da Pi'nin IP'si) |
| cmd_vel | panelin yayınladığı `geometry_msgs/msg/Twist` |
| odom | `nav_msgs/msg/Odometry` |
| scan | `sensor_msgs/msg/LaserScan` |
| battery | `sensor_msgs/msg/BatteryState` (isteğe bağlı) |
| pose | boş bırak; AMCL / slam_toolbox kullanıyorsan `/amcl_pose` yaz |
| gps | UGV için `sensor_msgs/msg/NavSatFix` konusu (isteğe bağlı) |

`/cmd_vel` → motor ve enkoder → `/odom` çevrimini araçta senin düğümün yapar
(ör. Arduino Mega ile seri konuşan bir düğüm). Panel yalnızca bu konuları kullanır.

## 2) Kendi ROS 2 kodunu simülasyona karşı test etmek

Simüle bir araç seç → **Bağlantı** → **Simüle aracı ROS 2'ye aç** → rosbridge adresi
(`ws://localhost:9090` ya da Pi) ve bir namespace (ör. `/sim`) → **Ayarları uygula**.

Araç şunları yayınlar / dinler:

| Konu | Tip | Yön |
|---|---|---|
| `/sim/scan` | LaserScan (frame `sim/laser`) | simülasyon → ROS |
| `/sim/odom` | Odometry (`sim/odom` → `sim/base_link`) | simülasyon → ROS |
| `/tf` | odom → base_link → laser | simülasyon → ROS |
| `/sim/cmd_vel` | Twist | ROS → simülasyon (araç **Manuel** moddayken) |

Böylece slam_toolbox, Nav2 ya da kendi kontrol düğümün gerçek araç yerine
simüle LiDAR ve odometriyle çalışır. Zaman damgaları duvar saatidir (`use_sim_time:=false`).
