#!/usr/bin/env bash
# AGV tek komut:  bash ~/agv_pi/agv.sh  [baslat|dur|durum|log|otomatik|otomatik-kapat]
#   (bos)          her seyi baslat: kamera (host, :8000) + LiDAR/SLAM/panel/gateway (Docker, :8080 :8765 :8766)
#   dur            hepsini durdur
#   durum          ne calisiyor?
#   log            son log satirlari
#   otomatik       acilista otomatik baslat (systemd)
#   otomatik-kapat otomatik baslatmayi kaldir
set -u

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR" || exit 1

IMAGE=agv-lidar
CONTAINER=agv-lidar
OWNER="$(stat -c %U "$DIR")"
OWNER_HOME="$(getent passwd "$OWNER" | cut -d: -f6)"
OWNER_HOME="${OWNER_HOME:-$HOME}"
MAPS_DIR="${AGV_MAPS:-$OWNER_HOME/maps}"
LOG_DIR="$DIR/logs"
KAMERA_LOG="$LOG_DIR/kamera.log"
KAMERA_PID="$LOG_DIR/kamera.pid"
PYTHON="${AGV_PYTHON:-/usr/bin/python3}"
[ -x "$PYTHON" ] || PYTHON=python3
KAMERA_UNIT=agv-kamera.service
LIDAR_UNIT=agv-lidar.service

if [ -t 1 ]; then
  G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; B=$'\e[1m'; C=$'\e[36m'; N=$'\e[0m'
else
  G=""; R=""; Y=""; B=""; C=""; N=""
fi
ok()   { echo "${G}[ OK ]${N} $*"; }
warn() { echo "${Y}[UYARI]${N} $*"; }
err()  { echo "${R}[HATA]${N} $*"; }
info() { echo "${C}[....]${N} $*"; }

SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"
DOCKER="$SUDO docker"

pi_ip() { hostname -I 2>/dev/null | awk '{print $1}'; }

port_pids() {  # $1 = port -> dinleyen PID'ler
  $SUDO ss -ltnpH "sport = :$1" 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u
}
port_open() {
  if command -v ss >/dev/null 2>&1; then ss -ltnH "sport = :$1" 2>/dev/null | grep -q .
  else (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null; fi
}

unit_enabled() { systemctl is-enabled --quiet "$1" 2>/dev/null; }

# ------------------------------------------------------------------ kamera
kamera_kill_old() {
  local p
  # "[u]ltra" kalibi: pkill/sudo kendi komut satirini eslemesin
  for p in '[u]ltra_simple' '[k]amera_ai\.py' '[k]amera_yayin\.py' '[k]amera_servis\.py'; do
    $SUDO pkill -f "$p" 2>/dev/null && info "eski süreç durduruldu: ${p//[\[\]\\]/}"
  done
  for p in $(port_pids 8000); do
    $SUDO kill "$p" 2>/dev/null && info "port 8000'i tutan süreç durduruldu (PID $p)"
  done
  sleep 0.5
  rm -f "$KAMERA_PID"
}

kamera_start() {
  mkdir -p "$LOG_DIR"
  if unit_enabled "$KAMERA_UNIT"; then
    $SUDO systemctl stop "$KAMERA_UNIT" 2>/dev/null
    kamera_kill_old
    info "kamera servisi (systemd) başlatılıyor…"
    $SUDO systemctl start "$KAMERA_UNIT"
  else
    kamera_kill_old
    info "kamera başlatılıyor… (log: logs/kamera.log)"
    {
      echo "===== $(date '+%F %T') kamera başlatılıyor ====="
    } >> "$KAMERA_LOG"
    nohup "$PYTHON" -u "$DIR/kamera/kamera_servis.py" >> "$KAMERA_LOG" 2>&1 &
    echo $! > "$KAMERA_PID"
  fi
  # ilk acilista IMX500 yazilimi yuklenir (uzun surebilir); 6 sn icinde cokmezse calisiyor say
  local i
  for i in 1 2 3 4 5 6; do
    sleep 1
    if ! kamera_running; then
      err "Kamera başlarken çöktü. LiDAR ile devam ediliyor. Son log satırları:"
      tail -n 6 "$KAMERA_LOG" | sed 's/^/        /'
      return 1
    fi
    port_open 8000 && break
  done
  if port_open 8000; then ok "Kamera çalışıyor (port 8000)"; else ok "Kamera açılıyor (AI modeli yükleniyor olabilir, birkaç sn)"; fi
  return 0
}

kamera_running() {
  if unit_enabled "$KAMERA_UNIT"; then
    systemctl is-active --quiet "$KAMERA_UNIT"
  else
    [ -f "$KAMERA_PID" ] && kill -0 "$(cat "$KAMERA_PID")" 2>/dev/null
  fi
}

kamera_stop() {
  unit_enabled "$KAMERA_UNIT" && $SUDO systemctl stop "$KAMERA_UNIT" 2>/dev/null
  if [ -f "$KAMERA_PID" ]; then
    kill "$(cat "$KAMERA_PID")" 2>/dev/null
    rm -f "$KAMERA_PID"
  fi
  $SUDO pkill -f '[k]amera_servis\.py' 2>/dev/null
  true
}

# ------------------------------------------------------------------ docker / lidar
src_hash() {
  (cd "$DIR/lidar_slam" && find Dockerfile src -type f \
     ! -path '*/__pycache__/*' ! -path '*/.pytest_cache/*' ! -name '*.pyc' -print0 \
     | sort -z | xargs -0 sha256sum) | sha256sum | cut -d' ' -f1
}

docker_check() {
  if ! command -v docker >/dev/null 2>&1; then
    err "Docker kurulu değil. Kur: curl -fsSL https://get.docker.com | sudo sh"
    return 1
  fi
  if ! $DOCKER info >/dev/null 2>&1; then
    info "Docker servisi başlatılıyor…"
    $SUDO systemctl start docker 2>/dev/null
    sleep 2
    $DOCKER info >/dev/null 2>&1 || { err "Docker çalışmıyor (sudo systemctl status docker)"; return 1; }
  fi
}

lidar_start() {
  docker_check || return 1
  [ -e /dev/ttyUSB0 ] || warn "/dev/ttyUSB0 yok — LiDAR USB kablosu takılı mı? (yine de devam ediliyor)"
  mkdir -p "$MAPS_DIR"
  [ -n "$SUDO" ] || chown "$OWNER": "$MAPS_DIR" 2>/dev/null

  local h old=""
  h="$(src_hash)"
  [ -f "$DIR/.build_hash" ] && old="$(cat "$DIR/.build_hash")"
  if ! $DOCKER image inspect "$IMAGE" >/dev/null 2>&1 || [ "$h" != "$old" ]; then
    info "Docker imajı derleniyor (ilk seferde birkaç dakika sürer)…"
    if $DOCKER build -t "$IMAGE" "$DIR/lidar_slam"; then
      echo "$h" > "$DIR/.build_hash"
      ok "İmaj hazır"
    else
      err "Docker imajı derlenemedi (internet bağlantısını kontrol et)"
      return 1
    fi
  else
    ok "Docker imajı güncel"
  fi

  $DOCKER rm -f "$CONTAINER" >/dev/null 2>&1
  info "LiDAR / SLAM / panel konteyneri başlatılıyor…"
  if ! $DOCKER run -d --name "$CONTAINER" --restart unless-stopped --net=host --privileged \
      -v /dev:/dev -v "$DIR/lidar_slam:/ws" -v "$MAPS_DIR:/maps" "$IMAGE" \
      bash -c "source /opt/ros/humble/setup.bash && source /opt/agv_ws/install/setup.bash && ros2 launch /ws/slam.launch.py" \
      >/dev/null; then
    err "Konteyner başlatılamadı"
    return 1
  fi

  local i
  for i in $(seq 1 60); do
    if curl -s -o /dev/null -m 1 http://127.0.0.1:8080/; then
      ok "Panel hazır (${i} sn)"
      return 0
    fi
    if [ "$($DOCKER inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
      err "Konteyner durdu. Son loglar:"
      $DOCKER logs --tail 20 "$CONTAINER" 2>&1 | sed 's/^/        /'
      return 1
    fi
    sleep 1
  done
  warn "Panel 60 sn içinde cevap vermedi. Log: bash $DIR/agv.sh log"
  return 1
}

lidar_stop() {
  command -v docker >/dev/null 2>&1 && $DOCKER rm -f "$CONTAINER" >/dev/null 2>&1
  true
}

# ------------------------------------------------------------------ komutlar
summary() {
  local ip
  ip="$(pi_ip)"; ip="${ip:-agv.local}"
  echo
  echo "${B}================= AGV HAZIR =================${N}"
  echo "  ${B}Panel (harita + kamera):${N}  ${G}http://$ip:8080${N}"
  echo "  ${B}Windows uygulaması:${N}       adres ${G}$ip${N}   port ${G}8765${N}"
  echo "  ${B}Sadece kamera:${N}            http://$ip:8000"
  echo "  Foxglove (isteğe bağlı):  ws://$ip:8766"
  echo "  Kayıtlı haritalar:        $MAPS_DIR"
  echo
  echo "  Durum: bash $DIR/agv.sh durum    Durdur: bash $DIR/agv.sh dur"
  echo "${B}=============================================${N}"
}

cmd_start() {
  echo "${B}AGV başlatılıyor…${N}"
  local kam=0 lid=0
  kamera_start || kam=1
  lidar_start || lid=1
  summary
  [ $kam -eq 1 ] && warn "Kamera çalışmıyor → log: tail -n 30 $KAMERA_LOG"
  [ $lid -eq 1 ] && warn "LiDAR/panel tarafında sorun var → bash $DIR/agv.sh log"
  return 0
}

cmd_stop() {
  kamera_stop
  lidar_stop
  ok "Hepsi durduruldu (kamera + konteyner)"
}

cmd_status() {
  local ip
  ip="$(pi_ip)"
  echo "${B}AGV durum${N}  (IP: ${ip:-?})"
  if command -v docker >/dev/null 2>&1 && [ "$($DOCKER inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]; then
    ok "Konteyner '$CONTAINER' çalışıyor"
  else
    err "Konteyner '$CONTAINER' çalışmıyor"
  fi
  if kamera_running; then ok "Kamera süreci çalışıyor"; else err "Kamera süreci çalışmıyor (log: $KAMERA_LOG)"; fi
  if [ -e /dev/ttyUSB0 ]; then ok "/dev/ttyUSB0 var"; else warn "/dev/ttyUSB0 yok (LiDAR takılı değil?)"; fi
  local p name
  for p in 8000 8080 8765 8766; do
    case $p in 8000) name="kamera";; 8080) name="panel";; 8765) name="Windows uygulaması";; 8766) name="foxglove";; esac
    if port_open $p; then ok "port $p dinleniyor ($name)"; else err "port $p kapalı ($name)"; fi
  done
  if [ "$($DOCKER inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]; then
    info "/scan hızı ölçülüyor (4 sn)…"
    local out
    out="$($DOCKER exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && timeout 4 ros2 topic hz /scan" 2>&1 | tail -n 4)"
    if echo "$out" | grep -q "average rate"; then
      while IFS= read -r l; do echo "        $l"; done <<< "$out"
    else
      err "/scan verisi yok (LiDAR çalışmıyor). 80008002 hatası varsa LiDAR USB'sini çıkar-tak."
    fi
  fi
  if unit_enabled "$KAMERA_UNIT"; then info "Açılışta otomatik başlatma: AÇIK"
  else info "Açılışta otomatik başlatma: kapalı (bash agv.sh otomatik)"; fi
}

cmd_log() {
  echo "${B}----- kamera ($KAMERA_LOG) -----${N}"
  if unit_enabled "$KAMERA_UNIT"; then
    tail -n 25 "$KAMERA_LOG" 2>/dev/null || journalctl -u "$KAMERA_UNIT" -n 25 --no-pager
  else
    tail -n 25 "$KAMERA_LOG" 2>/dev/null || echo "(kamera logu yok)"
  fi
  echo "${B}----- konteyner ($CONTAINER) -----${N}"
  $DOCKER logs --tail 40 "$CONTAINER" 2>&1 || echo "(konteyner yok)"
}

cmd_auto_on() {
  mkdir -p "$LOG_DIR"
  [ -n "$SUDO" ] || chown "$OWNER": "$LOG_DIR"
  info "systemd servisleri kuruluyor…"
  $SUDO tee "/etc/systemd/system/$KAMERA_UNIT" >/dev/null <<EOF
[Unit]
Description=AGV kamera servisi (port 8000)
After=network-online.target
Wants=network-online.target

[Service]
User=$OWNER
WorkingDirectory=$DIR/kamera
ExecStart=$PYTHON -u $DIR/kamera/kamera_servis.py
Restart=always
RestartSec=5
StandardOutput=append:$KAMERA_LOG
StandardError=append:$KAMERA_LOG

[Install]
WantedBy=multi-user.target
EOF
  $SUDO tee "/etc/systemd/system/$LIDAR_UNIT" >/dev/null <<EOF
[Unit]
Description=AGV LiDAR/SLAM/panel konteyneri (port 8080/8765/8766)
After=docker.service network-online.target
Requires=docker.service
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
Environment=AGV_MAPS=$MAPS_DIR
ExecStart=/bin/bash $DIR/agv.sh lidar
ExecStop=/bin/bash $DIR/agv.sh lidar-dur
TimeoutStartSec=900

[Install]
WantedBy=multi-user.target
EOF
  $SUDO systemctl daemon-reload
  $SUDO systemctl enable docker >/dev/null 2>&1
  $SUDO systemctl enable "$KAMERA_UNIT" "$LIDAR_UNIT" >/dev/null 2>&1
  ok "Açılışta otomatik başlatma AÇIK ($KAMERA_UNIT, $LIDAR_UNIT)"
  info "Şimdi de başlatmak için: bash $DIR/agv.sh"
}

cmd_auto_off() {
  $SUDO systemctl disable --now "$KAMERA_UNIT" "$LIDAR_UNIT" >/dev/null 2>&1
  $SUDO rm -f "/etc/systemd/system/$KAMERA_UNIT" "/etc/systemd/system/$LIDAR_UNIT"
  $SUDO systemctl daemon-reload
  ok "Otomatik başlatma kaldırıldı (konteyner durduruldu; tekrar başlatmak için: bash $DIR/agv.sh)"
}

case "${1:-baslat}" in
  baslat|start|"") cmd_start ;;
  dur|stop) cmd_stop ;;
  durum|status) cmd_status ;;
  log|loglar) cmd_log ;;
  otomatik) cmd_auto_on ;;
  otomatik-kapat) cmd_auto_off ;;
  lidar) lidar_start ;;        # systemd icin
  lidar-dur) lidar_stop ;;     # systemd icin
  -h|--help|yardim)
    sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//' ;;
  *) err "Bilinmeyen komut: $1"; sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
