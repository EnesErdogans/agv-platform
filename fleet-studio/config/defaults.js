'use strict';
// Varsayilan saha (fabrika ici + dis arazi), ornek AGV/UGV filosu ve "Arac Ekle" sablonlari

const world = {
  width: 48, height: 18,
  walls: [
    { x1: 30, y1: 0, x2: 30, y2: 7.5 }, { x1: 30, y1: 10.5, x2: 30, y2: 18 },   // fabrika dis duvari (kapi 7.5–10.5 m)
    { x1: 20, y1: 0, x2: 20, y2: 6 }, { x1: 20, y1: 9.5, x2: 20, y2: 18 },   // depo bolme duvari (kapi 6–9.5 m)
    { x1: 0, y1: 14.5, x2: 3.2, y2: 14.5 },                                   // sarj nisi
  ],
  obstacles: [
    // Raf siralari
    { id: 'raf1', x: 5, y: 5, w: 6, h: 1 }, { id: 'raf2', x: 13, y: 5, w: 4, h: 1 },
    { id: 'raf3', x: 5, y: 9, w: 6, h: 1 }, { id: 'raf4', x: 13, y: 9, w: 4, h: 1 },
    { id: 'raf5', x: 5, y: 13.5, w: 6, h: 1 }, { id: 'raf6', x: 13, y: 13.5, w: 4, h: 1 },
    { id: 'palet1', x: 26.5, y: 7.2, w: 1.2, h: 1 }, { id: 'palet2', x: 22.2, y: 15.6, w: 1.2, h: 1 },
    // Hareketli engel (yaya / forklift)
    // Dis saha: kayalar, agaclar, konteyner
    { id: 'kaya1', x: 37, y: 13, w: 1, h: 0.8 }, { id: 'kaya2', x: 41.5, y: 4.2, w: 1.2, h: 1 },
    { id: 'agac1', x: 37.6, y: 5, w: 0.5, h: 0.5 }, { id: 'agac2', x: 43.8, y: 9, w: 0.5, h: 0.5 },
    { id: 'konteyner', x: 31.5, y: 14.6, w: 2.6, h: 1.2 },
    { id: 'yaya', x: 21.6, y: 10.6, w: 0.5, h: 0.5, move: { dx: 5.5, dy: 0, speed: 0.6 } },
  ],
  stations: [
    { id: 'A', name: 'Yükleme A', x: 2.5, y: 3, theta: 0 },
    { id: 'B', name: 'Boşaltma B', x: 25, y: 14, theta: Math.PI / 2 },
    { id: 'C', name: 'Depo C', x: 25.5, y: 3, theta: 0 },
    { id: 'D', name: 'Montaj D', x: 15, y: 16.5, theta: Math.PI },
    { id: 'S', name: 'Şarj', x: 1.6, y: 16.2, theta: Math.PI, type: 'charger' },
    // Dis saha devriye noktalari
    { id: 'DV1', name: 'Devriye 1', x: 34, y: 3, theta: null, type: 'waypoint' },
    { id: 'DV2', name: 'Devriye 2', x: 46, y: 2.5, theta: null, type: 'waypoint' },
    { id: 'DV3', name: 'Devriye 3', x: 46, y: 16, theta: null, type: 'waypoint' },
    { id: 'DV4', name: 'Devriye 4', x: 35, y: 16.5, theta: null, type: 'waypoint' },
    { id: 'US', name: 'Saha Şarj', x: 32, y: 12.6, theta: Math.PI / 2, type: 'charger' },
  ],
  // Zemin bolgeleri: roof = kapali alan (GPS yok); digerleri arazi zeminleri
  zones: [
    { type: 'roof', x: 0, y: 0, w: 30, h: 18 },
    { type: 'grass', x: 35, y: 0, w: 13, h: 18 },
    { type: 'gravel', x: 30, y: 4.5, w: 5, h: 9 },
    { type: 'mud', x: 38.5, y: 6.5, w: 4, h: 4 },
    { type: 'sand', x: 43, y: 11.5, w: 4, h: 3.5 },
  ],
  origin: { lat: 39.0, lon: 35.0 },
  // Cizgi izleyen araclar icin serit dongusu (koseler pahli)
  tapes: [
    { x1: 4.5, y1: 2, x2: 17.5, y2: 2 }, { x1: 17.5, y1: 2, x2: 18.5, y2: 3 },
    { x1: 18.5, y1: 3, x2: 18.5, y2: 10.7 }, { x1: 18.5, y1: 10.7, x2: 17.5, y2: 11.7 },
    { x1: 17.5, y1: 11.7, x2: 4.5, y2: 11.7 }, { x1: 4.5, y1: 11.7, x2: 3.5, y2: 10.7 },
    { x1: 3.5, y1: 10.7, x2: 3.5, y2: 3 }, { x1: 3.5, y1: 3, x2: 4.5, y2: 2 },
  ],
};

const LINE_SCRIPT = `// Çizgi izleyen: şerit sensörü ofsetine göre PD kontrol
let last = 0;
function loop(agv, dt) {
  const line = agv.line;
  const front = agv.ultrasonic[1] || 0;          // orta ultrasonik (m), 0 = engel yok
  if (front > 0 && front < 0.45) return agv.stop();
  if (!line || line.offset === null) {           // çizgi kayboldu: son yöne doğru ara
    return agv.setVelocity(0.05, last >= 0 ? 0.9 : -0.9);
  }
  const e = line.offset;                          // +: çizgi solda
  const w = 14 * e + 1.2 * (e - last) / dt;
  last = e;
  agv.setVelocity(0.55 - Math.min(0.3, Math.abs(e) * 6), w);
}
`;

const PATROL_SCRIPT = `// GPS + pusula ile devriye (UGV): goTo kullanmadan kendi kontrolcün
const hedefler = ['DV1', 'DV2', 'DV3', 'DV4'];
let i = 0;
function loop(ugv, dt) {
  const g = ugv.gps;
  if (!g || !g.fix) return ugv.setVelocity(0.25, 0);      // çatı altı: uydu yok, düz çık
  const h = ugv.stations.find(s => s.id === hedefler[i]);
  const dx = h.x - g.x, dy = h.y - g.y;
  if (Math.hypot(dx, dy) < 1.2) {                          // GPS gürültüsü ~0.6 m
    ugv.log('Varıldı:', h.name, '| zemin:', ugv.terrain || 'beton', '| batarya %', ugv.battery);
    i = (i + 1) % hedefler.length;
    return;
  }
  let e = Math.atan2(dy, dx) - ugv.imu.heading;
  e = Math.atan2(Math.sin(e), Math.cos(e));
  const on = ugv.lidarMin(-35, 35);
  if (ugv.bumper || on < 0.3) return ugv.setVelocity(-0.25, 0);           // sıkıştı: geri çık
  if (on < 0.9) {                                                          // engel: açık tarafa kaç
    const sol = ugv.lidarMin(35, 110), sag = ugv.lidarMin(-110, -35);
    return ugv.setVelocity(0.15, sol > sag ? 1.4 : -1.4);
  }
  ugv.setVelocity(Math.abs(e) > 0.7 ? 0.05 : 0.6, 2.2 * e);
}
`;

const base = {
  class: 'AGV', offroad: 0, turnSlip: 0.15,
  color: '#f5b301', kinematics: 'diff', length: 0.9, width: 0.6, wheelRadius: 0.1, trackWidth: 0.5, wheelBase: 0.6,
  maxSpeed: 1.2, maxAngular: 1.8, maxAccel: 0.8, maxAngAccel: 2.5, maxSteer: 30, mass: 85, payload: 0, maxPayload: 200, motorTau: 0.2,
  battery: { voltage: 24, capacityWh: 960, chargeW: 400 }, encoder: { ticksPerRev: 2048, noise: 0.01 },
  safety: { enabled: true, stopDist: 0.3 }, sensors: [], driver: { type: 'sim' }, script: '',
};

const agv200 = {
  ...base, id: 'agv200', name: 'AGV-200', start: { x: 2.5, y: 3, theta: 0 },
  sensors: [
    { type: 'lidar', name: 'RPLIDAR A1M8', x: 0.2, y: 0, theta: 0, range: 12, fov: 360, rays: 360, noise: 0.01, rate: 7 },
    { type: 'camera', name: 'Pi AI Camera', x: 0.45, y: 0, theta: 0, fov: 76, range: 5 },
    { type: 'imu', noise: 0.004 }, { type: 'bumper' },
  ],
};

const vehicles = [
  agv200,
  {
    ...base, id: 'cizgi01', name: 'Çizgi-01', color: '#3ecf8e', length: 0.5, width: 0.4, wheelRadius: 0.05, trackWidth: 0.34,
    maxSpeed: 0.8, maxAngular: 2.5, maxAccel: 1.2, maxAngAccel: 5, mass: 18, maxPayload: 40, motorTau: 0.1,
    battery: { voltage: 12, capacityWh: 120, chargeW: 80 }, encoder: { ticksPerRev: 600, noise: 0.02 },
    start: { x: 7, y: 2, theta: 0 }, script: LINE_SCRIPT,
    sensors: [
      { type: 'line', name: 'Şerit sensörü', x: 0.22, count: 8, spacing: 0.015 },
      { type: 'ultrasonic', name: 'US sol', x: 0.24, y: 0.14, theta: 30, range: 4, cone: 20 },
      { type: 'ultrasonic', name: 'US orta', x: 0.25, y: 0, theta: 0, range: 4, cone: 20 },
      { type: 'ultrasonic', name: 'US sağ', x: 0.24, y: -0.14, theta: -30, range: 4, cone: 20 },
      { type: 'bumper' },
    ],
  },
  {
    ...base, id: 'cekici01', name: 'Çekici-AK', color: '#5aa9ff', kinematics: 'ackermann', length: 1.6, width: 0.9, wheelRadius: 0.15,
    trackWidth: 0.8, wheelBase: 1.1, maxSpeed: 1.5, maxAngular: 1.2, maxAccel: 0.6, maxSteer: 38, mass: 320, maxPayload: 1000, motorTau: 0.35,
    battery: { voltage: 48, capacityWh: 4800, chargeW: 1500 }, start: { x: 24, y: 11.6, theta: Math.PI / 2 },
    sensors: [
      { type: 'lidar', name: 'Güvenlik LiDAR', x: 1.3, y: 0, theta: 0, range: 20, fov: 270, rays: 270, noise: 0.015, rate: 12 },
      { type: 'ultrasonic', name: 'US arka', x: -0.25, y: 0, theta: 180, range: 4, cone: 30 },
      { type: 'imu' }, { type: 'bumper' },
    ],
  },
  {
    ...base, id: 'omni01', name: 'Omni-M4', color: '#c084fc', kinematics: 'mecanum', length: 0.7, width: 0.6, wheelRadius: 0.076,
    trackWidth: 0.5, wheelBase: 0.5, maxSpeed: 1.0, maxAngular: 2.0, maxAccel: 1.0, mass: 40, maxPayload: 100, motorTau: 0.12,
    battery: { voltage: 24, capacityWh: 480, chargeW: 200 }, start: { x: 12, y: 16.5, theta: 0 },
    sensors: [
      { type: 'lidar', name: 'LiDAR 8 m', x: 0, y: 0, theta: 0, range: 8, fov: 360, rays: 240, noise: 0.02, rate: 10 },
      { type: 'imu' }, { type: 'bumper' },
    ],
  },
  {
    ...base, id: 'rover01', name: 'Paletli Rover', class: 'UGV', color: '#ff7a59', kinematics: 'skid', length: 0.42, width: 0.34, wheelRadius: 0.035,
    trackWidth: 0.28, maxSpeed: 0.6, maxAngular: 2.2, maxAccel: 1.0, maxAngAccel: 5, mass: 3.5, maxPayload: 3, motorTau: 0.12, offroad: 0.9, turnSlip: 0.22,
    battery: { voltage: 11.1, capacityWh: 55, chargeW: 40 }, idlePower: 9, encoder: { ticksPerRev: 0, noise: 0 },   // enkodersiz motorlar
    safety: { enabled: true, stopDist: 0.2 }, start: { x: 33, y: 9, theta: 0 }, script: PATROL_SCRIPT,
    sensors: [
      { type: 'lidar', name: 'RPLIDAR A1M8', x: 0, y: 0, theta: 0, range: 12, fov: 360, rays: 360, noise: 0.01, rate: 7 },
      { type: 'camera', name: 'Pi AI Camera', x: 0.18, y: 0, theta: 0, fov: 76, range: 5 },
      { type: 'gps', name: 'GPS', noise: 0.6, rate: 5 }, { type: 'imu', noise: 0.004 }, { type: 'bumper' },
    ],
  },
  {
    ...base, id: 'kesif01', name: 'Keşif-01', class: 'UGV', color: '#f472b6', kinematics: 'skid', length: 0.24, width: 0.2, wheelRadius: 0.025,
    trackWidth: 0.16, maxSpeed: 0.45, maxAngular: 2.5, maxAccel: 1.2, maxAngAccel: 6, mass: 1.2, maxPayload: 0.5, motorTau: 0.1, offroad: 0.95, turnSlip: 0.25,
    battery: { voltage: 7.4, capacityWh: 20, chargeW: 15 }, idlePower: 6, encoder: { ticksPerRev: 360, noise: 0.03 },
    safety: { enabled: true, stopDist: 0.15 }, start: { x: 33, y: 6.5, theta: 0 },
    sensors: [
      { type: 'camera', name: 'Pi AI Camera', x: 0.1, y: 0, theta: 0, fov: 76, range: 4 },
      { type: 'ultrasonic', name: 'US ön', x: 0.12, y: 0, theta: 0, range: 3, cone: 25 },
      { type: 'imu' }, { type: 'bumper' },
    ],
  },
];

// "Arac Ekle" penceresindeki sablonlar
const presets = [
  { label: 'Simülasyon — AGV-200 (diferansiyel, LiDAR + kamera)', spec: { ...agv200, id: '', name: 'AGV-200' } },
  { label: 'Simülasyon — Çizgi izleyen (şerit + ultrasonik)', spec: { ...vehicles[1], id: '', name: 'Çizgi' } },
  { label: 'Simülasyon — Ackermann çekici', spec: { ...vehicles[2], id: '', name: 'Çekici' } },
  { label: 'Simülasyon — Mecanum (omni)', spec: { ...vehicles[3], id: '', name: 'Omni' } },
  { label: 'Simülasyon — UGV paletli rover (LiDAR + kamera + GPS, enkodersiz)', spec: { ...vehicles[4], id: '', name: 'Rover' } },
  { label: 'Simülasyon — UGV keşif aracı (kamera + ultrasonik)', spec: { ...vehicles[5], id: '', name: 'Keşif' } },
  {
    label: 'GERÇEK — UGV / AGV: Raspberry Pi köprüsü (UDP → Arduino Mega, enkodersiz olur)',
    spec: { ...vehicles[4], id: '', name: 'Rover (Pi)', script: '', driver: { type: 'udp', host: 'agv.local', port: 4210, listenPort: 4211 } },
  },
  {
    label: 'GERÇEK — ROS 2 Humble (rosbridge, Raspberry Pi)',
    spec: { ...agv200, id: '', name: 'AGV (ROS 2)', color: '#ff7a59', driver: { type: 'rosbridge', url: 'ws://agv.local:9090', namespace: '', topics: { cmd_vel: '/cmd_vel', odom: '/odom', scan: '/scan', battery: '/battery_state', pose: '', gps: '' } } },
  },
  {
    label: 'GERÇEK — Arduino Mega (USB seri)',
    spec: { ...agv200, id: '', name: 'AGV (Seri)', color: '#ff7a59', sensors: [{ type: 'ultrasonic', name: 'US ön', x: 0.45, y: 0, theta: 0, range: 4 }, { type: 'bumper' }], driver: { type: 'serial', port: '', baud: 115200 } },
  },
  {
    label: 'GERÇEK — ESP32 (Wi-Fi UDP)',
    spec: { ...agv200, id: '', name: 'AGV (ESP32)', color: '#ff7a59', sensors: [{ type: 'ultrasonic', name: 'US ön', x: 0.45, y: 0, theta: 0, range: 4 }], driver: { type: 'udp', host: '192.168.1.50', port: 4210, listenPort: 4212, oneWay: false } },
  },
];

module.exports = { world, vehicles, presets, base, LINE_SCRIPT, PATROL_SCRIPT };
