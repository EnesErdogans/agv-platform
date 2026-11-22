'use strict';
// Gercek araclar icin suruculer: ROS 2 (rosbridge), seri port (Arduino), UDP (ESP32)
const dgram = require('dgram');
const WebSocket = require('ws');
const { wrap, TAU } = require('../sim/geom');

let SerialPort = null, ReadlineParser = null;
try { ({ SerialPort, ReadlineParser } = require('serialport')); } catch (e) { /* serialport yoksa seri surucu devre disi */ }

function yawFromQuat(q) { return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); }
function quatFromYaw(th) { return { x: 0, y: 0, z: Math.sin(th / 2), w: Math.cos(th / 2) }; }
function stamp() { const t = Date.now(); return { sec: Math.floor(t / 1000), nanosec: (t % 1000) * 1e6 }; }

class RealBase {
  constructor(spec, fleet) {
    this.spec = spec; this.fleet = fleet;
    this.cfg = spec.driver || {};
    this.connected = false; this.wantConnected = false;
    this.cmd = { v: 0, vy: 0, w: 0 };
    this.raw = { x: 0, y: 0, theta: 0 };            // aracin bildirdigi odometri
    const s = spec.start || { x: 2, y: 2, theta: 0 };
    this.offset = { x: s.x, y: s.y, theta: s.theta || 0 }; // harita <- odom donusumu
    this.absolute = false;                          // true ise arac dogrudan harita pozu bildiriyor (or. AMCL)
    this.vel = { v: 0, vy: 0, w: 0 };
    this.sensors = {}; this.scanSeq = 0;
    this.battery = { soc: null, voltage: null, charging: false, power: null };
    this.collided = false; this.payload = spec.payload || 0;
    this.lastRx = 0; this.lastTicks = null; this.distance = 0;
    this._retry = null;
  }
  get pose() {
    if (this.absolute) return this.raw;
    const o = this.offset, c = Math.cos(o.theta), s = Math.sin(o.theta);
    return { x: o.x + c * this.raw.x - s * this.raw.y, y: o.y + s * this.raw.x + c * this.raw.y, theta: wrap(o.theta + this.raw.theta) };
  }
  get odom() { return this.raw; }
  // Araci haritada istenen poza oturtur (odom ofsetini yeniden hesaplar)
  setPose(p) {
    this.absolute = false;
    const th = wrap(p.theta - this.raw.theta), c = Math.cos(th), s = Math.sin(th);
    this.offset = { x: p.x - (c * this.raw.x - s * this.raw.y), y: p.y - (s * this.raw.x + c * this.raw.y), theta: th };
  }
  reset() { }
  step(dt) {
    this.stale = this.connected && Date.now() - this.lastRx > 3000;
    // Enkodersiz arac (ticksPerRev = 0) ve konum bildirmiyorsa: gonderilen komuttan acik cevrim tahmin
    const noEnc = !(this.spec.encoder && +this.spec.encoder.ticksPerRev > 0);
    if (this.connected && dt && noEnc && !this._lastPoseT) {
      const c = this.cmd, th = this.raw.theta;
      this.raw = { x: this.raw.x + ((c.v || 0) * Math.cos(th) - (c.vy || 0) * Math.sin(th)) * dt, y: this.raw.y + ((c.v || 0) * Math.sin(th) + (c.vy || 0) * Math.cos(th)) * dt, theta: wrap(th + (c.w || 0) * dt) };
      this.vel = { v: c.v || 0, vy: c.vy || 0, w: c.w || 0 };
      this.deadReckoning = true;
    } else this.deadReckoning = false;
  }
  log(level, msg) { this.fleet.log(level, msg, this.spec.id); }
  setConnected(on, why) {
    if (on !== this.connected) this.log(on ? 'ok' : 'warn', on ? 'Bağlantı kuruldu' : ('Bağlantı kesildi' + (why ? ': ' + why : '')));
    this.connected = on;
  }
  scheduleRetry() {
    clearTimeout(this._retry);
    if (this.wantConnected) this._retry = setTimeout(() => this.wantConnected && this.open(), 3000);
  }
  connect() { this.wantConnected = true; this.open(); }
  disconnect() { this.wantConnected = false; clearTimeout(this._retry); this.close(); this.setConnected(false); }

  // Ortak telemetri: { enc:[l,r], x,y,th, v,w, bat(V), soc, us:[m..], bumper, scan:[m..], scanMin, scanInc }
  applyTelemetry(t) {
    this.lastRx = Date.now();
    const sp = this.spec;
    const hasEnc = !!(sp.encoder && +sp.encoder.ticksPerRev > 0);   // enkodersiz aracta enc alani yok sayilir
    if (hasEnc && Array.isArray(t.enc) && t.enc.length >= 2) {
      if (this.lastTicks) {
        const r = sp.wheelRadius || 0.08, T = sp.trackWidth || sp.width, tpr = (sp.encoder && sp.encoder.ticksPerRev) || 1024;
        const dl = (t.enc[0] - this.lastTicks[0]) / tpr * TAU * r, dr = (t.enc[1] - this.lastTicks[1]) / tpr * TAU * r;
        const ds = (dl + dr) / 2, dth = (dr - dl) / T;
        if (Math.abs(ds) < 2) { // sayac sifirlanmasi / tasma korumasi
          this.raw = { x: this.raw.x + ds * Math.cos(this.raw.theta + dth / 2), y: this.raw.y + ds * Math.sin(this.raw.theta + dth / 2), theta: wrap(this.raw.theta + dth) };
          this.distance += Math.abs(ds);
          const dt = (this.lastRx - this._lastEncT) / 1000;
          if (dt > 0.005) this.vel = { v: ds / dt, vy: 0, w: dth / dt };
        }
      }
      this.lastTicks = [t.enc[0], t.enc[1]]; this._lastEncT = this.lastRx; this._lastPoseT = this.lastRx;
      this.sensors.encoders = t.enc.slice(0, 4);
    }
    if (typeof t.x === 'number' && typeof t.y === 'number') { this.raw = { x: t.x, y: t.y, theta: +t.th || 0 }; this._lastPoseT = this.lastRx; }
    if (typeof t.lat === 'number' && typeof t.lon === 'number') this.sensors.gps = { fix: true, lat: t.lat, lon: t.lon };
    if (typeof t.v === 'number') this.vel = { v: t.v, vy: +t.vy || 0, w: +t.w || 0 };
    if (typeof t.bat === 'number') {
      const nom = (sp.battery && sp.battery.voltage) || 24;
      this.battery.voltage = t.bat;
      this.battery.soc = Math.max(0, Math.min(1, (t.bat / nom - 0.86) / 0.19));
    }
    if (typeof t.soc === 'number') this.battery.soc = t.soc;
    if (Array.isArray(t.us)) {
      const defs = (sp.sensors || []).filter(s => s.type === 'ultrasonic');
      this.sensors.ultrasonic = t.us.map((d, i) => { const m = defs[i] || {}; return { d, x: m.x || 0, y: m.y || 0, theta: (m.theta || 0) * Math.PI / 180, range: m.range || 4, name: m.name || ('us' + i) }; });
    }
    if (t.bumper != null) { this.sensors.bumper = !!t.bumper; this.collided = !!t.bumper; }
    if (Array.isArray(t.scan) && t.scan.length) this.setScan(t.scan, t.scanMin == null ? -Math.PI : t.scanMin, t.scanInc || TAU / t.scan.length, 12, 0.15);
  }
  setScan(ranges, angleMin, angleInc, range, rangeMin) {
    const m = (this.spec.sensors || []).find(s => s.type === 'lidar') || {};
    this.sensors.lidar = { seq: ++this.scanSeq, angleMin, angleInc, range, rangeMin, ranges, x: m.x || 0, y: m.y || 0, theta: (m.theta || 0) * Math.PI / 180, name: m.name || 'lidar' };
  }
}

// ---- ROS 2: rosbridge_suite WebSocket (varsayilan ws://<arac>:9090) ----
class RosbridgeDriver extends RealBase {
  constructor(spec, fleet) { super(spec, fleet); this.type = 'rosbridge'; this.ws = null; this._lastPub = 0; }
  topic(name, def) { const ns = (this.cfg.namespace || '').replace(/\/+$/, ''); const t = (this.cfg.topics && this.cfg.topics[name]) || def; return t.startsWith('/') ? ns + t : ns + '/' + t; }
  open() {
    this.close();
    const url = this.cfg.url || 'ws://agv.local:9090';
    let ws;
    try { ws = new WebSocket(url, { handshakeTimeout: 4000 }); } catch (e) { this.log('error', 'Geçersiz adres: ' + url); return; }
    this.ws = ws;
    ws.on('open', () => {
      if (this.ws !== ws) return;
      this.setConnected(true); this.lastRx = Date.now();
      const tx = o => ws.send(JSON.stringify(o));
      tx({ op: 'advertise', topic: this.topic('cmd_vel', '/cmd_vel'), type: 'geometry_msgs/msg/Twist' });
      tx({ op: 'subscribe', topic: this.topic('odom', '/odom'), type: 'nav_msgs/msg/Odometry', throttle_rate: 50, queue_length: 1 });
      tx({ op: 'subscribe', topic: this.topic('scan', '/scan'), type: 'sensor_msgs/msg/LaserScan', throttle_rate: 120, queue_length: 1 });
      tx({ op: 'subscribe', topic: this.topic('battery', '/battery_state'), type: 'sensor_msgs/msg/BatteryState', throttle_rate: 1000, queue_length: 1 });
      const gt = this.cfg.topics && this.cfg.topics.gps;
      if (gt) tx({ op: 'subscribe', topic: this.topic('gps', gt), type: 'sensor_msgs/msg/NavSatFix', throttle_rate: 500, queue_length: 1 });
      const pt = this.cfg.topics && this.cfg.topics.pose;
      if (pt) tx({ op: 'subscribe', topic: this.topic('pose', pt), type: 'geometry_msgs/msg/PoseWithCovarianceStamped', queue_length: 1 });
    });
    ws.on('message', data => {
      let m; try { m = JSON.parse(data); } catch (e) { return; }
      if (m.op !== 'publish' || !m.msg) return;
      this.lastRx = Date.now();
      const msg = m.msg;
      if (m.topic === this.topic('odom', '/odom')) {
        const p = msg.pose.pose, tw = msg.twist.twist;
        const np = { x: p.position.x, y: p.position.y, theta: yawFromQuat(p.orientation) };
        this._lastPoseT = Date.now();
        if (!this.absolute) { this.distance += Math.min(1, Math.hypot(np.x - this.raw.x, np.y - this.raw.y)); this.raw = np; }
        this.vel = { v: tw.linear.x, vy: tw.linear.y, w: tw.angular.z };
      } else if (m.topic === this.topic('scan', '/scan')) {
        const r = msg.ranges.map(x => (x == null || !isFinite(x) || x > msg.range_max) ? 0 : +(+x).toFixed(3));
        this.setScan(r, msg.angle_min, msg.angle_increment, msg.range_max, msg.range_min);
      } else if (m.topic === this.topic('battery', '/battery_state')) {
        this.battery.voltage = msg.voltage;
        if (msg.percentage != null && isFinite(msg.percentage)) this.battery.soc = msg.percentage > 1 ? msg.percentage / 100 : msg.percentage;
      } else if (this.cfg.topics && this.cfg.topics.gps && m.topic === this.topic('gps', this.cfg.topics.gps)) {
        this.sensors.gps = { fix: msg.status ? msg.status.status >= 0 : true, lat: msg.latitude, lon: msg.longitude };
      } else if (this.cfg.topics && this.cfg.topics.pose && m.topic === this.topic('pose', this.cfg.topics.pose)) {
        const p = msg.pose.pose; this.absolute = true; this._lastPoseT = Date.now();
        this.raw = { x: p.position.x, y: p.position.y, theta: yawFromQuat(p.orientation) };
      }
    });
    ws.on('close', () => { if (this.ws === ws) { this.setConnected(false); this.scheduleRetry(); } });
    ws.on('error', e => { if (this.ws === ws && !this.connected) this.log('warn', 'Bağlanılamadı (' + url + '): ' + (e.code || e.message)); });
  }
  close() { if (this.ws) { const w = this.ws; this.ws = null; try { w.terminate(); } catch (e) { } } }
  send(cmd) {
    this.cmd = cmd;
    const now = Date.now();
    if (!this.connected || !this.ws || this.ws.readyState !== 1 || now - this._lastPub < 50) return;
    this._lastPub = now;
    this.ws.send(JSON.stringify({ op: 'publish', topic: this.topic('cmd_vel', '/cmd_vel'), msg: { linear: { x: cmd.v || 0, y: cmd.vy || 0, z: 0 }, angular: { x: 0, y: 0, z: cmd.w || 0 } } }));
  }
}

// ---- Seri port: Arduino Mega satir protokolu ----
//  PC -> MCU :  V,<v_mm_s>,<w_mrad_s>\n   |  S\n (dur)  |  R\n (enkoder sifirla)
//  MCU -> PC :  T,<encL>,<encR>,<vbat_mV>,<bumper>,<us1_mm>,<us2_mm>,...\n
class SerialDriver extends RealBase {
  constructor(spec, fleet) { super(spec, fleet); this.type = 'serial'; this.port = null; this._lastTx = 0; }
  open() {
    this.close();
    if (!SerialPort) { this.log('error', 'serialport modülü yüklü değil'); return; }
    const path = this.cfg.port;
    if (!path) { this.log('error', 'Seri port seçilmedi'); return; }
    const port = new SerialPort({ path, baudRate: +this.cfg.baud || 115200, autoOpen: false });
    this.port = port;
    port.open(err => {
      if (this.port !== port) return;
      if (err) { this.log('warn', 'Port açılamadı (' + path + '): ' + err.message); this.scheduleRetry(); return; }
      this.setConnected(true); this.lastRx = Date.now(); this.lastTicks = null;
      port.pipe(new ReadlineParser({ delimiter: '\n' })).on('data', line => this.onLine(String(line).trim()));
    });
    port.on('close', () => { if (this.port === port) { this.setConnected(false); this.scheduleRetry(); } });
    port.on('error', e => this.log('warn', 'Seri hata: ' + e.message));
  }
  onLine(line) {
    if (!line.startsWith('T,')) { if (line) this.log('info', 'MCU: ' + line.slice(0, 120)); return; }
    const f = line.split(',').map(Number);
    if (f.length < 3 || f.slice(1).some(isNaN)) return;
    const t = { enc: [f[1], f[2]] };
    if (f.length > 3 && f[3] > 0) t.bat = f[3] / 1000;
    if (f.length > 4) t.bumper = f[4] !== 0;
    if (f.length > 5) t.us = f.slice(5).map(mm => mm / 1000);
    this.applyTelemetry(t);
  }
  close() { if (this.port) { const p = this.port; this.port = null; try { if (p.isOpen) p.close(); } catch (e) { } } }
  send(cmd) {
    this.cmd = cmd;
    const now = Date.now();
    if (!this.connected || !this.port || now - this._lastTx < 50) return;
    this._lastTx = now;
    this.port.write('V,' + Math.round((cmd.v || 0) * 1000) + ',' + Math.round((cmd.w || 0) * 1000) + '\n');
  }
}

// ---- UDP: ESP32 / herhangi bir Wi-Fi kontrolcu, JSON datagram ----
//  PC -> arac : {"t":"cmd","v":0.5,"vy":0,"w":0.2}
//  arac -> PC : {"t":"tel","enc":[l,r],"bat":25.1,"us":[0.8,1.2],"bumper":0}  (ya da "x","y","th")
class UdpDriver extends RealBase {
  constructor(spec, fleet) { super(spec, fleet); this.type = 'udp'; this.sock = null; this._lastTx = 0; }
  open() {
    this.close();
    const sock = dgram.createSocket('udp4');
    this.sock = sock;
    sock.on('message', buf => {
      let t; try { t = JSON.parse(buf.toString()); } catch (e) { return; }
      if (!this.connected) this.setConnected(true);
      this.applyTelemetry(t);
    });
    sock.on('error', e => { this.log('warn', 'UDP hata: ' + e.message); });
    if (this.cfg.oneWay) { this.setConnected(true); this.lastRx = Date.now(); } // telemetri gondermeyen basit araclar
    sock.bind(+this.cfg.listenPort || 4211, () => this.log('info', 'UDP dinleniyor: ' + (+this.cfg.listenPort || 4211) + ' → araç ' + (this.cfg.host || '?') + ':' + (+this.cfg.port || 4210)));
  }
  step(dt) { if (this.connected && !this.cfg.oneWay && Date.now() - this.lastRx > 3000) this.setConnected(false, 'telemetri gelmiyor'); super.step(dt); }
  close() { if (this.sock) { try { this.sock.close(); } catch (e) { } this.sock = null; } }
  send(cmd) {
    this.cmd = cmd;
    const now = Date.now();
    if (!this.sock || !this.cfg.host || now - this._lastTx < 50) return;
    this._lastTx = now;
    const b = Buffer.from(JSON.stringify({ t: 'cmd', v: +(cmd.v || 0).toFixed(3), vy: +(cmd.vy || 0).toFixed(3), w: +(cmd.w || 0).toFixed(3) }));
    this.sock.send(b, +this.cfg.port || 4210, this.cfg.host, () => { });
  }
}

// ---- Simule araci ROS 2'ye acar: kendi ROS kodunu simulasyona karsi test etmek icin ----
// Yayinlar: <ns>/scan, <ns>/odom, /tf   |  Dinler: <ns>/cmd_vel (arac Manuel/Harici moddayken uygulanir)
class RosExposer {
  constructor(vehicle, fleet) { this.v = vehicle; this.fleet = fleet; this.ws = null; this.connected = false; this.want = false; this._seq = -1; this._lastOdom = 0; }
  get cfg() { return this.v.spec.rosExpose || {}; }
  t(name) { const ns = (this.cfg.namespace || '').replace(/\/+$/, ''); return ns + '/' + name; }
  start() { this.want = true; this.open(); }
  stop() { this.want = false; clearTimeout(this._retry); if (this.ws) { try { this.ws.terminate(); } catch (e) { } this.ws = null; } this.connected = false; }
  open() {
    const url = this.cfg.url || 'ws://localhost:9090';
    let ws; try { ws = new WebSocket(url, { handshakeTimeout: 4000 }); } catch (e) { return; }
    this.ws = ws;
    ws.on('open', () => {
      this.connected = true; this.fleet.log('ok', 'ROS köprüsüne açıldı: ' + url, this.v.spec.id);
      const tx = o => ws.send(JSON.stringify(o));
      tx({ op: 'advertise', topic: this.t('scan'), type: 'sensor_msgs/msg/LaserScan' });
      tx({ op: 'advertise', topic: this.t('odom'), type: 'nav_msgs/msg/Odometry' });
      tx({ op: 'advertise', topic: '/tf', type: 'tf2_msgs/msg/TFMessage' });
      tx({ op: 'subscribe', topic: this.t('cmd_vel'), type: 'geometry_msgs/msg/Twist', queue_length: 1 });
    });
    ws.on('message', d => {
      let m; try { m = JSON.parse(d); } catch (e) { return; }
      if (m.op === 'publish' && m.topic === this.t('cmd_vel') && m.msg) this.v.setTeleop(m.msg.linear.x, m.msg.angular.z, m.msg.linear.y, 'ros');
    });
    ws.on('close', () => { if (this.ws !== ws) return; if (this.connected) this.fleet.log('warn', 'ROS köprüsü kapandı', this.v.spec.id); this.connected = false; clearTimeout(this._retry); if (this.want) this._retry = setTimeout(() => this.want && this.open(), 3000); });
    ws.on('error', e => { if (!this.connected) this.fleet.log('warn', 'ROS köprüsüne bağlanılamadı (' + url + '): ' + (e.code || e.message), this.v.spec.id); });
  }
  tick() {
    if (!this.connected || !this.ws || this.ws.readyState !== 1) return;
    const d = this.v.driver, now = Date.now(), ns = (this.cfg.namespace || '').replace(/^\/+|\/+$/g, ''), pre = ns ? ns + '/' : '';
    const pub = (topic, msg) => this.ws.send(JSON.stringify({ op: 'publish', topic, msg }));
    if (now - this._lastOdom >= 50) {
      this._lastOdom = now;
      const o = d.odom, q = quatFromYaw(o.theta), st = stamp();
      pub(this.t('odom'), { header: { stamp: st, frame_id: pre + 'odom' }, child_frame_id: pre + 'base_link', pose: { pose: { position: { x: o.x, y: o.y, z: 0 }, orientation: q }, covariance: new Array(36).fill(0) }, twist: { twist: { linear: { x: d.vel.v, y: d.vel.vy, z: 0 }, angular: { x: 0, y: 0, z: d.vel.w } }, covariance: new Array(36).fill(0) } });
      const tfs = [{ header: { stamp: st, frame_id: pre + 'odom' }, child_frame_id: pre + 'base_link', transform: { translation: { x: o.x, y: o.y, z: 0 }, rotation: q } }];
      const L = d.sensors.lidar;
      if (L) tfs.push({ header: { stamp: st, frame_id: pre + 'base_link' }, child_frame_id: pre + 'laser', transform: { translation: { x: L.x, y: L.y, z: 0.2 }, rotation: quatFromYaw(L.theta) } });
      pub('/tf', { transforms: tfs });
    }
    const L = d.sensors.lidar;
    if (L && L.seq !== this._seq) {
      this._seq = L.seq;
      pub(this.t('scan'), { header: { stamp: stamp(), frame_id: pre + 'laser' }, angle_min: L.angleMin, angle_max: L.angleMin + L.angleInc * (L.ranges.length - 1), angle_increment: L.angleInc, time_increment: 0, scan_time: 0.125, range_min: L.rangeMin, range_max: L.range, ranges: L.ranges, intensities: [] });
    }
  }
}

async function listPorts() {
  if (!SerialPort) return [];
  try { return (await SerialPort.list()).map(p => ({ path: p.path, label: [p.path, p.manufacturer, p.friendlyName].filter(Boolean).join(' — ') })); } catch (e) { return []; }
}

module.exports = { RosbridgeDriver, SerialDriver, UdpDriver, RosExposer, listPorts, serialAvailable: !!SerialPort };
