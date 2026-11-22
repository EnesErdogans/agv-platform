'use strict';
const vm = require('vm');
const fs = require('fs');
const path = require('path');
const { World } = require('./sim/world');
const { clamp, footprint, robotRadius, toWorld, segSeg, pointSegDist } = require('./sim/geom');
const { SimDriver } = require('./drivers/sim');
const { RosbridgeDriver, SerialDriver, UdpDriver, RosExposer } = require('./drivers/real');
const planner = require('./nav/planner');
const { follow, cumulative } = require('./nav/controller');
const { LidarMapper } = require('./nav/mapper');
const defaults = require('../config/defaults');

const ZERO = { v: 0, vy: 0, w: 0 };
const DRIVERS = { sim: SimDriver, rosbridge: RosbridgeDriver, serial: SerialDriver, udp: UdpDriver };

// Referans noktasindan govdenin en uzak kosesine mesafe (yerlesim icin)
function reach(spec) { return (spec.kinematics === 'ackermann' ? (spec.wheelBase || 0) / 2 : 0) + Math.hypot(spec.length, spec.width) / 2; }

function normalize(spec) {
  const b = defaults.base;
  const s = { ...b, ...spec };
  s.battery = { ...b.battery, ...(spec.battery || {}) };
  s.encoder = { ...b.encoder, ...(spec.encoder || {}) };
  s.safety = { ...b.safety, ...(spec.safety || {}) };
  s.driver = { type: 'sim', ...(spec.driver || {}) };
  if (!DRIVERS[s.driver.type]) s.driver.type = 'sim';
  s.sensors = Array.isArray(spec.sensors) ? spec.sensors.map(x => ({ ...x })) : [];
  s.start = { x: 2, y: 2, theta: 0, ...(spec.start || {}) };
  for (const k of ['length', 'width', 'wheelRadius', 'trackWidth', 'wheelBase', 'maxSpeed', 'maxAngular', 'maxAccel', 'maxAngAccel', 'maxSteer', 'mass', 'payload', 'maxPayload', 'motorTau']) {
    s[k] = +s[k]; if (!isFinite(s[k])) s[k] = b[k];
  }
  s.length = clamp(s.length, 0.1, 6); s.width = clamp(s.width, 0.1, 4);
  s.maxSpeed = clamp(s.maxSpeed, 0.05, 10); s.maxAccel = clamp(s.maxAccel, 0.05, 10); s.mass = Math.max(1, s.mass);
  if (!['diff', 'skid', 'ackermann', 'mecanum'].includes(s.kinematics)) s.kinematics = 'diff';
  s.class = s.class === 'UGV' ? 'UGV' : 'AGV';
  s.offroad = clamp(+s.offroad || 0, 0, 1);
  s.turnSlip = clamp(s.turnSlip == null ? 0.15 : +s.turnSlip || 0, 0, 0.8);
  s.name = String(s.name || 'AGV').slice(0, 40);
  s.script = typeof s.script === 'string' ? s.script : '';
  return s;
}

class Vehicle {
  constructor(spec, fleet) {
    this.fleet = fleet;
    this.spec = normalize(spec);
    this.mode = 'manual';
    this.mapping = false; this.mapSource = 'pose'; this._mapSeq = -1;   // LiDAR haritalama
    this.hidden = false;   // gizli arac: simule edilmez, cizilmez, engel sayilmaz
    this.estop = false;
    this.teleop = { v: 0, vy: 0, w: 0, t: 0 };
    this.nav = null; this.mission = null;
    this.status = 'Hazır';
    this.pathSeq = 0; this.blockedT = 0; this.speedScale = 1;
    this.safety = { state: 'ok', gap: null };
    this.cmdOut = ZERO;
    this.script = { running: false, sandbox: null, ctx: null, call: null, cmd: ZERO, acc: 0 };
    this._sentSeq = -1;
    this.exposer = null;
    this.driver = new DRIVERS[this.spec.driver.type](this.spec, fleet);
    this.syncExposer();
  }
  get id() { return this.spec.id; }
  get isSim() { return this.driver.type === 'sim'; }
  get pose() { return this.driver.pose; }
  log(level, msg) { this.fleet.log(level, msg, this.id); }

  syncExposer() {
    const want = this.isSim && this.spec.rosExpose && this.spec.rosExpose.enabled;
    if (this.exposer) { this.exposer.stop(); this.exposer = null; }
    if (want) { this.exposer = new RosExposer(this, this.fleet); this.exposer.start(); }
  }

  dispose() { this.stopScript(); if (this.exposer) this.exposer.stop(); try { this.driver.disconnect(); } catch (e) { } }

  setTeleop(v, w, vy, src) {
    this.teleop = { v: +v || 0, vy: +vy || 0, w: +w || 0, t: Date.now(), src: src || 'panel' };
  }

  setMode(mode) {
    if (!['manual', 'auto', 'script'].includes(mode)) return;
    if (mode !== this.mode) { this.cancel(); if (this.mode === 'script') this.stopScript(); }
    this.mode = mode;
    this.status = mode === 'manual' ? 'Manuel' : mode === 'auto' ? 'Otonom — hedef bekliyor' : 'Kod modu';
  }

  // ---------- Navigasyon ----------
  lidarWorldPoints(maxDist) {
    const L = this.driver.sensors.lidar, out = [];
    if (!L) return out;
    const p = this.pose;
    for (let i = 0; i < L.ranges.length; i += 2) {
      const r = L.ranges[i];
      if (r <= L.rangeMin || r > maxDist) continue;
      const a = L.theta + L.angleMin + i * L.angleInc;
      out.push(toWorld(p, L.x + r * Math.cos(a), L.y + r * Math.sin(a)));
    }
    return out;
  }

  goTo(target, opts = {}) {
    const res = planner.plan(this.fleet.world, this.pose, target, robotRadius(this.spec), opts.avoid ? this.lidarWorldPoints(4) : null);
    if (!res || res.points.length < 1) {
      if (!opts.quiet) this.log('error', 'Yol bulunamadı → ' + (target.name || `(${target.x.toFixed(1)}, ${target.y.toFixed(1)})`));
      return false;
    }
    this.nav = { path: res.points, cum: cumulative(res.points), idx: 0, goalTheta: target.theta == null ? null : target.theta, goal: target, rotating: false };
    this.pathSeq++; this.blockedT = 0;
    this.status = 'Gidiyor → ' + (target.name || `(${target.x.toFixed(1)}, ${target.y.toFixed(1)})`);
    if (res.goalAdjusted && !opts.quiet) this.log('warn', 'Hedef engele çok yakın; en yakın serbest noktaya gidiliyor');
    return true;
  }

  cancel() {
    if (this.nav) { this.nav = null; this.pathSeq++; }
    this.mission = null;
  }

  startMission(m) {
    const steps = (m.steps || []).filter(s => s && ['goto', 'wait', 'load', 'unload'].includes(s.type));
    if (!steps.length) return;
    this.cancel();
    if (this.mode !== 'auto') this.setMode('auto');
    this.mission = { steps, loop: !!m.loop, i: 0, state: 'next', waitLeft: 0, laps: 0 };
    this.log('info', `Görev başladı (${steps.length} adım${m.loop ? ', döngü' : ''})`);
  }

  missionTick(dt) {
    const m = this.mission;
    if (!m) return;
    if (m.state === 'moving') { if (this.nav) return; m.i++; m.state = 'next'; }
    if (m.state === 'waiting' || m.state === 'retry') {
      m.waitLeft -= dt;
      if (m.waitLeft > 0) return;
      if (m.state === 'waiting') m.i++;
      m.state = 'next';
    }
    if (m.i >= m.steps.length) {
      if (m.loop) { m.i = 0; m.laps++; } else { this.mission = null; this.status = 'Görev tamamlandı'; this.log('ok', 'Görev tamamlandı'); return; }
    }
    const s = m.steps[m.i];
    if (s.type === 'goto') {
      const st = this.fleet.world.findStation(s.station);
      if (!st) { this.log('error', 'İstasyon yok: ' + s.station); m.i++; return; }
      if (this.goTo(st, { quiet: m.retried })) { m.state = 'moving'; m.retried = false; } else { m.state = 'retry'; m.waitLeft = 3; m.retried = true; }
    } else if (s.type === 'wait') {
      m.state = 'waiting'; m.waitLeft = Math.max(0, +s.sec || 0); this.status = `Bekliyor (${m.waitLeft.toFixed(0)} sn)`;
    } else if (s.type === 'load') {
      this.driver.payload = clamp(+s.kg || 0, 0, this.spec.maxPayload); this.log('info', `Yük alındı: ${this.driver.payload} kg`); m.i++;
    } else if (s.type === 'unload') {
      this.driver.payload = 0; this.log('info', 'Yük bırakıldı'); m.i++;
    }
  }

  navCmd(dt) {
    const r = follow(this.spec, this.pose, this.nav, this.speedScale);
    if (r.done) {
      const g = this.nav.goal;
      this.nav = null; this.pathSeq++;
      this.status = 'Vardı' + (g && g.name ? ': ' + g.name : '');
      this.log('ok', this.status);
      return ZERO;
    }
    return r;
  }

  // ---------- Guvenlik katmani: gidis yonundeki engele gore hiz siniri ----------
  applySafety(cmd, dt) {
    const sp = this.spec, sens = this.driver.sensors;
    this.safety = { state: 'ok', gap: null };
    if (!sp.safety.enabled) return cmd;
    const lin = Math.hypot(cmd.v || 0, cmd.vy || 0);
    if (lin < 0.02) return cmd;
    const dx = cmd.v / lin, dy = (cmd.vy || 0) / lin;
    const off = sp.kinematics === 'ackermann' ? sp.wheelBase / 2 : 0;
    const ext = Math.abs(dx) * sp.length / 2 + Math.abs(dy) * sp.width / 2;
    const lat = Math.abs(dx) * sp.width / 2 + Math.abs(dy) * sp.length / 2 + 0.08;
    let gap = Infinity;
    const test = (px, py) => {
      px -= off;
      const along = px * dx + py * dy;
      if (along <= 0) return;
      if (Math.abs(px * dy - py * dx) > lat) return;
      if (along - ext < gap) gap = along - ext;
    };
    const L = sens.lidar;
    if (L) for (let i = 0; i < L.ranges.length; i++) {
      const r = L.ranges[i]; if (r <= L.rangeMin) continue;
      const a = L.theta + L.angleMin + i * L.angleInc;
      test(L.x + r * Math.cos(a), L.y + r * Math.sin(a));
    }
    if (sens.ultrasonic) for (const u of sens.ultrasonic) if (u && u.d > 0.02) test(u.x + u.d * Math.cos(u.theta), u.y + u.d * Math.sin(u.theta));
    if (gap === Infinity) return cmd;
    gap = Math.max(0, gap);
    this.safety.gap = gap;
    const stop = sp.safety.stopDist;
    const a = sp.maxAccel * clamp(sp.mass / (sp.mass + (this.driver.payload || 0)) * 1.6, 0.25, 1);
    const allowed = gap <= stop ? 0 : Math.sqrt(2 * a * 0.6 * (gap - stop));
    if (allowed >= lin) return cmd;
    this.safety.state = allowed === 0 ? 'stop' : 'slow';
    const k = allowed / lin;
    return { v: cmd.v * k, vy: (cmd.vy || 0) * k, w: sp.kinematics === 'ackermann' ? cmd.w * k : cmd.w };
  }

  control(dt) {
    let cmd = ZERO;
    if (this.mode === 'manual') {
      if (Date.now() - this.teleop.t < 600) cmd = this.teleop;
    } else if (this.mode === 'auto') {
      this.missionTick(dt);
      if (this.nav) cmd = this.navCmd(dt);
    } else if (this.mode === 'script') {
      this.runScript(dt);
      cmd = this.nav ? this.navCmd(dt) : this.script.cmd;
    }
    const wanted = Math.hypot(cmd.v || 0, cmd.vy || 0);
    cmd = this.applySafety(cmd, dt);
    // Yol kapaliysa bekle, sonra LiDAR verisiyle yeniden planla
    if (this.nav && this.safety.state === 'stop' && wanted > 0.05) {
      this.blockedT += dt;
      if (this.blockedT > 3.5) {
        this.blockedT = 0;
        const g = this.nav.goal;
        if (this.goTo(g, { avoid: true, quiet: true })) this.log('info', 'Yol kapalı — alternatif rota planlandı');
        else this.status = 'Engel — bekliyor';
      }
    } else if (this.blockedT > 0) this.blockedT = Math.max(0, this.blockedT - dt);
    if (this.estop) cmd = ZERO;
    this.cmdOut = cmd;
    this.driver.send(cmd);
  }

  // ---------- Kod testi (kullanici betigi) ----------
  scriptLog(level, args) {
    const msg = args.map(a => typeof a === 'string' ? a : (() => { try { return JSON.stringify(a); } catch (e) { return String(a); } })()).join(' ').slice(0, 400);
    this.fleet.emit({ op: 'scriptLog', id: this.id, level, msg });
  }

  scriptApi() {
    const v = this, d = () => v.driver;
    const round = (o) => ({ x: +o.x.toFixed(4), y: +o.y.toFixed(4), theta: +o.theta.toFixed(4) });
    return {
      get pose() { return round(v.pose); },
      get odom() { return round(d().odom); },
      get velocity() { return { v: d().vel.v, vy: d().vel.vy, w: d().vel.w }; },
      get lidar() { return d().sensors.lidar || null; },
      get ultrasonic() { return (d().sensors.ultrasonic || []).map(u => u ? u.d : 0); },
      get line() { return d().sensors.line || null; },
      get camera() { const c = d().sensors.camera; return c && c[0] ? c[0].detections : []; },
      get imu() { return d().sensors.imu || null; },
      get gps() { return d().sensors.gps || null; },
      get terrain() { return d().terrain || null; },
      get bumper() { return !!d().sensors.bumper; },
      get encoders() { return d().sensors.encoders || []; },
      get battery() { return d().battery.soc == null ? null : +(d().battery.soc * 100).toFixed(1); },
      get payload() { return d().payload || 0; },
      get time() { return v.isSim ? v.fleet.simTime : Date.now() / 1000; },
      get stations() { return v.fleet.world.stations.map(s => ({ id: s.id, name: s.name, x: s.x, y: s.y, theta: s.theta, type: s.type })); },
      get navigating() { return !!v.nav; },
      get spec() { return JSON.parse(JSON.stringify(v.spec)); },
      setVelocity(vx, w, vy) { v.script.cmd = { v: +vx || 0, w: +w || 0, vy: +vy || 0 }; },
      stop() { v.script.cmd = ZERO; if (v.nav) { v.nav = null; v.pathSeq++; } },
      goTo(a, b, th) {
        if (typeof a === 'string') { const st = v.fleet.world.findStation(a); if (!st) { v.scriptLog('error', ['İstasyon yok: ' + a]); return false; } return v.goTo(st); }
        return v.goTo({ x: +a, y: +b, theta: th == null ? null : +th });
      },
      setPayload(kg) { d().payload = clamp(+kg || 0, 0, v.spec.maxPayload); },
      // Aracin onune gore [fromDeg, toDeg] acisal diliminde en yakin LiDAR mesafesi (m)
      lidarMin(fromDeg, toDeg) {
        const L = d().sensors.lidar; if (!L) return Infinity;
        let best = Infinity; const lo = fromDeg * Math.PI / 180, hi = toDeg * Math.PI / 180;
        for (let i = 0; i < L.ranges.length; i++) {
          const r = L.ranges[i]; if (r <= L.rangeMin) continue;
          let a = L.theta + L.angleMin + i * L.angleInc; a = Math.atan2(Math.sin(a), Math.cos(a));
          if (a >= lo && a <= hi && r < best) best = r;
        }
        return best;
      },
      log(...args) { v.scriptLog('info', args); },
    };
  }

  startScript(code) {
    this.stopScript();
    if (typeof code === 'string') this.spec.script = code;
    const api = this.scriptApi();
    const sandbox = { agv: api, ugv: api, robot: api, console: { log: (...a) => this.scriptLog('info', a), error: (...a) => this.scriptLog('error', a) }, __dt: 0 };
    try {
      const ctx = vm.createContext(sandbox);
      vm.runInContext(this.spec.script, ctx, { timeout: 500, filename: 'agv-script.js' });
      if (vm.runInContext('typeof loop', ctx) !== 'function') throw new Error('loop(agv, dt) fonksiyonu tanımlı değil');
      vm.runInContext('if (typeof setup === "function") setup(agv)', ctx, { timeout: 500 });
      this.script = { running: true, sandbox, ctx, call: new vm.Script('loop(agv, __dt)'), cmd: ZERO, acc: 0 };
    } catch (e) {
      this.scriptLog('error', ['Hata: ' + e.message]);
      return false;
    }
    if (this.mode !== 'script') this.setMode('script');
    this.status = 'Kod çalışıyor';
    this.scriptLog('ok', ['Kod başlatıldı']);
    return true;
  }

  stopScript() {
    if (this.script.running) this.scriptLog('info', ['Kod durduruldu']);
    this.script = { running: false, sandbox: null, ctx: null, call: null, cmd: ZERO, acc: 0 };
    if (this.mode === 'script') { if (this.nav) { this.nav = null; this.pathSeq++; } this.status = 'Kod modu'; }
  }

  runScript(dt) {
    const s = this.script;
    if (!s.running) return;
    s.acc += dt;
    if (s.acc < 0.05) return; // 20 Hz kontrol dongusu
    s.sandbox.__dt = s.acc; s.acc = 0;
    try { s.call.runInContext(s.ctx, { timeout: 60 }); }
    catch (e) { this.scriptLog('error', ['Çalışma hatası: ' + e.message]); this.stopScript(); }
  }

  // ---------- Istemciye gonderilen durum ----------
  state() {
    const d = this.driver, p = this.pose, f = n => (n == null || !isFinite(n)) ? null : +n.toFixed(3);
    const out = {
      id: this.id, hid: this.hidden, map: this.mapping, mapSrc: this.mapSource, x: f(p.x), y: f(p.y), th: f(p.theta), v: f(d.vel.v), vy: f(d.vel.vy), w: f(d.vel.w),
      cv: f(this.cmdOut.v), cvy: f(this.cmdOut.vy), cw: f(this.cmdOut.w),
      mode: this.mode, estop: this.estop, status: this.status, type: d.type, conn: d.connected, coll: !!d.collided,
      soc: f(d.battery.soc), volt: f(d.battery.voltage), chg: !!d.battery.charging, pw: f(d.battery.power),
      payload: d.payload || 0, odom: [f(d.odom.x), f(d.odom.y), f(d.odom.theta)], dist: f(d.distance),
      pathSeq: this.pathSeq, safe: this.safety.state, gap: f(this.safety.gap), run: this.script.running,
      mission: this.mission ? { i: this.mission.i, n: this.mission.steps.length, laps: this.mission.laps, state: this.mission.state } : null,
      ros: this.exposer ? this.exposer.connected : null, steer: d.steer != null ? f(d.steer) : undefined,
    };
    const S = d.sensors, sens = {};
    if (S.lidar && S.lidar.seq !== this._sentSeq) { this._sentSeq = S.lidar.seq; sens.lidar = S.lidar; }
    if (S.ultrasonic) sens.us = S.ultrasonic;
    if (S.bumper != null) sens.bumper = S.bumper;
    if (S.imu) sens.imu = S.imu;
    if (S.line) sens.line = S.line;
    if (S.camera) sens.cam = S.camera;
    if (S.encoders) sens.enc = S.encoders;
    if (S.gps) sens.gps = S.gps;
    if (d.terrain) out.terrain = d.terrain;
    out.sens = sens;
    return out;
  }
}

class Fleet {
  constructor(dataDir) {
    this.dataDir = dataDir;
    this.file = path.join(dataDir, 'fleet.json');
    this.vehicles = [];
    this.listeners = new Set();
    this.logs = [];
    this.sim = { running: true, speed: 1 };
    this.simTime = 0; this._acc = 0; this._last = Date.now();
    this.load();
  }

  emit(msg) { for (const l of this.listeners) l(msg); }
  log(level, msg, id) {
    const e = { op: 'log', t: Date.now(), level, msg, id: id || null };
    this.logs.push(e); if (this.logs.length > 300) this.logs.shift();
    this.emit(e);
  }

  load() {
    let data = null;
    try { data = JSON.parse(fs.readFileSync(this.file, 'utf8')); } catch (e) { /* ilk calistirma */ }
    // Haritalar: her harita kendi sahasini, arac baslangic konumlarini ve gizli arac listesini tutar
    let maps = data && Array.isArray(data.maps) && data.maps.length ? data.maps : null;
    if (!maps) maps = [data && data.world ? { id: 'm1', name: 'Haritam', world: data.world } : { id: 'm1', name: 'Tesis (fabrika + arazi)', world: defaults.world }];
    this.maps = maps.map((m, i) => ({ id: String(m.id || 'm' + (i + 1)), name: String(m.name || 'Harita ' + (i + 1)).slice(0, 40), world: m.world || {}, starts: m.starts || {}, hidden: Array.isArray(m.hidden) ? m.hidden : [] }));
    this.activeMap = data && this.maps.some(m => m.id === data.activeMap) ? data.activeMap : this.maps[0].id;
    const cur = this.curMap();
    this.world = new World(cur.world);
    this.mapper = new LidarMapper(this.world.width, this.world.height);
    for (const v of this.vehicles) v.dispose();
    this.vehicles = [];
    for (const s of (data && Array.isArray(data.vehicles) ? data.vehicles : defaults.vehicles)) {
      const v = this.addVehicle(s, true);
      v.hidden = cur.hidden.includes(v.id);
    }
  }

  curMap() { return this.maps.find(m => m.id === this.activeMap) || this.maps[0]; }
  mapList() { return this.maps.map(m => ({ id: m.id, name: m.name })); }
  emitMaps() { this.emit({ op: 'maps', maps: this.mapList(), active: this.activeMap }); }

  // Calisan durumu etkin harita kaydina yazar
  syncMap() {
    const m = this.curMap();
    m.world = this.world.toJSON();
    m.starts = {}; m.hidden = [];
    for (const v of this.vehicles) {
      const p = v.isSim ? v.spec.start : v.pose;
      m.starts[v.id] = { x: +p.x.toFixed(3), y: +p.y.toFixed(3), theta: +(+p.theta || 0).toFixed(4) };
      if (v.hidden) m.hidden.push(v.id);
    }
  }

  poseFree(spec, p, placed) {
    const r = reach(spec) + 0.2, w = this.world;
    if (p.x < r || p.y < r || p.x > w.width - r || p.y > w.height - r) return false;
    for (const o of w.obstacles) if (p.x > o.x - r && p.x < o.x + o.w + r && p.y > o.y - r && p.y < o.y + o.h + r) return false;
    for (const s of w.segments()) if (pointSegDist(p.x, p.y, s) < r) return false;
    const fp = footprint(spec, p);
    for (const e of fp) for (const s of w.segments()) if (segSeg(e, s)) return false;
    for (const q of placed) if (Math.hypot(q.x - p.x, q.y - p.y) < r + q.r + 0.4) return false;
    return true;
  }

  // Haritada konumu olmayan arac icin bos bir yer bulur
  autoPlace(spec, placed) {
    // Sol kenardan baslayip yukari dogru dizer: hepsi +x'e bakar, kimse kimsenin onunde durmaz
    for (let x = 1.5; x < this.world.width - 1; x += 0.75)
      for (let y = 1.5; y < this.world.height - 1; y += 0.75) { const p = { x, y, theta: 0 }; if (this.poseFree(spec, p, placed)) return p; }
    return { x: this.world.width / 2, y: this.world.height / 2, theta: 0 };
  }

  // Etkin haritayi yukler: saha + araclarin bu haritadaki konumu / gizliligi
  loadMapState() {
    const m = this.curMap();
    this.world.load(m.world); this.world.resetDynamics(); this.simTime = 0;
    this.mapper.reset(this.world.width, this.world.height);
    const placed = [];
    const order = this.vehicles.slice().sort((a, b) => (m.starts[b.id] ? 1 : 0) - (m.starts[a.id] ? 1 : 0));
    for (const v of order) {
      v.cancel(); v.stopScript(); v.estop = false; v.setMode('manual');
      v.hidden = m.hidden.includes(v.id);
      let p = m.starts[v.id];
      if (!p || !this.poseFree(v.spec, p, placed)) p = this.autoPlace(v.spec, placed);
      placed.push({ x: p.x, y: p.y, r: reach(v.spec) });
      if (v.isSim) { v.spec.start = { ...p }; v.driver.reset(p); } else v.driver.setPose(p);
    }
    this.emit({ op: 'world', world: this.world.toJSON() }); this.emitSpecs(); this.emitMaps();
  }

  selectMap(id) {
    if (id === this.activeMap || !this.maps.some(m => m.id === id)) return;
    this.syncMap(); this.activeMap = id; this.loadMapState();
    this.log('info', 'Harita açıldı: ' + this.curMap().name);
  }

  newMap(name, world, copy) {
    this.syncMap();
    const cur = this.curMap();
    const m = { id: 'm' + Date.now().toString(36), name: String(name || 'Yeni harita').slice(0, 40), world: copy ? JSON.parse(JSON.stringify(cur.world)) : new World(world || {}).toJSON(), starts: copy ? JSON.parse(JSON.stringify(cur.starts)) : {}, hidden: copy ? cur.hidden.slice() : [] };
    this.maps.push(m); this.activeMap = m.id; this.loadMapState();
    this.log('ok', (copy ? 'Harita çoğaltıldı: ' : 'Yeni harita oluşturuldu: ') + m.name);
  }

  renameMap(id, name) {
    const m = this.maps.find(x => x.id === id); if (!m || !String(name || '').trim()) return;
    m.name = String(name).trim().slice(0, 40); this.emitMaps();
  }

  deleteMap(id) {
    const m = this.maps.find(x => x.id === id); if (!m) return;
    if (this.maps.length < 2) return this.log('warn', 'Son harita silinemez — önce yeni bir harita oluşturun');
    this.maps = this.maps.filter(x => x !== m);
    if (this.activeMap === id) { this.activeMap = this.maps[0].id; this.loadMapState(); } else this.emitMaps();
    this.log('info', 'Harita silindi: ' + m.name);
  }

  setHidden(v, on) {
    v.hidden = !!on;
    if (on) { v.cancel(); v.stopScript(); v.cmdOut = ZERO; v.driver.send(ZERO); if (v.driver.vel) v.driver.vel = { v: 0, vy: 0, w: 0 }; if (v.driver.drv) v.driver.drv = { v: 0, vy: 0, w: 0 }; v.status = 'Gizli'; }
    else { v.status = 'Hazır'; if (v.isSim && !this.poseFree(v.spec, v.pose, this.vehicles.filter(o => o !== v && !o.hidden).map(o => ({ x: o.pose.x, y: o.pose.y, r: reach(o.spec) })))) v.driver.setPose(this.autoPlace(v.spec, this.vehicles.filter(o => o !== v && !o.hidden).map(o => ({ x: o.pose.x, y: o.pose.y, r: reach(o.spec) })))); }
    this.log('info', on ? 'Araç gizlendi (devre dışı)' : 'Araç yeniden etkin', v.id);
  }

  save() {
    try {
      fs.mkdirSync(this.dataDir, { recursive: true });
      this.syncMap();
      fs.writeFileSync(this.file, JSON.stringify({ maps: this.maps, activeMap: this.activeMap, vehicles: this.vehicles.map(v => v.spec) }, null, 2));
      this.log('ok', 'Proje kaydedildi: ' + this.file);
    } catch (e) { this.log('error', 'Kaydedilemedi: ' + e.message); }
  }

  factoryReset() {
    try { fs.unlinkSync(this.file); } catch (e) { }
    this.load(); this.simTime = 0;
    this.mapper.reset(this.world.width, this.world.height);
    this.emit({ op: 'world', world: this.world.toJSON() }); this.emitSpecs(); this.emitMaps();
    this.log('info', 'Varsayılan proje yüklendi');
  }

  get(id) { return this.vehicles.find(v => v.id === id); }
  specs() { return this.vehicles.map(v => v.spec); }
  emitSpecs() { this.emit({ op: 'specs', vehicles: this.specs() }); }

  uniqueId(baseName) {
    const stem = String(baseName || 'agv').toLowerCase().replace(/[^a-z0-9]+/g, '').slice(0, 12) || 'agv';
    let i = 1, id = stem;
    while (this.get(id)) id = stem + (++i);
    return id;
  }

  addVehicle(spec, silent) {
    const s = { ...spec };
    if (!s.id || this.get(s.id)) s.id = this.uniqueId(s.id || s.name);
    const v = new Vehicle(s, this);
    this.vehicles.push(v);
    if (!silent) { this.emitSpecs(); this.log('info', 'Araç eklendi: ' + v.spec.name, v.id); }
    return v;
  }

  updateVehicle(id, spec) {
    const old = this.get(id);
    if (!old) return;
    const pose = { ...old.pose }, mode = old.mode, payload = old.driver.payload, soc = old.driver.battery.soc;
    const wasConn = old.driver.wantConnected;
    old.dispose();
    const v = new Vehicle({ ...spec, id }, this);
    v.driver.setPose(pose);
    if (v.isSim) { v.driver.battery.soc = soc == null ? 1 : soc; v.driver.payload = clamp(payload || 0, 0, v.spec.maxPayload); }
    else if (wasConn) v.driver.connect();
    v.mode = mode === 'script' ? 'manual' : mode; v.hidden = old.hidden; v.mapping = old.mapping; v.mapSource = old.mapSource;
    this.vehicles[this.vehicles.indexOf(old)] = v;
    this.emitSpecs();
    this.log('info', 'Tasarım güncellendi', id);
  }

  removeVehicle(id) {
    const v = this.get(id); if (!v) return;
    v.dispose(); this.vehicles = this.vehicles.filter(x => x !== v);
    this.emitSpecs(); this.log('info', 'Araç silindi: ' + v.spec.name);
  }

  setWorld(data) {
    const ow = this.world.width, oh = this.world.height;
    this.world.load(data);
    if (this.world.width !== ow || this.world.height !== oh) this.mapper.reset(this.world.width, this.world.height);
    for (const v of this.vehicles) if (v.nav) { const g = v.nav.goal; if (!v.goTo(g, { quiet: true })) { v.nav = null; v.pathSeq++; } }
    this.emit({ op: 'world', world: this.world.toJSON() });
  }

  resetSim() {
    this.world.resetDynamics(); this.simTime = 0;
    for (const v of this.vehicles) {
      v.cancel(); v.estop = false;
      if (v.isSim) { const run = v.script.running && !v.hidden; v.driver.reset(v.spec.start); if (run) v.startScript(); }
    }
    this.log('info', 'Simülasyon sıfırlandı');
  }

  // Haritalama acik araclarin yeni LiDAR taramalarini ortak izgaraya isler
  mapScan(v) {
    if (!v.mapping || v.hidden) return;
    const L = v.driver.sensors.lidar;
    if (!L || L.seq === v._mapSeq) return;
    v._mapSeq = L.seq;
    this.mapper.integrate(v.isSim && v.mapSource === 'odom' ? v.driver.odom : v.pose, L);
  }

  othersOf(v) {
    const segs = [];
    for (const o of this.vehicles) if (o !== v && !o.hidden) segs.push(...footprint(o.spec, o.pose));
    return segs;
  }

  simStep(dt) {
    this.world.step(dt);
    for (const v of this.vehicles) {
      if (!v.isSim || v.hidden) continue;
      v.control(dt);
      v.driver.step(dt, this.world, this.othersOf(v));
      this.mapScan(v);
    }
    this.simTime += dt;
  }

  // 50 Hz ana dongu
  tick() {
    const now = Date.now(), wall = Math.min(0.2, (now - this._last) / 1000);
    this._last = now;
    if (this.sim.running) {
      this._acc += this.sim.speed;
      let n = 0;
      while (this._acc >= 1 && n++ < 16) { this.simStep(0.02); this._acc -= 1; }
    } else {
      for (const v of this.vehicles) if (v.isSim) v.driver.send(ZERO);
    }
    for (const v of this.vehicles) {
      if (!v.isSim) { if (v.hidden) v.driver.send(ZERO); else v.control(wall); v.driver.step(wall); this.mapScan(v); }
      if (v.exposer) v.exposer.tick();
    }
  }

  snapshot() {
    return { op: 'state', t: +this.simTime.toFixed(2), sim: this.sim, dyn: this.world.dynamicState(), vehicles: this.vehicles.map(v => v.state()) };
  }
  paths() { return this.vehicles.map(v => ({ id: v.id, seq: v.pathSeq, path: v.nav ? v.nav.path.map(p => [+p.x.toFixed(2), +p.y.toFixed(2)]) : null })); }

  start() { this.timer = setInterval(() => this.tick(), 20); }
  stop() { clearInterval(this.timer); for (const v of this.vehicles) v.dispose(); }
}

module.exports = { Fleet, Vehicle, normalize };
