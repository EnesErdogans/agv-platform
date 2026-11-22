'use strict';
const { wrap, clamp, gauss, segSeg, pointSegDist, toWorld, footprint, TAU } = require('../sim/geom');

const DEG = Math.PI / 180;
const RATES = { lidar: 8, ultrasonic: 15, line: 40, camera: 10, imu: 50, bumper: 50, gps: 5 };

// Simule arac: kinematik + dinamik sinirlar + sensor modelleri + batarya
class SimDriver {
  constructor(spec, fleet) {
    this.type = 'sim';
    this.spec = spec; this.fleet = fleet;
    this.connected = true;
    this.cmd = { v: 0, vy: 0, w: 0 };
    this.reset(spec.start || { x: 2, y: 2, theta: 0 });
  }

  reset(p) {
    this.pose = { x: +p.x, y: +p.y, theta: +p.theta || 0 };
    this.odom = { ...this.pose };
    this.vel = { v: 0, vy: 0, w: 0 };   // gercek govde hizi
    this.drv = { v: 0, vy: 0, w: 0 };   // tahrik (teker / palet) hizi; kayma oncesi
    this.terrain = null;
    this.steer = 0;
    this.collided = false;
    this.payload = this.spec.payload || 0;
    this.battery = { soc: 1, voltage: (this.spec.battery && this.spec.battery.voltage) || 24, charging: false, power: 0 };
    this.encoders = [0, 0, 0, 0];
    this.imuBias = gauss(0.002);
    this.imuYaw = this.pose.theta;
    this.odoErr = { v: 1 + gauss(0.008), w: 1 + gauss(0.015) }; // sistematik odometri hatasi
    this.sensors = {};
    this._acc = {};
    this.scanSeq = 0;
    this.distance = 0;
  }

  connect() { this.connected = true; }
  disconnect() { }
  send(cmd) { this.cmd = cmd; }
  setPose(p) { this.pose = { x: p.x, y: p.y, theta: p.theta }; this.odom = { ...this.pose }; this.imuYaw = p.theta; this.vel = { v: 0, vy: 0, w: 0 }; this.drv = { v: 0, vy: 0, w: 0 }; this.collided = false; }

  collides(pose, world, others) {
    const fp = footprint(this.spec, pose);
    const segs = world.segments();
    for (const e of fp) {
      for (let i = 0; i < segs.length; i++) if (segSeg(e, segs[i])) return true;
      for (let i = 0; i < others.length; i++) if (segSeg(e, others[i])) return true;
    }
    return false;
  }

  step(dt, world, others) {
    const sp = this.spec, c = this.cmd, vel = this.vel, drv = this.drv;
    const dead = this.battery.soc <= 0;
    const kin = sp.kinematics, skid = kin === 'skid';
    // Zemin etkisi (arazi kabiliyeti yuksek araclar daha az etkilenir)
    const z = world.terrainAt(this.pose.x, this.pose.y), off = clamp(sp.offroad || 0, 0, 1);
    const zSpeed = z ? 1 - (1 - z.speed) * (1 - 0.85 * off) : 1;
    const zSlip = z ? z.slip * (1 - 0.7 * off) : 0;
    const zDrag = z ? 1 + (z.drag - 1) * (1 - 0.6 * off) : 1;
    this.terrain = z ? z.type : null;

    const massRatio = clamp(sp.mass / (sp.mass + this.payload) * 1.6, 0.25, 1);
    const aMax = sp.maxAccel * massRatio;
    const tau = Math.max(0.02, sp.motorTau || 0.15);
    const lag = (cur, tgt, lim) => cur + clamp((tgt - cur) / tau, -lim, lim) * dt;
    const vMax = sp.maxSpeed * zSpeed;
    const tv = dead ? 0 : clamp(c.v || 0, -vMax, vMax);
    const tw = dead ? 0 : clamp(c.w || 0, -sp.maxAngular, sp.maxAngular);
    const tvy = dead || kin !== 'mecanum' ? 0 : clamp(c.vy || 0, -vMax, vMax);

    drv.v = lag(drv.v, tv, aMax);
    drv.vy = lag(drv.vy, tvy, aMax);
    vel.v = drv.v * (1 - zSlip); vel.vy = drv.vy * (1 - zSlip);
    if (kin === 'ackermann') {
      const L = sp.wheelBase || 1, maxSt = (sp.maxSteer || 30) * DEG;
      const vref = Math.abs(tv) > 0.03 ? tv : (Math.abs(drv.v) > 0.03 ? drv.v : 0);
      const want = vref !== 0 ? clamp(Math.atan(tw * L / vref), -maxSt, maxSt) : this.steer;
      this.steer += clamp(want - this.steer, -1.2 * dt, 1.2 * dt); // direksiyon hizi ~70 derece/s
      drv.w = drv.v * Math.tan(this.steer) / L;
      vel.w = vel.v * Math.tan(this.steer) / L;
    } else {
      drv.w = lag(drv.w, tw, (sp.maxAngAccel || 3) * massRatio);
      // Palet / skid-steer: donuste paletler kayar, govde komuttan daha az doner
      const turnSlip = skid ? clamp((sp.turnSlip == null ? 0.15 : sp.turnSlip) + zSlip, 0, 0.9) : zSlip * 0.5;
      vel.w = drv.w * (1 - turnSlip);
    }

    // Poz entegrasyonu + carpisma kontrolu
    const th = this.pose.theta, co = Math.cos(th), si = Math.sin(th);
    const np = {
      x: this.pose.x + (vel.v * co - vel.vy * si) * dt,
      y: this.pose.y + (vel.v * si + vel.vy * co) * dt,
      theta: wrap(th + vel.w * dt),
    };
    if (this.collides(np, world, others)) {
      if (!this.collided) this.fleet.log('warn', 'Çarpışma algılandı', sp.id);
      this.collided = true;
      const tOnly = { x: np.x, y: np.y, theta: th }, rOnly = { x: this.pose.x, y: this.pose.y, theta: np.theta };
      if (Math.abs(vel.w) > 1e-3 && !this.collides(rOnly, world, others)) { this.pose = rOnly; vel.v = vel.vy = 0; drv.v = drv.vy = 0; }
      else if (!this.collides(tOnly, world, others)) { this.distance += Math.hypot(np.x - this.pose.x, np.y - this.pose.y); this.pose = tOnly; vel.w = 0; drv.w = 0; }
      else { vel.v = vel.vy = vel.w = 0; drv.v = drv.vy = drv.w = 0; }
    } else {
      this.distance += Math.hypot(np.x - this.pose.x, np.y - this.pose.y);
      this.pose = np;
      if (Math.abs(vel.v) + Math.abs(vel.vy) + Math.abs(vel.w) > 0.02) this.collided = false;
    }

    // Enkoder + odometri. Enkoder yoksa (ticksPerRev = 0) odometri komuttan acik cevrim hesaplanir.
    const r = sp.wheelRadius || 0.08, T = sp.trackWidth || sp.width, tpr = sp.encoder ? +sp.encoder.ticksPerRev || 0 : 0;
    this.hasEncoders = tpr > 0;
    let ov, ovy, ow;
    if (this.hasEncoders) {
      const k = tpr / (TAU * r) * dt;
      if (kin === 'mecanum') {
        const a = ((sp.wheelBase || sp.length * 0.7) + T) / 2;
        const ws = [drv.v - drv.vy - a * drv.w, drv.v + drv.vy + a * drv.w, drv.v + drv.vy - a * drv.w, drv.v - drv.vy + a * drv.w];
        for (let i = 0; i < 4; i++) this.encoders[i] += ws[i] * k;
      } else {
        this.encoders[0] += (drv.v - drv.w * T / 2) * k;
        this.encoders[1] += (drv.v + drv.w * T / 2) * k;
      }
      const en = sp.encoder.noise == null ? 0.01 : sp.encoder.noise;
      ov = drv.v * this.odoErr.v * (1 + gauss(en)); ovy = drv.vy * this.odoErr.v; ow = drv.w * this.odoErr.w * (1 + gauss(en));
    } else {
      ov = tv * this.odoErr.v * 1.05; ovy = tvy * this.odoErr.v; ow = (kin === 'ackermann' ? drv.w : tw) * this.odoErr.w * 1.05;
    }
    const oc = Math.cos(this.odom.theta), os = Math.sin(this.odom.theta);
    this.odom.x += (ov * oc - ovy * os) * dt; this.odom.y += (ov * os + ovy * oc) * dt; this.odom.theta = wrap(this.odom.theta + ow * dt);

    // Batarya
    const b = this.battery, cap = (sp.battery && sp.battery.capacityWh) || 500;
    const mTot = sp.mass + this.payload, speed = Math.hypot(vel.v, vel.vy), dspeed = Math.hypot(drv.v, drv.vy);
    const power = (sp.idlePower || 12) + (mTot * 9.81 * 0.025 * zDrag * dspeed + (skid ? 0.25 : 0.06) * mTot * Math.abs(drv.w) * zDrag) / 0.7;
    const st = world.stations.find(s => s.type === 'charger' && Math.hypot(s.x - this.pose.x, s.y - this.pose.y) < 0.5);
    b.charging = !!st && speed < 0.05;
    b.power = b.charging ? -(sp.battery && sp.battery.chargeW || 300) : power;
    b.soc = clamp(b.soc - b.power * dt / 3600 / cap, 0, 1);
    const nominal = (sp.battery && sp.battery.voltage) || 24;
    b.voltage = nominal * (0.86 + 0.19 * b.soc);

    this.updateSensors(dt, world, others);
  }

  due(key, rate, dt) {
    this._acc[key] = (this._acc[key] || 0) + dt;
    const period = 1 / rate;
    if (this._acc[key] >= period) { this._acc[key] %= period; return true; }
    return false;
  }

  updateSensors(dt, world, others) {
    const sp = this.spec, S = this.sensors;
    const us = [], cams = [];
    let lidarDone = false;
    (sp.sensors || []).forEach((sn, idx) => {
      const rate = sn.rate || RATES[sn.type] || 10, key = sn.type + idx;
      const mount = toWorld(this.pose, sn.x || 0, sn.y || 0), mth = this.pose.theta + (sn.theta || 0) * DEG;
      if (sn.type === 'lidar') {
        if (lidarDone) return; lidarDone = true;
        if (!this.due(key, rate, dt)) return;
        const n = sn.rays || 360, fov = (sn.fov || 360) * DEG, range = sn.range || 12, noise = sn.noise == null ? 0.01 : sn.noise;
        const a0 = -fov / 2, inc = fov / (fov >= TAU - 1e-6 ? n : Math.max(1, n - 1));
        const ranges = new Array(n);
        for (let i = 0; i < n; i++) {
          const d = world.raycast(mount.x, mount.y, mth + a0 + i * inc, range, others);
          ranges[i] = d === Infinity ? 0 : Math.max(0.02, +(d + gauss(noise)).toFixed(3));
        }
        S.lidar = { seq: ++this.scanSeq, angleMin: a0, angleInc: inc, range, rangeMin: sn.rangeMin || 0.15, ranges, x: sn.x || 0, y: sn.y || 0, theta: (sn.theta || 0) * DEG, name: sn.name || 'lidar' };
      } else if (sn.type === 'ultrasonic') {
        const slot = us.length; us.push(null);
        if (!this.due(key, rate, dt)) { us[slot] = (S.ultrasonic && S.ultrasonic[slot]) || { d: 0 }; return; }
        const range = sn.range || 4, cone = (sn.cone || 20) * DEG;
        let best = Infinity;
        for (const a of [-cone / 2, 0, cone / 2]) best = Math.min(best, world.raycast(mount.x, mount.y, mth + a, range, others));
        us[slot] = { d: best === Infinity ? 0 : +(best + gauss(0.01)).toFixed(3), x: sn.x || 0, y: sn.y || 0, theta: (sn.theta || 0) * DEG, range, name: sn.name || ('us' + slot) };
      } else if (sn.type === 'bumper') {
        S.bumper = this.collided;
      } else if (sn.type === 'imu') {
        if (!this.due(key, rate, dt)) return;
        const yr = this.vel.w + this.imuBias + gauss(sn.noise == null ? 0.004 : sn.noise);
        this.imuYaw = wrap(this.imuYaw + yr / rate);
        S.imu = { yawRate: +yr.toFixed(4), yaw: +this.imuYaw.toFixed(4), heading: +wrap(this.pose.theta + gauss(0.03)).toFixed(4) }; // heading: pusula (kaymaz, gurultulu)
      } else if (sn.type === 'gps') {
        if (!this.due(key, rate, dt)) return;
        if (world.roofAt(this.pose.x, this.pose.y)) { S.gps = { fix: false }; return; } // kapali alanda uydu yok
        const n = sn.noise == null ? 0.6 : sn.noise;
        const gx = this.pose.x + gauss(n), gy = this.pose.y + gauss(n), ll = world.toLatLon(gx, gy);
        S.gps = { fix: true, x: +gx.toFixed(2), y: +gy.toFixed(2), lat: +ll.lat.toFixed(7), lon: +ll.lon.toFixed(7), hdop: +(0.7 + n).toFixed(1) };
      } else if (sn.type === 'line') {
        if (!this.due(key, rate, dt)) return;
        const n = sn.count || 8, spacing = sn.spacing || 0.02, bits = [];
        let sum = 0, cnt = 0;
        for (let i = 0; i < n; i++) {
          const off = (i - (n - 1) / 2) * spacing;
          const p = toWorld(this.pose, sn.x || 0, off);
          let on = 0;
          for (const t of world.tapes) if (pointSegDist(p.x, p.y, t) < 0.025) { on = 1; break; }
          bits.push(on); if (on) { sum += off; cnt++; }
        }
        S.line = { bits, offset: cnt ? +(sum / cnt).toFixed(4) : null, x: sn.x || 0, spacing };
      } else if (sn.type === 'camera') {
        const slot = cams.length; cams.push(null);
        if (!this.due(key, rate, dt)) { cams[slot] = (S.camera && S.camera[slot]) || { detections: [] }; return; }
        const fov = (sn.fov || 76) * DEG, range = sn.range || 6, det = [];
        const see = (kind, name, x, y) => {
          const dx = x - mount.x, dy = y - mount.y, dist = Math.hypot(dx, dy);
          if (dist > range || dist < 0.05) return;
          const bearing = wrap(Math.atan2(dy, dx) - mth);
          if (Math.abs(bearing) > fov / 2) return;
          if (world.raycast(mount.x, mount.y, Math.atan2(dy, dx), dist - 0.08, null) !== Infinity && kind !== 'obstacle') return;
          det.push({ kind, name, bearing: +bearing.toFixed(3), distance: +dist.toFixed(2) });
        };
        for (const st of world.stations) see('station', st.name, st.x, st.y);
        for (const v of this.fleet.vehicles) if (v.spec.id !== sp.id && !v.hidden) see('vehicle', v.spec.name, v.pose.x, v.pose.y);
        for (const o of world.obstacles) {
          const cx = o.cx + o.w / 2, cy = o.cy + o.h / 2, dist = Math.hypot(cx - mount.x, cy - mount.y);
          const ang = Math.atan2(cy - mount.y, cx - mount.x), hit = world.raycast(mount.x, mount.y, ang, range, null);
          if (hit !== Infinity && hit <= dist) see('obstacle', o.move ? 'hareketli engel' : 'engel', mount.x + Math.cos(ang) * hit, mount.y + Math.sin(ang) * hit);
        }
        cams[slot] = { detections: det, x: sn.x || 0, y: sn.y || 0, theta: (sn.theta || 0) * DEG, fov, range, name: sn.name || 'kamera' };
      }
    });
    if (us.length) S.ultrasonic = us;
    if (cams.length) S.camera = cams;
    if (this.hasEncoders) S.encoders = this.encoders.slice(0, sp.kinematics === 'mecanum' ? 4 : 2).map(Math.round); else delete S.encoders;
  }
}

module.exports = { SimDriver };
