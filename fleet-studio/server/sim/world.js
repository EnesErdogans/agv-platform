'use strict';
const { raySeg } = require('./geom');

// Zemin tipleri: hiz carpani, kayma, surukleme (enerji), planlayici maliyeti
const TERRAIN = {
  grass: { speed: 0.8, slip: 0.08, drag: 1.8, cost: 1.6 },
  gravel: { speed: 0.85, slip: 0.12, drag: 1.5, cost: 1.4 },
  sand: { speed: 0.6, slip: 0.25, drag: 2.8, cost: 3 },
  mud: { speed: 0.45, slip: 0.3, drag: 3.5, cost: 4 },
};

// Saha: bina ici (fabrika) + dis alan (arazi): duvarlar, engeller (sabit / hareketli), istasyonlar, serit (cizgi) yollari
class World {
  constructor(data) { this.version = 0; this.load(data || {}); }

  load(d) {
    this.width = +d.width || 30;
    this.height = +d.height || 18;
    this.walls = (d.walls || []).map(w => ({ x1: +w.x1, y1: +w.y1, x2: +w.x2, y2: +w.y2 }));
    this.obstacles = (d.obstacles || []).map((o, i) => ({
      id: o.id || 'o' + i, x: +o.x, y: +o.y, w: +o.w, h: +o.h,
      move: o.move ? { dx: +o.move.dx || 0, dy: +o.move.dy || 0, speed: +o.move.speed || 0.5 } : null,
      cx: +o.x, cy: +o.y, _p: 0, _dir: 1,
    }));
    this.stations = (d.stations || []).map((s, i) => ({
      id: s.id || 's' + i, name: s.name || ('İstasyon ' + (i + 1)), x: +s.x, y: +s.y,
      theta: s.theta == null ? null : +s.theta, type: s.type || 'station',
    }));
    this.tapes = (d.tapes || []).map(t => ({ x1: +t.x1, y1: +t.y1, x2: +t.x2, y2: +t.y2 }));
    // zones: { type: 'roof' | 'grass' | 'gravel' | 'sand' | 'mud', x, y, w, h }  (roof = kapali alan, GPS yok)
    this.zones = (d.zones || []).map(z => ({ type: z.type, x: +z.x, y: +z.y, w: +z.w, h: +z.h })).filter(z => z.type === 'roof' || TERRAIN[z.type]);
    this.origin = { lat: 39.0, lon: 35.0, ...(d.origin || {}) };
    this._segs = null;
    this.version++;
  }

  terrainAt(x, y) {
    let t = null;
    for (const z of this.zones) if (z.type !== 'roof' && x >= z.x && x <= z.x + z.w && y >= z.y && y <= z.y + z.h) t = z.type;
    return t ? { type: t, ...TERRAIN[t] } : null;
  }
  roofAt(x, y) { return this.zones.some(z => z.type === 'roof' && x >= z.x && x <= z.x + z.w && y >= z.y && y <= z.y + z.h); }
  toLatLon(x, y) { return { lat: this.origin.lat + y / 111320, lon: this.origin.lon + x / (111320 * Math.cos(this.origin.lat * Math.PI / 180)) }; }

  toJSON() {
    return {
      width: this.width, height: this.height, walls: this.walls,
      obstacles: this.obstacles.map(o => ({ id: o.id, x: o.x, y: o.y, w: o.w, h: o.h, move: o.move || undefined })),
      stations: this.stations, tapes: this.tapes, zones: this.zones, origin: this.origin,
    };
  }

  static rectSegs(x, y, w, h) {
    return [
      { x1: x, y1: y, x2: x + w, y2: y }, { x1: x + w, y1: y, x2: x + w, y2: y + h },
      { x1: x + w, y1: y + h, x2: x, y2: y + h }, { x1: x, y1: y + h, x2: x, y2: y },
    ];
  }

  // Tum carpisma / isin kenarlari (dis sinir + duvarlar + engeller)
  segments() {
    if (this._segs) return this._segs;
    const s = World.rectSegs(0, 0, this.width, this.height).concat(this.walls);
    for (const o of this.obstacles) s.push(...World.rectSegs(o.cx, o.cy, o.w, o.h));
    this._segs = s;
    return s;
  }

  // Yalnizca sabit geometri (yol planlayici icin)
  staticSegments() {
    const s = World.rectSegs(0, 0, this.width, this.height).concat(this.walls);
    for (const o of this.obstacles) if (!o.move) s.push(...World.rectSegs(o.x, o.y, o.w, o.h));
    return s;
  }

  step(dt) {
    let moved = false;
    for (const o of this.obstacles) {
      if (!o.move) continue;
      const len = Math.hypot(o.move.dx, o.move.dy);
      if (len < 1e-6) continue;
      o._p += o._dir * o.move.speed * dt / len;
      if (o._p > 1) { o._p = 1; o._dir = -1; } else if (o._p < 0) { o._p = 0; o._dir = 1; }
      o.cx = o.x + o.move.dx * o._p; o.cy = o.y + o.move.dy * o._p;
      moved = true;
    }
    if (moved) this._segs = null;
  }

  resetDynamics() {
    for (const o of this.obstacles) { o._p = 0; o._dir = 1; o.cx = o.x; o.cy = o.y; }
    this._segs = null;
  }

  dynamicState() {
    return this.obstacles.filter(o => o.move).map(o => ({ id: o.id, x: +o.cx.toFixed(3), y: +o.cy.toFixed(3) }));
  }

  // (ox,oy) noktasindan ang yonunde en yakin kenara mesafe; yoksa Infinity
  raycast(ox, oy, ang, maxRange, extraSegs) {
    const dx = Math.cos(ang), dy = Math.sin(ang);
    let best = maxRange;
    const segs = this.segments();
    for (let i = 0; i < segs.length; i++) {
      const t = raySeg(ox, oy, dx, dy, segs[i]);
      if (t < best) best = t;
    }
    if (extraSegs) for (let i = 0; i < extraSegs.length; i++) {
      const t = raySeg(ox, oy, dx, dy, extraSegs[i]);
      if (t < best) best = t;
    }
    return best < maxRange ? best : Infinity;
  }

  findStation(key) {
    if (key == null) return null;
    const k = String(key).toLowerCase();
    return this.stations.find(s => s.id === key) || this.stations.find(s => s.name.toLowerCase() === k) || null;
  }
}

module.exports = { World, TERRAIN };
