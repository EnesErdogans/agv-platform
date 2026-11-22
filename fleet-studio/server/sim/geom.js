'use strict';
// Temel 2B geometri yardimcilari

const TAU = Math.PI * 2;

function wrap(a) {
  while (a > Math.PI) a -= TAU;
  while (a < -Math.PI) a += TAU;
  return a;
}

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

function gauss(sigma = 1) {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return sigma * Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * v);
}

// Isin (o + t*d) ile dogru parcasi kesisimi; t (mesafe) ya da Infinity
function raySeg(ox, oy, dx, dy, s) {
  const ex = s.x2 - s.x1, ey = s.y2 - s.y1;
  const den = dx * ey - dy * ex;
  if (Math.abs(den) < 1e-12) return Infinity;
  const t = ((s.x1 - ox) * ey - (s.y1 - oy) * ex) / den;
  const u = ((s.x1 - ox) * dy - (s.y1 - oy) * dx) / den;
  return (t >= 0 && u >= 0 && u <= 1) ? t : Infinity;
}

function orient(ax, ay, bx, by, cx, cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

function segSeg(a, b) {
  const d1 = orient(b.x1, b.y1, b.x2, b.y2, a.x1, a.y1);
  const d2 = orient(b.x1, b.y1, b.x2, b.y2, a.x2, a.y2);
  const d3 = orient(a.x1, a.y1, a.x2, a.y2, b.x1, b.y1);
  const d4 = orient(a.x1, a.y1, a.x2, a.y2, b.x2, b.y2);
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

function pointSegDist(px, py, s) {
  const ex = s.x2 - s.x1, ey = s.y2 - s.y1;
  const l2 = ex * ex + ey * ey;
  let t = l2 > 0 ? ((px - s.x1) * ex + (py - s.y1) * ey) / l2 : 0;
  t = clamp(t, 0, 1);
  return Math.hypot(px - (s.x1 + t * ex), py - (s.y1 + t * ey));
}

// Govde cercevesindeki noktayi dunya cercevesine tasir
function toWorld(pose, bx, by) {
  const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
  return { x: pose.x + c * bx - s * by, y: pose.y + s * bx + c * by };
}

// Aracin dikdortgen ayak izi (4 kose + 4 kenar)
function footprint(spec, pose) {
  const off = spec.kinematics === 'ackermann' ? (spec.wheelBase || 0) / 2 : 0;
  const hl = spec.length / 2, hw = spec.width / 2;
  const pts = [[off + hl, hw], [off + hl, -hw], [off - hl, -hw], [off - hl, hw]].map(p => toWorld(pose, p[0], p[1]));
  const segs = [];
  for (let i = 0; i < 4; i++) {
    const a = pts[i], b = pts[(i + 1) % 4];
    segs.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y });
  }
  return segs;
}

function robotRadius(spec) {
  return 0.5 * Math.hypot(spec.length, spec.width) * 0.85 + 0.08;
}

module.exports = { TAU, wrap, clamp, gauss, raySeg, segSeg, pointSegDist, toWorld, footprint, robotRadius };
