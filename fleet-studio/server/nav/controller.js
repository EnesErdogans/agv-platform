'use strict';
const { wrap, clamp } = require('../sim/geom');

// Pure-pursuit yol takipcisi. nav: { path, cum, idx, goalTheta, rotating }
// Donus: { v, vy, w, done }
function follow(spec, pose, nav, speedScale = 1) {
  const path = nav.path, last = path.length - 1;
  const d = i => Math.hypot(path[i].x - pose.x, path[i].y - pose.y);
  while (nav.idx < last && (d(nav.idx + 1) <= d(nav.idx) || d(nav.idx) < 0.25)) nav.idx++;
  const remaining = d(nav.idx) + (nav.cum[last] - nav.cum[nav.idx]);
  const kin = spec.kinematics;
  const tol = kin === 'ackermann' ? 0.18 : 0.07;

  if (nav.idx >= last && d(last) < tol) {
    if (nav.goalTheta != null && kin !== 'ackermann') {
      const e = wrap(nav.goalTheta - pose.theta);
      if (Math.abs(e) > 0.04) return { v: 0, vy: 0, w: clamp(2.2 * e, -spec.maxAngular * 0.7, spec.maxAngular * 0.7) || 0, done: false };
    }
    return { v: 0, vy: 0, w: 0, done: true };
  }

  const vmax = spec.maxSpeed * speedScale;
  const Ld = clamp(0.45 + 0.5 * vmax, 0.45, 1.3);
  // Ileri bakis noktasi
  let ti = nav.idx;
  while (ti < last && d(ti) < Ld) ti++;
  const tx = path[ti].x, ty = path[ti].y;
  const dist = Math.max(0.05, Math.hypot(tx - pose.x, ty - pose.y));
  const alpha = wrap(Math.atan2(ty - pose.y, tx - pose.x) - pose.theta);
  const vlim = Math.sqrt(2 * spec.maxAccel * 0.5 * Math.max(0, remaining)) + 0.04;
  const sp = Math.min(vmax, vlim);

  if (kin === 'mecanum') {
    let wantTh = nav.goalTheta != null && remaining < 1.5 ? nav.goalTheta : pose.theta + alpha;
    const w = clamp(1.6 * wrap(wantTh - pose.theta), -spec.maxAngular * 0.6, spec.maxAngular * 0.6);
    return { v: sp * Math.cos(alpha), vy: sp * Math.sin(alpha), w, done: false };
  }

  if (kin === 'ackermann') {
    const L = spec.wheelBase || 1;
    const wMaxPerV = Math.tan((spec.maxSteer || 30) * Math.PI / 180) / L;
    if (Math.abs(alpha) > 2.0 && remaining < 3) { // hedef arkada ve yakin: geri manevra
      const v = -Math.min(sp, 0.4 * spec.maxSpeed);
      return { v, vy: 0, w: clamp(-alpha, -1, 1) * Math.abs(v) * wMaxPerV * -1, done: false };
    }
    const v = sp * clamp(1.15 - Math.abs(alpha) * 0.6, 0.35, 1);
    const k = clamp(2 * Math.sin(alpha) / dist, -wMaxPerV, wMaxPerV);
    return { v, vy: 0, w: v * k, done: false };
  }

  // Diferansiyel surus
  const a = Math.abs(alpha);
  if (a > 1.0 || (nav.rotating && a > 0.2)) {
    nav.rotating = true;
    return { v: 0, vy: 0, w: clamp(2.5 * alpha, -spec.maxAngular * 0.8, spec.maxAngular * 0.8), done: false };
  }
  nav.rotating = false;
  const v = sp * Math.max(0.25, Math.cos(alpha));
  const w = clamp(2 * v * Math.sin(alpha) / dist, -spec.maxAngular, spec.maxAngular);
  return { v, vy: 0, w, done: false };
}

function cumulative(path) {
  const cum = [0];
  for (let i = 1; i < path.length; i++) cum.push(cum[i - 1] + Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y));
  return cum;
}

module.exports = { follow, cumulative };
