'use strict';
const { TERRAIN } = require('../sim/world');
// Izgara tabanli A* yol planlayici (engel sisirme + yol yumusatma)

const RES = 0.1; // hucre boyutu (m)
const cache = new Map();

function diskOffsets(rCells) {
  const out = [];
  const r = Math.ceil(rCells);
  for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
    if (dx * dx + dy * dy <= rCells * rCells) out.push([dx, dy]);
  }
  return out;
}

function stamp(grid, W, H, cx, cy, offs, val) {
  for (let i = 0; i < offs.length; i++) {
    const x = cx + offs[i][0], y = cy + offs[i][1];
    if (x >= 0 && y >= 0 && x < W && y < H) { const k = y * W + x; if (grid[k] < val) grid[k] = val; }
  }
}

function buildGrid(world, radius) {
  const key = world.version + ':' + radius.toFixed(2);
  if (cache.has(key)) return cache.get(key);
  if (cache.size > 12) cache.clear();
  const W = Math.ceil(world.width / RES), H = Math.ceil(world.height / RES);
  const grid = new Uint8Array(W * H);
  const lethal = diskOffsets(radius / RES), near = diskOffsets((radius + 0.35) / RES);
  const cells = new Set();
  for (const s of world.staticSegments()) {
    const len = Math.hypot(s.x2 - s.x1, s.y2 - s.y1);
    const n = Math.max(1, Math.ceil(len / (RES * 0.5)));
    for (let i = 0; i <= n; i++) {
      const x = Math.floor((s.x1 + (s.x2 - s.x1) * i / n) / RES), y = Math.floor((s.y1 + (s.y2 - s.y1) * i / n) / RES);
      cells.add(Math.min(H - 1, Math.max(0, y)) * W + Math.min(W - 1, Math.max(0, x)));
    }
  }
  for (const o of world.obstacles) {
    if (o.move) continue;
    for (let y = Math.floor(o.y / RES); y <= Math.floor((o.y + o.h) / RES); y++)
      for (let x = Math.floor(o.x / RES); x <= Math.floor((o.x + o.w) / RES); x++)
        if (x >= 0 && y >= 0 && x < W && y < H) cells.add(y * W + x);
  }
  for (const k of cells) { const cx = k % W, cy = (k - cx) / W; stamp(grid, W, H, cx, cy, near, 1); }
  for (const k of cells) { const cx = k % W, cy = (k - cx) / W; stamp(grid, W, H, cx, cy, lethal, 2); }
  // Zemin maliyeti (x10): camur / kum gibi bolgelerden kacinmak icin
  const zc = new Uint8Array(W * H).fill(10);
  for (const z of world.zones) {
    const t = TERRAIN[z.type]; if (!t) continue;
    for (let y = Math.max(0, Math.floor(z.y / RES)); y < Math.min(H, Math.ceil((z.y + z.h) / RES)); y++)
      for (let x = Math.max(0, Math.floor(z.x / RES)); x < Math.min(W, Math.ceil((z.x + z.w) / RES)); x++) zc[y * W + x] = Math.round(t.cost * 10);
  }
  const g = { grid, W, H, lethal, zc };
  cache.set(key, g);
  return g;
}

class Heap {
  constructor() { this.k = []; this.v = []; }
  get size() { return this.k.length; }
  push(key, val) {
    const k = this.k, v = this.v; let i = k.length; k.push(key); v.push(val);
    while (i > 0) { const p = (i - 1) >> 1; if (k[p] <= key) break; k[i] = k[p]; v[i] = v[p]; i = p; }
    k[i] = key; v[i] = val;
  }
  pop() {
    const k = this.k, v = this.v; const top = v[0]; const lk = k.pop(), lv = v.pop(); const n = k.length;
    if (n > 0) {
      let i = 0;
      for (;;) {
        let c = 2 * i + 1; if (c >= n) break;
        if (c + 1 < n && k[c + 1] < k[c]) c++;
        if (k[c] >= lk) break;
        k[i] = k[c]; v[i] = v[c]; i = c;
      }
      k[i] = lk; v[i] = lv;
    }
    return top;
  }
}

function nearestFree(grid, W, H, cx, cy) {
  cx = Math.min(W - 1, Math.max(0, cx)); cy = Math.min(H - 1, Math.max(0, cy));
  if (grid[cy * W + cx] < 2) return [cx, cy];
  for (let r = 1; r < 40; r++) {
    let best = null, bd = Infinity;
    for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
      if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;
      const x = cx + dx, y = cy + dy;
      if (x < 0 || y < 0 || x >= W || y >= H || grid[y * W + x] >= 2) continue;
      const d = dx * dx + dy * dy; if (d < bd) { bd = d; best = [x, y]; }
    }
    if (best) return best;
  }
  return null;
}

function los(grid, W, x0, y0, x1, y1, strict, zc) {
  const zmax = zc ? Math.max(zc[y0 * W + x0], zc[y1 * W + x1]) : 0;
  const n = Math.max(Math.abs(x1 - x0), Math.abs(y1 - y0)) * 2;
  for (let i = 0; i <= n; i++) {
    const x = Math.round(x0 + (x1 - x0) * i / (n || 1)), y = Math.round(y0 + (y1 - y0) * i / (n || 1));
    const c = grid[y * W + x];
    if (c >= 2 || (strict && c >= 1)) return false;
    if (zc && zc[y * W + x] > zmax) return false;
  }
  return true;
}

// start/goal: {x,y} metre. extraPoints: gecici engel noktalari (or. LiDAR vuruslari)
function plan(world, start, goal, radius, extraPoints) {
  const base = buildGrid(world, radius);
  const W = base.W, H = base.H, zc = base.zc;
  let grid = base.grid;
  if (extraPoints && extraPoints.length) {
    grid = new Uint8Array(base.grid);
    for (const p of extraPoints) stamp(grid, W, H, Math.floor(p.x / RES), Math.floor(p.y / RES), base.lethal, 2);
  }
  const s = nearestFree(grid, W, H, Math.floor(start.x / RES), Math.floor(start.y / RES));
  const g = nearestFree(grid, W, H, Math.floor(goal.x / RES), Math.floor(goal.y / RES));
  if (!s || !g) return null;
  const N = W * H, sk = s[1] * W + s[0], gk = g[1] * W + g[0];
  const gs = new Float32Array(N).fill(Infinity), from = new Int32Array(N).fill(-1), closed = new Uint8Array(N);
  const heap = new Heap();
  const h = k => { const x = k % W, y = (k - x) / W; const dx = Math.abs(x - g[0]), dy = Math.abs(y - g[1]); return (dx + dy) + (Math.SQRT2 - 2) * Math.min(dx, dy); };
  gs[sk] = 0; heap.push(h(sk), sk);
  const NB = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];
  let found = false;
  while (heap.size) {
    const k = heap.pop();
    if (closed[k]) continue;
    closed[k] = 1;
    if (k === gk) { found = true; break; }
    const x = k % W, y = (k - x) / W;
    for (let i = 0; i < 8; i++) {
      const nx = x + NB[i][0], ny = y + NB[i][1];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const nk = ny * W + nx, c = grid[nk];
      if (c >= 2 || closed[nk]) continue;
      if (i >= 4 && (grid[y * W + nx] >= 2 || grid[ny * W + x] >= 2)) continue; // kose kesme yok
      const ng = gs[k] + NB[i][2] * (c === 1 ? 3 : 1) * zc[nk] / 10;
      if (ng < gs[nk]) { gs[nk] = ng; from[nk] = k; heap.push(ng + h(nk), nk); }
    }
  }
  if (!found) return null;
  const cells = [];
  for (let k = gk; k !== -1; k = from[k]) cells.push([k % W, Math.floor(k / W)]);
  cells.reverse();
  // Gorus hatti ile kisaltma
  const sm = [cells[0]];
  let i = 0;
  while (i < cells.length - 1) {
    let j = Math.min(cells.length - 1, i + 60);
    for (; j > i + 1; j--) {
      const a = cells[i], b = cells[j];
      const strict = grid[a[1] * W + a[0]] === 0 && grid[b[1] * W + b[0]] === 0;
      if (los(grid, W, a[0], a[1], b[0], b[1], strict, zc)) break;
    }
    sm.push(cells[j]); i = j;
  }
  let pts = sm.map(c => ({ x: (c[0] + 0.5) * RES, y: (c[1] + 0.5) * RES }));
  // Uc noktalar gercek konumlara oturtulur (hucre serbestse)
  if (grid[sk] < 2 && Math.hypot(pts[0].x - start.x, pts[0].y - start.y) < RES * 1.5) pts[0] = { x: start.x, y: start.y };
  const gFree = Math.floor(goal.x / RES) === g[0] && Math.floor(goal.y / RES) === g[1];
  if (gFree) pts[pts.length - 1] = { x: goal.x, y: goal.y };
  // 0.15 m aralikla siklastir
  const dense = [pts[0]];
  for (let k = 1; k < pts.length; k++) {
    const a = pts[k - 1], b = pts[k], d = Math.hypot(b.x - a.x, b.y - a.y), n = Math.max(1, Math.ceil(d / 0.15));
    for (let m = 1; m <= n; m++) dense.push({ x: a.x + (b.x - a.x) * m / n, y: a.y + (b.y - a.y) * m / n });
  }
  return { points: dense, goalAdjusted: !gFree };
}

module.exports = { plan, RES };
