'use strict';
// LiDAR ile doluluk izgarasi (occupancy grid) haritalama.
// Her tarama, aracin o anki konum kestirimine gore izgaraya islenir: isin boyunca hucreler "bos",
// isin ucundaki hucre "dolu" yonunde guncellenir (log-odds).
class LidarMapper {
  constructor(width, height) { this.reset(width, height); }

  reset(width, height) {
    this.width = width; this.height = height;
    this.res = width * height <= 1500 ? 0.05 : width * height <= 12000 ? 0.1 : 0.2;
    this.W = Math.ceil(width / this.res); this.H = Math.ceil(height / this.res);
    this.grid = new Int8Array(this.W * this.H);   // 0 = bilinmiyor, + dolu, - bos
    this.dirty = true; this.scans = 0;
  }

  clear() { this.grid.fill(0); this.dirty = true; this.scans = 0; }

  // pose: {x, y, theta} (harita cercevesi), L: lidar verisi ({ranges, angleMin, angleInc, range, rangeMin, x, y, theta})
  integrate(pose, L) {
    const { res, W, H, grid } = this;
    const c = Math.cos(pose.theta), s = Math.sin(pose.theta);
    const ox = pose.x + c * L.x - s * L.y, oy = pose.y + s * L.x + c * L.y;
    const maxR = L.range * 0.95, step = res * 0.9;
    for (let i = 0; i < L.ranges.length; i++) {
      let r = L.ranges[i];
      const hit = r > L.rangeMin && r < maxR;
      if (!hit) { if (r !== 0) continue; r = maxR; }             // 0 = donus yok: menzile kadar bos
      const a = pose.theta + L.theta + L.angleMin + i * L.angleInc, dx = Math.cos(a), dy = Math.sin(a);
      const end = hit ? r - res * 0.5 : r;
      let last = -1;
      for (let d = 0; d < end; d += step) {
        const gx = Math.floor((ox + dx * d) / res), gy = Math.floor((oy + dy * d) / res);
        if (gx < 0 || gy < 0 || gx >= W || gy >= H) break;
        const k = gy * W + gx; if (k === last) continue; last = k;
        if (grid[k] > -60) grid[k] -= 2;
      }
      if (hit) {
        let gx = Math.floor((ox + dx * r) / res), gy = Math.floor((oy + dy * r) / res);
        if (gx === W) gx = W - 1; if (gy === H) gy = H - 1; if (gx === -1) gx = 0; if (gy === -1) gy = 0;   // dis sinir duvarlari
        if (gx >= 0 && gy >= 0 && gx < W && gy < H) { const k = gy * W + gx; grid[k] = Math.min(60, grid[k] + 9); }
      }
    }
    this.scans++; this.dirty = true;
  }

  // Istemciye gonderim: 0 bilinmiyor / 1 bos / 2 dolu, run-length kodlu [deger, adet, deger, adet, ...]
  encode() {
    const g = this.grid, rle = [];
    let known = 0, occ = 0, cur = -1, n = 0;
    for (let i = 0; i < g.length; i++) {
      const v = g[i] > 10 ? 2 : g[i] < -5 ? 1 : 0;
      if (v) known++; if (v === 2) occ++;
      if (v === cur) n++; else { if (n) rle.push(cur, n); cur = v; n = 1; }
    }
    if (n) rle.push(cur, n);
    this.dirty = false;
    return { op: 'grid', w: this.W, h: this.H, res: this.res, rle, known: +(known / g.length * 100).toFixed(1), occ, scans: this.scans };
  }
}

module.exports = { LidarMapper };
