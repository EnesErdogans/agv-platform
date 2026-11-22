// Kod sekmesindeki hazir ornekler. Betik API'si: agv / ugv / robot ayni nesnedir.
window.EXAMPLES = [
  {
    name: 'Engelden kaçan gezgin (LiDAR)',
    code: `// LiDAR ile serbest gezinme: önü kapanınca açık tarafa dön
function loop(agv, dt) {
  const on = agv.lidarMin(-25, 25);
  const sol = agv.lidarMin(25, 90), sag = agv.lidarMin(-90, -25);
  if (on < 0.9) return agv.setVelocity(0, sol > sag ? 1.2 : -1.2);
  const v = Math.min(agv.spec.maxSpeed * 0.7, 0.3 + on * 0.25);
  agv.setVelocity(v, (sol - sag) * 0.15);   // koridoru ortala
}
`,
  },
  {
    name: 'Duvar takibi (sağ duvar, PD)',
    code: `// Sağ duvarı 0.8 m mesafeden takip et
const HEDEF = 0.8;
let onceki = 0;
function loop(agv, dt) {
  const on = agv.lidarMin(-20, 20);
  const sag = Math.min(agv.lidarMin(-100, -45), 3);
  if (on < 0.9) return agv.setVelocity(0.05, 1.3);        // köşe: sola dön
  const e = HEDEF - sag;                                   // +: duvara fazla yakın
  const w = 1.6 * e + 0.6 * (e - onceki) / dt;
  onceki = e;
  agv.setVelocity(0.5, Math.max(-1, Math.min(1, w)));
}
`,
  },
  {
    name: 'Çizgi izleyen (şerit sensörü, PD)',
    code: `// Çizgi izleyen: şerit sensörü ofsetine göre PD kontrol
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
`,
  },
  {
    name: 'İstasyon turu (dahili navigasyon + yük)',
    code: `// Dahili yol planlayıcıyı kullanarak taşıma döngüsü
const tur = ['A', 'C', 'B'];
let i = 0, bekle = 0;
function setup(agv) { agv.log('İstasyonlar:', agv.stations.map(s => s.id).join(', ')); }
function loop(agv, dt) {
  if (agv.navigating) return;                    // yolda
  if (bekle > 0) { bekle -= dt; return; }
  if (tur[i] === 'A') agv.setPayload(agv.spec.maxPayload); else if (tur[i] === 'B') agv.setPayload(0);
  i = (i + 1) % tur.length;
  agv.log('Hedef →', tur[i], '| yük', agv.payload, 'kg | batarya %', agv.battery);
  agv.goTo(tur[i]);
  bekle = 2;
}
`,
  },
  {
    name: 'Kare çiz (odometri / enkoder testi)',
    code: `// Yalnızca odometriye bakarak 3 m'lik kare çiz. Haritadaki kesikli "odom hayaleti" ile
// gerçek konum arasındaki fark, enkoder + kayma hatasını gösterir.
const KENAR = 3;
let adim = 0, x0 = null, th0 = 0;
function loop(agv, dt) {
  const o = agv.odom;
  if (x0 === null) { x0 = { x: o.x, y: o.y }; th0 = o.theta; }
  if (adim % 2 === 0) {                           // düz git
    if (Math.hypot(o.x - x0.x, o.y - x0.y) >= KENAR) { adim++; th0 = o.theta; return agv.setVelocity(0, 0); }
    agv.setVelocity(0.5, 0);
  } else {                                        // 90° dön
    let d = o.theta - th0; d = Math.atan2(Math.sin(d), Math.cos(d));
    if (Math.abs(d) >= Math.PI / 2 - 0.03) { adim++; x0 = { x: o.x, y: o.y }; if (adim % 8 === 0) agv.log('Tur bitti. Gerçek konum:', agv.pose.x, agv.pose.y); return agv.setVelocity(0, 0); }
    agv.setVelocity(0, 0.8);
  }
}
`,
  },
  {
    name: 'UGV: GPS + pusula ile devriye',
    code: `// GPS + pusula ile devriye (UGV): goTo kullanmadan kendi kontrolcün
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
`,
  },
  {
    name: 'UGV: kamera ile hedef takibi',
    code: `// Kameranın gördüğü ilk aracı takip et (AI kamera tespitlerinin simülasyonu)
function loop(ugv, dt) {
  const hedef = ugv.camera.find(d => d.kind === 'vehicle');
  if (!hedef) return ugv.setVelocity(0, 0.6);              // ara: yerinde dön
  const v = hedef.distance > 1.2 ? 0.4 : 0;                // 1.2 m mesafede dur
  ugv.setVelocity(v, 2.0 * hedef.bearing);
}
`,
  },
];
