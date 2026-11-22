'use strict';
// Otomatik testler:  npm test
const os = require('os');
const fs = require('fs');
const path = require('path');
const dgram = require('dgram');
const WebSocket = require('ws');
const { Fleet } = require('../server/fleet');
const { startServer } = require('../server/index');
const { World } = require('../server/sim/world');

let failed = 0;
const ok = (cond, name, extra) => { console.log((cond ? '  ✓ ' : '  ✗ ') + name + (extra ? '  — ' + extra : '')); if (!cond) failed++; };
const tmp = () => fs.mkdtempSync(path.join(os.tmpdir(), 'agv-test-'));
const sleep = ms => new Promise(r => setTimeout(r, ms));

function run(fleet, seconds, until) {
  const n = Math.round(seconds / 0.02);
  for (let i = 0; i < n; i++) { fleet.simStep(0.02); if (until && until()) return (i + 1) * 0.02; }
  return null;
}

async function main() {
  console.log('Sensör modeli');
  {
    const w = new World({ width: 10, height: 10, walls: [{ x1: 6, y1: 0, x2: 6, y2: 10 }] });
    const d = w.raycast(2, 5, 0, 12, null);
    ok(Math.abs(d - 4) < 1e-9, 'LiDAR ışını duvara 4.000 m', d.toFixed(3));
    ok(w.raycast(2, 5, Math.PI, 1.5, null) === Infinity, 'Menzil dışı → dönüş yok');
  }

  console.log('Otonom sürüş (her kinematik tip, A → hedef)');
  {
    const f = new Fleet(tmp());
    const goals = { agv200: 'B', cekici01: 'C', omni01: 'A', cizgi01: 'D', rover01: 'DV3', kesif01: 'DV1' };
    for (const v of f.vehicles) { v.setMode('auto'); ok(v.goTo(f.world.findStation(goals[v.id])), v.spec.name + ' için yol planlandı', v.nav ? v.nav.path.length + ' nokta' : ''); }
    let collisions = 0;
    const seen = new Set();
    const t = run(f, 240, () => { for (const v of f.vehicles) if (v.driver.collided && !seen.has(v.id)) { seen.add(v.id); collisions++; } return f.vehicles.every(v => !v.nav); });
    for (const v of f.vehicles) {
      const st = f.world.findStation(goals[v.id]), err = Math.hypot(v.pose.x - st.x, v.pose.y - st.y);
      ok(!v.nav && err < 0.3, `${v.spec.name} hedefe vardı`, `hata ${err.toFixed(2)} m, yol ${v.driver.distance.toFixed(1)} m`);
    }
    ok(collisions === 0, 'Çarpışma yok', t ? `süre ${t.toFixed(0)} sn (sim)` : 'zaman aşımı');
  }

  console.log('Görev döngüsü + 200 kg yük');
  {
    const f = new Fleet(tmp());
    const v = f.get('agv200');
    v.startMission({ steps: [{ type: 'load', kg: 200 }, { type: 'goto', station: 'C' }, { type: 'wait', sec: 2 }, { type: 'unload' }, { type: 'goto', station: 'A' }], loop: false });
    let maxPayload = 0;
    const t = run(f, 300, () => { maxPayload = Math.max(maxPayload, v.driver.payload); return !v.mission; });
    ok(t !== null && maxPayload === 200 && v.driver.payload === 0, 'Yükle → C → bekle → boşalt → A tamamlandı', t ? t.toFixed(0) + ' sn' : 'zaman aşımı');
    ok(v.driver.battery.soc < 1 && v.driver.battery.soc > 0.9, 'Batarya tüketimi makul', (v.driver.battery.soc * 100).toFixed(2) + ' %');
    ok(!v.driver.collided, 'Çarpışma yok');
  }

  console.log('Güvenlik katmanı');
  {
    const f = new Fleet(tmp());
    const v = f.get('agv200');
    v.driver.setPose({ x: 15, y: 3, theta: 0 }); // 5 m ileride bolme duvari (x=20)
    v.driver.payload = 200;
    let hit = false;
    run(f, 12, () => { v.setTeleop(1.2, 0, 0); if (v.driver.collided) hit = true; });
    const gap = 20 - (v.pose.x + v.spec.length / 2);
    ok(!hit && gap > 0.1 && gap < 0.6, 'Tam gaz + 200 kg yükle duvara sürüldü: durdu', `duvara ${gap.toFixed(2)} m`);
    v.spec.safety.enabled = false; v.driver.setPose({ x: 15, y: 3, theta: 0 });
    run(f, 12, () => { v.setTeleop(1.2, 0, 0); if (v.driver.collided) hit = true; });
    ok(hit, 'Güvenlik kapalıyken çarpışma algılanıyor (tampon)');
  }

  console.log('Kod testi (çizgi izleyen betik)');
  {
    const f = new Fleet(tmp());
    const v = f.get('cizgi01');
    ok(v.startScript(), 'Betik derlendi ve başladı');
    let lost = 0, samples = 0;
    run(f, 150, () => { const l = v.driver.sensors.line; if (l) { samples++; if (l.offset === null) lost++; } });
    ok(v.script.running, 'Betik hâlâ çalışıyor');
    ok(v.driver.distance > 40, 'Şerit üzerinde ilerledi', v.driver.distance.toFixed(1) + ' m');
    ok(lost / samples < 0.1, 'Çizgi takibi kararlı', (100 * lost / samples).toFixed(1) + ' % kayıp');
    ok(!v.driver.collided, 'Çarpışma yok');
    const v2 = f.get('agv200');
    ok(!v2.startScript('function loop(agv){ while(true){} }') || (run(f, 0.2), !v2.script.running), 'Sonsuz döngülü betik zaman aşımıyla durduruldu');
  }

  console.log('UGV: arazi, palet kayması, GPS, enkodersiz odometri');
  {
    const f = new Fleet(tmp());
    const r = f.get('rover01'), a = f.get('agv200');
    // Ayni camur seridinde UGV ve AGV: arazi kabiliyeti farki
    r.driver.setPose({ x: 39, y: 8.5, theta: 0 }); a.driver.setPose({ x: 39, y: 7.3, theta: 0 });
    a.spec.safety.enabled = false; r.spec.safety.enabled = false;
    run(f, 2, () => { r.setTeleop(0.6, 0, 0); a.setTeleop(1.2, 0, 0); });
    const fr = r.driver.vel.v / r.spec.maxSpeed, fa = a.driver.vel.v / a.spec.maxSpeed;
    ok(r.driver.terrain === 'mud' && fr > 0.75 && fa < 0.45, 'Çamurda UGV hızını korur, AGV yavaşlar', `UGV %${(fr * 100).toFixed(0)}, AGV %${(fa * 100).toFixed(0)}`);
    // Palet kaymasi: komut edilen donusten az doner
    r.driver.setPose({ x: 33, y: 9, theta: 0 });
    run(f, 3, () => r.setTeleop(0, 1.0, 0));
    ok(r.driver.vel.w < 0.85 && r.driver.vel.w > 0.5, 'Palet dönüş kayması', `komut 1.00 → gerçek ${r.driver.vel.w.toFixed(2)} rad/s`);
    ok(!r.driver.sensors.encoders, 'Enkodersiz araç: enkoder verisi yok, odometri açık çevrim');
    ok(r.driver.sensors.gps && r.driver.sensors.gps.fix && Math.hypot(r.driver.sensors.gps.x - r.pose.x, r.driver.sensors.gps.y - r.pose.y) < 3, 'Dış sahada GPS fix', JSON.stringify(r.driver.sensors.gps));
    r.driver.setPose({ x: 25, y: 9, theta: 0 }); run(f, 0.5);
    ok(r.driver.sensors.gps.fix === false, 'Fabrika içinde GPS fix yok');
    // GPS devriye betigi
    r.driver.setPose({ x: 33, y: 9, theta: 0 }); r.spec.safety.enabled = true;
    ok(r.startScript(), 'GPS devriye betiği başladı');
    const visited = new Set();
    f.listeners.add(m => { if (m.op === 'scriptLog' && m.id === 'rover01' && m.msg.startsWith('Varıldı')) visited.add(m.msg.split('|')[0]); });
    const t = run(f, 400, () => visited.size >= 4);
    ok(t !== null, 'Devriye turu tamamlandı (4 nokta)', t ? `${t.toFixed(0)} sn, ${r.driver.distance.toFixed(0)} m, batarya %${(r.driver.battery.soc * 100).toFixed(1)}` : [...visited].join(','));
    ok(!r.driver.collided, 'Çarpışma yok');
    const oe = Math.hypot(r.driver.odom.x - r.pose.x, r.driver.odom.y - r.pose.y);
    ok(oe > 0.5, 'Açık çevrim odometri gerçek konumdan sapıyor (beklenen)', oe.toFixed(1) + ' m');
  }

  console.log('Gerçek araç sürücüleri (sahte uçlarla)');
  {
    // Sahte rosbridge sunucusu
    const wss = new WebSocket.Server({ port: 0 });
    const port = wss.address().port;
    const got = { advertise: [], subscribe: [], cmd: null, scan: null, odom: null };
    wss.on('connection', ws => ws.on('message', raw => {
      const m = JSON.parse(raw);
      if (m.op === 'advertise') got.advertise.push(m.topic);
      if (m.op === 'subscribe') {
        got.subscribe.push(m.topic);
        if (m.topic === '/odom') ws.send(JSON.stringify({ op: 'publish', topic: '/odom', msg: { pose: { pose: { position: { x: 1, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }, twist: { twist: { linear: { x: 0.3, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } } } } }));
        if (m.topic === '/scan') ws.send(JSON.stringify({ op: 'publish', topic: '/scan', msg: { angle_min: -3.14, angle_increment: 0.0175, range_min: 0.15, range_max: 12, ranges: new Array(360).fill(2.5) } }));
        if (m.topic === '/sim/cmd_vel') setInterval(() => ws.readyState === 1 && ws.send(JSON.stringify({ op: 'publish', topic: '/sim/cmd_vel', msg: { linear: { x: 0.4, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } } })), 100).unref();
      }
      if (m.op === 'publish' && m.topic === '/cmd_vel') got.cmd = m.msg;
      if (m.op === 'publish' && m.topic === '/sim/scan') got.scan = m.msg;
      if (m.op === 'publish' && m.topic === '/sim/odom') got.odom = m.msg;
    }));
    const f = new Fleet(tmp());
    const real = f.addVehicle({ ...f.get('agv200').spec, id: 'gercek', name: 'Gerçek', start: { x: 10, y: 3, theta: Math.PI / 2 }, driver: { type: 'rosbridge', url: 'ws://127.0.0.1:' + port } });
    real.driver.connect();
    f.updateVehicle('agv200', { ...f.get('agv200').spec, rosExpose: { enabled: true, url: 'ws://127.0.0.1:' + port, namespace: '/sim' } });
    f.start();
    await sleep(500);
    for (let i = 0; i < 6; i++) { real.setTeleop(0.5, 0.1, 0); await sleep(100); }
    ok(real.driver.connected, 'rosbridge bağlantısı kuruldu');
    ok(got.advertise.includes('/cmd_vel') && got.subscribe.includes('/odom') && got.subscribe.includes('/scan'), 'cmd_vel yayını, odom/scan aboneliği');
    ok(Math.abs(real.pose.x - 10) < 1e-6 && Math.abs(real.pose.y - 4) < 1e-6, 'Odometri harita çerçevesine oturtuldu', `(${real.pose.x.toFixed(2)}, ${real.pose.y.toFixed(2)})`);
    ok(got.cmd && got.cmd.linear.x > 0 && got.cmd.linear.x <= 0.5, 'Gerçek araca cmd_vel gönderildi', got.cmd ? 'v=' + got.cmd.linear.x.toFixed(2) : '');
    ok(real.driver.sensors.lidar && real.driver.sensors.lidar.ranges.length === 360, 'Gerçek LaserScan alındı');
    ok(got.scan && got.scan.ranges.length === 360 && got.odom, 'Simüle araç ROS\'a /sim/scan ve /sim/odom yayınlıyor');
    ok(f.get('agv200').driver.vel.v > 0.1, 'ROS /sim/cmd_vel simüle aracı sürdü', 'v=' + f.get('agv200').driver.vel.v.toFixed(2));

    // UDP (ESP32) geri dongu
    const esp = dgram.createSocket('udp4');
    let espCmd = null;
    await new Promise(r => esp.bind(0, '127.0.0.1', r));
    esp.on('message', b => { espCmd = JSON.parse(b.toString()); });
    const u = f.addVehicle({ ...f.get('agv200').spec, id: 'esp', name: 'ESP', driver: { type: 'udp', host: '127.0.0.1', port: esp.address().port, listenPort: 42555 } }); u.spec.sensors = [{ type: 'ultrasonic', x: 0.45, y: 0, theta: 0 }];
    u.driver.connect();
    await sleep(150);
    const tpr = u.spec.encoder.ticksPerRev, per = tpr / (2 * Math.PI * u.spec.wheelRadius);
    for (const m of [0, 0.5, 1.0]) { esp.send(JSON.stringify({ t: 'tel', enc: [Math.round(m * per), Math.round(m * per)], bat: 25.2, us: [0.8] }), 42555, '127.0.0.1'); await sleep(40); }
    u.setTeleop(0.3, 0, 0);
    await sleep(200);
    ok(u.driver.connected && Math.abs(u.driver.raw.x - 1) < 0.01, 'UDP telemetri: enkoderden odometri (1.00 m)', u.driver.raw.x.toFixed(3) + ' m');
    ok(espCmd && espCmd.t === 'cmd' && espCmd.v > 0, 'UDP komutu araca ulaştı', JSON.stringify(espCmd));
    // Enkodersiz arac (paletli rover): telemetri yalnizca "bagliyim" sinyali, konum komuttan tahmin edilir
    const rv = f.addVehicle({ ...f.get('rover01').spec, id: 'rv', name: 'RoverPi', script: '', start: { x: 40, y: 3, theta: 0 }, driver: { type: 'udp', host: '127.0.0.1', port: esp.address().port, listenPort: 42556 } });
    rv.spec.sensors = []; rv.driver.connect(); await sleep(100);
    for (let i = 0; i < 10; i++) { esp.send(JSON.stringify({ t: 'tel', enc: [0, 0], bat: 11.8 }), 42556, '127.0.0.1'); rv.setTeleop(0.5, 0, 0); await sleep(100); }
    ok(rv.driver.connected && rv.driver.deadReckoning && rv.pose.x > 40.3 && rv.pose.x < 40.7, 'Enkodersiz gerçek araç: konum komuttan tahmin ediliyor', `x = ${rv.pose.x.toFixed(2)} m (1 sn × 0.5 m/s)`);
    f.stop(); esp.close(); wss.close();
  }

  console.log('Çoklu harita + araç gizleme / silme');
  {
    const dir = tmp(), f = new Fleet(dir);
    f.newMap('Boş depo', { width: 20, height: 12, zones: [{ type: 'roof', x: 0, y: 0, w: 20, h: 12 }] });
    const second = f.activeMap;
    ok(f.maps.length === 2 && f.world.width === 20 && f.world.walls.length === 0 && f.world.stations.length === 0, 'Sıfırdan boş harita oluşturuldu ve açıldı', f.curMap().name);
    const ps = f.vehicles.map(v => v.pose);
    const inside = ps.every(p => p.x > 0.2 && p.y > 0.2 && p.x < 19.8 && p.y < 11.8);
    let minD = Infinity; for (let i = 0; i < ps.length; i++) for (let j = i + 1; j < ps.length; j++) minD = Math.min(minD, Math.hypot(ps[i].x - ps[j].x, ps[i].y - ps[j].y));
    ok(inside && minD > 0.4, 'Araçlar yeni haritaya çakışmadan yerleştirildi', 'en yakın çift ' + minD.toFixed(2) + ' m');
    const c = f.get('cekici01'), a = f.get('agv200');
    f.setHidden(c, true);
    const before = { ...c.pose };
    run(f, 2, () => { c.setTeleop(1, 0, 0); });
    ok(c.hidden && c.pose.x === before.x && f.othersOf(a).length === 4 * (f.vehicles.length - 2), 'Gizli araç simüle edilmiyor ve engel sayılmıyor');
    f.setWorld({ ...f.world.toJSON(), walls: [{ x1: 5, y1: 6, x2: 10, y2: 6 }] });
    f.selectMap('m1');
    ok(f.world.width === 48 && f.world.stations.length === 10 && !f.get('cekici01').hidden && Math.abs(a.pose.x - 2.5) < 1e-6, 'İlk haritaya dönüldü: saha, konumlar ve gizlilik haritaya özel');
    f.selectMap(second);
    ok(f.world.walls.length === 1 && f.get('cekici01').hidden, 'İkinci harita düzenlemeleriyle geri geldi');
    f.save();
    const f2 = new Fleet(dir);
    ok(f2.maps.length === 2 && f2.activeMap === second && f2.world.width === 20 && f2.get('cekici01').hidden, 'Haritalar diske kaydedilip geri yüklendi');
    f.removeVehicle('omni01');
    ok(!f.get('omni01') && f.vehicles.length === 5, 'Araç silindi');
    f.deleteMap(second);
    ok(f.maps.length === 1 && f.world.width === 48, 'Harita silindi, kalan harita açıldı');
    f.deleteMap(f.activeMap);
    ok(f.maps.length === 1, 'Son harita silinemiyor');
    // eski surum proje dosyasi (tek "world") sorunsuz aciliyor
    const old = tmp(); fs.writeFileSync(path.join(old, 'fleet.json'), JSON.stringify({ world: { width: 15, height: 9 }, vehicles: [f.get('agv200').spec] }));
    const f3 = new Fleet(old);
    ok(f3.maps.length === 1 && f3.world.width === 15 && f3.vehicles.length === 1, 'v1.0 proje dosyası yeni sürümde açılıyor');
  }

  console.log('LiDAR haritalama (doluluk ızgarası)');
  {
    const f = new Fleet(tmp());
    f.newMap('Oda', { width: 12, height: 8, walls: [{ x1: 6, y1: 0, x2: 6, y2: 5 }], zones: [{ type: 'roof', x: 0, y: 0, w: 12, h: 8 }] });
    for (const v of f.vehicles) if (v.id !== 'agv200') f.setHidden(v, true);
    const a = f.get('agv200'); a.driver.setPose({ x: 2, y: 2, theta: 0 }); a.mapping = true;
    f.setWorld({ ...f.world.toJSON(), stations: [{ id: 'X', name: 'X', x: 10, y: 2, theta: null }] });
    a.setMode('auto'); a.goTo(f.world.findStation('X'));
    run(f, 90, () => !a.nav);
    const g = f.mapper, e = g.encode(), cell = (x, y) => g.grid[Math.floor(y / g.res) * g.W + Math.floor(x / g.res)];
    let wallHits = 0; for (let y = 0.5; y < 4.6; y += 0.25) if (cell(6.0 - 0.01, y) > 10 || cell(6.0 + 0.01, y) > 10) wallHits++;
    ok(!a.nav && e.known > 85, 'Araç gezerken oda haritalandı', `kapsama %${e.known}, ${e.scans} tarama, ${g.res * 100} cm/hücre`);
    ok(wallHits >= 14 && cell(3, 3) < -5 && cell(9, 3) < -5, 'İç duvar dolu, odaların içi boş işaretlendi', wallHits + '/17 duvar örneği');
    let sum = 0; for (let i = 1; i < e.rle.length; i += 2) sum += e.rle[i];
    ok(sum === g.W * g.H, 'Izgara istemci için doğru kodlandı (RLE)', e.rle.length / 2 + ' parça');
    f.newMap('Başka', { width: 20, height: 10 });
    ok(f.mapper.W === 400 && f.mapper.encode().known === 0, 'Harita değişince tarama sıfırlandı');
  }

  console.log('Sunucu + WebSocket API');
  {
    const s = await startServer({ dataDir: tmp(), port: 18765 });
    const ws = new WebSocket(`ws://127.0.0.1:${s.port}/ws`);
    const msgs = [];
    ws.on('message', m => msgs.push(JSON.parse(m)));
    await new Promise(r => ws.on('open', r));
    ws.send(JSON.stringify({ op: 'goto', id: 'agv200', station: 'D' }));
    ws.send(JSON.stringify({ op: 'sim', action: 'play', speed: 4 }));
    await sleep(1500);
    const hello = msgs.find(m => m.op === 'hello'), st = msgs.filter(m => m.op === 'state').pop(), pth = msgs.filter(m => m.op === 'path' && m.id === 'agv200' && m.path).pop();
    ok(hello && hello.vehicles.length === 6 && hello.world.stations.length === 10, 'hello: 6 araç, 10 istasyon');
    ok(pth && pth.path.length > 10, 'Yol istemciye iletildi');
    const a = st && st.vehicles.find(v => v.id === 'agv200');
    ok(a && a.mode === 'auto' && Math.abs(a.v) > 0.2, 'Durum akışı: araç otonom ilerliyor', a ? `v=${a.v} m/s` : '');
    const page = await new Promise(r => require('http').get(s.url, res => { let b = ''; res.on('data', d => b += d); res.on('end', () => r(b)); }));
    ok(page.includes('Fleet Studio'), 'Arayüz HTTP üzerinden sunuluyor');
    ws.send(JSON.stringify({ op: 'save' })); await sleep(200);
    ok(fs.existsSync(path.join(s.fleet.dataDir, 'fleet.json')), 'Proje dosyaya kaydedildi');
    ws.close(); s.close();
  }

  console.log(failed ? `\n${failed} test BAŞARISIZ` : '\nTüm testler geçti');
  process.exit(failed ? 1 : 0);
}

main().catch(e => { console.error(e); process.exit(1); });
