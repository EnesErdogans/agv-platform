'use strict';
// AGV Fleet Studio sunucusu: HTTP (arayuz) + WebSocket (canli durum / komut API'si)
const http = require('http');
const fs = require('fs');
const path = require('path');
const os = require('os');
const { WebSocketServer } = require('ws');
const { Fleet } = require('./fleet');
const { listPorts, serialAvailable } = require('./drivers/real');
const defaults = require('../config/defaults');

const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png', '.json': 'application/json' };
const WEB = path.join(__dirname, '..', 'web');

function startServer(opts = {}) {
  const dataDir = opts.dataDir || process.env.AGV_DATA_DIR || path.join(os.homedir(), '.agv-fleet-studio');
  const host = opts.lan || process.env.AGV_LAN === '1' ? '0.0.0.0' : '127.0.0.1';
  const fleet = new Fleet(dataDir);

  const server = http.createServer((req, res) => {
    let p = decodeURIComponent((req.url || '/').split('?')[0]);
    if (p === '/') p = '/index.html';
    const file = path.normalize(path.join(WEB, p));
    if (!file.startsWith(WEB)) { res.writeHead(403); return res.end(); }
    fs.readFile(file, (err, buf) => {
      if (err) { res.writeHead(404); return res.end('Not found'); }
      res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'application/octet-stream', 'Cache-Control': 'no-store' });
      res.end(buf);
    });
  });

  const wss = new WebSocketServer({ server, path: '/ws' });
  const clients = new Set();
  const sendAll = msg => { const s = JSON.stringify(msg); for (const c of clients) if (c.readyState === 1) c.send(s); };
  fleet.listeners.add(sendAll);

  const pathSeqs = new Map();
  const stateTimer = setInterval(() => {
    if (!clients.size) return;
    for (const p of fleet.paths()) if (pathSeqs.get(p.id) !== p.seq) { pathSeqs.set(p.id, p.seq); sendAll({ op: 'path', id: p.id, path: p.path }); }
    sendAll(fleet.snapshot());
  }, 66);
  const gridTimer = setInterval(() => { if (clients.size && fleet.mapper.dirty) sendAll(fleet.mapper.encode()); }, 700);

  const handlers = {
    teleop(m, v) { if (v && !v.hidden) v.setTeleop(m.v, m.w, m.vy, m.src); },
    mode(m, v) { if (v) v.setMode(m.mode); },
    goto(m, v) {
      if (!v || v.hidden) return;
      const t = m.station != null ? fleet.world.findStation(m.station) : { x: +m.x, y: +m.y, theta: m.theta == null ? null : +m.theta };
      if (!t || !isFinite(t.x) || !isFinite(t.y)) return fleet.log('error', 'Geçersiz hedef', v.id);
      if (v.mode === 'manual') v.setMode('auto');
      v.mission = null; v.goTo(t);
    },
    mission(m, v) { if (v && !v.hidden) v.startMission(m.mission || {}); },
    cancel(m, v) { if (v) { v.cancel(); v.status = 'İptal edildi'; } },
    estop(m, v) {
      const list = m.id === 'all' || !m.id ? fleet.vehicles : (v ? [v] : []);
      for (const x of list) { x.estop = !!m.on; if (m.on) x.cancel(); }
      fleet.log(m.on ? 'error' : 'ok', (m.on ? 'ACİL DURDURMA' : 'Acil durdurma kaldırıldı') + (list.length === 1 ? '' : ' (tüm filo)'), list.length === 1 ? list[0].id : null);
    },
    sim(m) {
      if (m.action === 'play') fleet.sim.running = true;
      else if (m.action === 'pause') fleet.sim.running = false;
      else if (m.action === 'reset') fleet.resetSim();
      if (m.speed) fleet.sim.speed = Math.min(8, Math.max(0.25, +m.speed));
    },
    addVehicle(m) { if (m.spec) fleet.addVehicle(m.spec); },
    updateVehicle(m, v) { if (v && m.spec) fleet.updateVehicle(v.id, m.spec); },
    removeVehicle(m, v) { if (v) fleet.removeVehicle(v.id); },
    setPose(m, v) {
      if (!v) return;
      const p = { x: +m.x, y: +m.y, theta: +m.theta || 0 };
      v.cancel(); v.driver.setPose(p);
      if (v.isSim) v.spec.start = p;
    },
    payload(m, v) { if (v) v.driver.payload = Math.min(v.spec.maxPayload, Math.max(0, +m.kg || 0)); },
    speedScale(m, v) { if (v) v.speedScale = Math.min(1, Math.max(0.1, +m.value || 1)); },
    world(m) { if (m.world) fleet.setWorld(m.world); },
    script(m, v) { if (!v) return; if (typeof m.code === 'string') v.spec.script = m.code; if (m.run === true && !v.hidden) v.startScript(); else if (m.run === false) v.stopScript(); },
    connect(m, v) { if (v && !v.isSim) v.driver.connect(); },
    disconnect(m, v) { if (v && !v.isSim) v.driver.disconnect(); },
    mapping(m, v) {
      if (!v) return;
      if (m.source === 'pose' || m.source === 'odom') v.mapSource = m.source;
      if (m.on != null) { v.mapping = !!m.on; fleet.log('info', v.mapping ? 'LiDAR haritalama başladı' : 'LiDAR haritalama durdu', v.id); }
    },
    mappingClear() { fleet.mapper.clear(); },
    hide(m, v) { if (v) fleet.setHidden(v, m.on); },
    removeAllVehicles() { for (const x of fleet.vehicles.slice()) fleet.removeVehicle(x.id); },
    mapSelect(m) { fleet.selectMap(m.map); },
    mapNew(m) { fleet.newMap(m.name, m.world, !!m.copy); },
    mapRename(m) { fleet.renameMap(m.map, m.name); },
    mapDelete(m) { fleet.deleteMap(m.map); },
    save() { fleet.save(); },
    factoryReset() { fleet.factoryReset(); },
    async listPorts(m, v, ws) { ws.send(JSON.stringify({ op: 'ports', ports: await listPorts(), available: serialAvailable })); },
  };

  wss.on('connection', ws => {
    clients.add(ws);
    ws.send(JSON.stringify({ op: 'hello', version: require('../package.json').version, world: fleet.world.toJSON(), maps: fleet.mapList(), activeMap: fleet.activeMap, vehicles: fleet.specs(), presets: defaults.presets, logs: fleet.logs.slice(-80), dataDir, serialAvailable }));
    for (const p of fleet.paths()) ws.send(JSON.stringify({ op: 'path', id: p.id, path: p.path }));
    ws.send(JSON.stringify(fleet.mapper.encode()));
    ws.on('message', raw => {
      let m; try { m = JSON.parse(raw); } catch (e) { return; }
      const h = handlers[m && m.op];
      if (!h) return;
      try { h(m, m.id ? fleet.get(m.id) : null, ws); } catch (e) { fleet.log('error', 'Komut hatası (' + m.op + '): ' + e.message); }
    });
    ws.on('close', () => clients.delete(ws));
    ws.on('error', () => clients.delete(ws));
  });

  fleet.start();

  return new Promise((resolve, reject) => {
    let port = +opts.port || +process.env.AGV_PORT || 8765, tries = 0;
    server.on('error', e => {
      if (e.code === 'EADDRINUSE' && tries++ < 20) server.listen(++port, host); else reject(e);
    });
    server.on('listening', () => resolve({
      port, host, fleet, url: `http://127.0.0.1:${port}`,
      close() { clearInterval(stateTimer); clearInterval(gridTimer); fleet.stop(); for (const c of clients) c.terminate(); wss.close(); server.close(); },
    }));
    server.listen(port, host);
  });
}

module.exports = { startServer };

if (require.main === module) {
  startServer({ lan: process.argv.includes('--lan') }).then(s => {
    console.log(`AGV Fleet Studio çalışıyor → ${s.url}  (${s.host === '0.0.0.0' ? 'ağa açık' : 'yalnızca bu bilgisayar'})`);
  }).catch(e => { console.error(e); process.exit(1); });
}
