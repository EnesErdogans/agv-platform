'use strict';
/* AGV-UGV Fleet Studio — arayuz */
const $ = s => document.querySelector(s);
const esc = s => String(s == null ? '' : s).replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const clone = o => JSON.parse(JSON.stringify(o));
const fmt = (n, d = 2) => (n == null || isNaN(n)) ? '—' : Number(n).toFixed(d);
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const DEG = Math.PI / 180;
const getPath = (o, p) => p.split('.').reduce((a, k) => (a == null ? undefined : a[k]), o);
const setPath = (o, p, v) => { const ks = p.split('.'); let a = o; for (let i = 0; i < ks.length - 1; i++) { if (a[ks[i]] == null || typeof a[ks[i]] !== 'object') a[ks[i]] = {}; a = a[ks[i]]; } a[ks[ks.length - 1]] = v; };

const S = {
  world: null, specs: [], st: new Map(), lidar: new Map(), paths: new Map(), trails: new Map(), dyn: [],
  sel: null, tab: 'control', tool: 'select', layers: { site: true, scan: true, lidar: true, path: true, trail: true, sensors: true }, grid: null,
  maps: [], activeMap: null, presets: [], sim: { running: true, speed: 1 }, t: 0, draft: null, dirty: false, code: {}, missions: {}, hist: [],
  consoles: {}, ports: [], portsLoaded: false, dataDir: '', speedPct: 70, joy: { x: 0, y: 0, on: false }, keys: new Set(), userView: false, tickN: 0,
};
const TOOL_HINT = {
  select: '', goto: 'Hedefe tıkla — sürükleyerek varış yönünü ver', pose: 'Aracın yeni konumuna tıkla — sürükleyerek yön ver',
  wall: 'Sürükleyerek duvar çiz', box: 'Sürükleyerek engel çiz — Shift ile bırak: hareketli engel', station: 'İstasyon konumuna tıkla — sürükleyerek yön ver',
  tape: 'Sürükleyerek şerit çiz (çizgi izleyenler için)', zone: 'Sürükleyerek zemin bölgesi çiz', erase: 'Silinecek harita öğesine tıkla',
};
const ZONE = {
  roof: { name: 'Kapalı alan (GPS yok)', fill: 'rgba(120,160,210,.06)' }, grass: { name: 'Çim', fill: 'rgba(70,160,90,.17)' },
  gravel: { name: 'Çakıl', fill: 'rgba(170,170,160,.15)' }, sand: { name: 'Kum', fill: 'rgba(220,185,110,.2)' }, mud: { name: 'Çamur', fill: 'rgba(135,88,50,.42)' },
};
const DRV = { sim: 'SİM', rosbridge: 'ROS 2', serial: 'SERİ', udp: 'UDP' };
const spec = id => S.specs.find(s => s.id === id);
const selSpec = () => spec(S.sel);
const selSt = () => S.st.get(S.sel);

// ------------------------------------------------------------------ WebSocket
let ws;
const send = o => { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); };
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { $('#conn').className = 'pill ok'; };
  ws.onclose = () => { $('#conn').className = 'pill off'; hint('Sunucu bağlantısı koptu — yeniden bağlanılıyor…', 1400); setTimeout(connect, 1500); };
  ws.onmessage = e => onMsg(JSON.parse(e.data));
}
function onMsg(m) {
  if (m.op === 'state') return onState(m);
  if (m.op === 'hello') {
    S.world = m.world; S.presets = m.presets; S.dataDir = m.dataDir; S.version = m.version; setMaps(m.maps, m.activeMap);
    $('#logBody').innerHTML = ''; (m.logs || []).forEach(addLog);
    setSpecs(m.vehicles); fit();
  } else if (m.op === 'specs') setSpecs(m.vehicles);
  else if (m.op === 'world') { S.world = m.world; if (S.tab === 'control') renderPanel(); }
  else if (m.op === 'maps') setMaps(m.maps, m.active);
  else if (m.op === 'grid') setGrid(m);
  else if (m.op === 'path') S.paths.set(m.id, m.path);
  else if (m.op === 'log') addLog(m);
  else if (m.op === 'scriptLog') { (S.consoles[m.id] = S.consoles[m.id] || []).push(m); if (S.consoles[m.id].length > 200) S.consoles[m.id].shift(); if (m.id === S.sel && S.tab === 'code') appendConsole(m); }
  else if (m.op === 'ports') { S.ports = m.ports; S.portsLoaded = true; if (S.tab === 'link') renderPanel(); }
}
// LiDAR doluluk haritasi: RLE coz, ekran disi tuvale ciz (0 bilinmiyor, 1 bos, 2 dolu)
function setGrid(m) {
  const st = new Uint8Array(m.w * m.h); let k = 0;
  for (let i = 0; i < m.rle.length; i += 2) { const v = m.rle[i], n = m.rle[i + 1]; if (v) st.fill(v, k, k + n); k += n; }
  let c = S.grid && S.grid.canvas; if (!c || c.width !== m.w || c.height !== m.h) { c = document.createElement('canvas'); c.width = m.w; c.height = m.h; }
  const g = c.getContext('2d'), img = g.createImageData(m.w, m.h), d = img.data;
  for (let y = 0; y < m.h; y++) for (let x = 0; x < m.w; x++) {
    const v = st[y * m.w + x]; if (!v) continue;
    const o = ((m.h - 1 - y) * m.w + x) * 4;
    if (v === 2) { d[o] = 235; d[o + 1] = 246; d[o + 2] = 255; d[o + 3] = 255; } else { d[o] = 110; d[o + 1] = 190; d[o + 2] = 255; d[o + 3] = 34; }
  }
  g.putImageData(img, 0, 0);
  S.grid = { w: m.w, h: m.h, res: m.res, known: m.known, scans: m.scans, st, canvas: c };
}
function gridPgm() {
  const G = S.grid, head = `P5\n# AGV-UGV Fleet Studio LiDAR haritasi\n${G.w} ${G.h}\n255\n`, out = new Uint8Array(head.length + G.w * G.h);
  for (let i = 0; i < head.length; i++) out[i] = head.charCodeAt(i);
  let k = head.length;
  for (let y = G.h - 1; y >= 0; y--) for (let x = 0; x < G.w; x++) { const v = G.st[y * G.w + x]; out[k++] = v === 2 ? 0 : v === 1 ? 254 : 205; }
  return out;
}
function setMaps(maps, active) {
  const changed = S.activeMap !== active;
  S.maps = maps || []; S.activeMap = active;
  $('#selMap').innerHTML = S.maps.map(m => `<option value="${esc(m.id)}" ${m.id === active ? 'selected' : ''}>${esc(m.name)}</option>`).join('');
  if (changed) { S.trails.clear(); S.lidar.clear(); S.paths.clear(); S.userView = false; setTimeout(fit, 60); }
}
function setSpecs(list) {
  S.specs = list;
  const ids = new Set(list.map(s => s.id));
  for (const k of [...S.st.keys()]) if (!ids.has(k)) { S.st.delete(k); S.lidar.delete(k); S.paths.delete(k); S.trails.delete(k); }
  if (!ids.has(S.sel)) { S.sel = list.length ? list[0].id : null; S.dirty = false; S.hist = []; }
  if (!S.dirty) S.draft = selSpec() ? clone(selSpec()) : null;
  renderCards(); renderPanel();
}
function onState(m) {
  if (m.t < S.t - 0.5) S.trails.clear();
  S.t = m.t; S.sim = m.sim; S.dyn = m.dyn; S.tickN++;
  for (const v of m.vehicles) {
    const prev = S.st.get(v.id);
    if (v.sens.lidar) S.lidar.set(v.id, { ...v.sens.lidar, px: v.x, py: v.y, pth: v.th });
    S.st.set(v.id, v);
    let tr = S.trails.get(v.id); if (!tr) S.trails.set(v.id, tr = []);
    const l = tr[tr.length - 1];
    if (!l || Math.hypot(l[0] - v.x, l[1] - v.y) > 0.12) { tr.push([v.x, v.y]); if (tr.length > 700) tr.shift(); }
    if (prev && Math.hypot(prev.x - v.x, prev.y - v.y) > 3) tr.length = 0;
    if (v.id === S.sel) { S.hist.push([v.v, v.cv, v.w]); if (S.hist.length > 220) S.hist.shift(); }
  }
  const mm = Math.floor(m.t / 60), ss = (m.t % 60).toFixed(1).padStart(4, '0');
  $('#clock').textContent = String(mm).padStart(2, '0') + ':' + ss;
  $('#btnPlay').textContent = S.sim.running ? '⏸' : '▶';
  if ($('#selSpeed').value != S.sim.speed && document.activeElement !== $('#selSpeed')) $('#selSpeed').value = S.sim.speed;
  const any = m.vehicles.some(v => v.estop);
  $('#btnEstop').classList.toggle('active', any); $('#btnEstop').textContent = any ? 'DURDURMAYI KALDIR' : 'ACİL DURDUR';
  updateCards(); updatePanel();
}
function addLog(m) {
  const b = $('#logBody'), d = new Date(m.t), who = m.id ? (spec(m.id) ? spec(m.id).name : m.id) : '';
  const el = document.createElement('div'); el.className = m.level;
  el.innerHTML = `<span class="ts">${d.toTimeString().slice(0, 8)}</span>${who ? `<span class="who">${esc(who)}</span>` : ''}${esc(m.msg)}`;
  b.appendChild(el); while (b.children.length > 250) b.firstChild.remove();
  b.scrollTop = b.scrollHeight;
}

// ------------------------------------------------------------------ Filo kartlari
function statusText(v) {
  if (v.hid) return 'Gizli — devre dışı';
  if (v.estop) return 'ACİL DURDURMA';
  if (v.type !== 'sim' && !v.conn) return 'Bağlı değil';
  if (v.coll) return 'Çarpışma / tampon';
  if (v.safe === 'stop') return 'Engel — durdu';
  return v.status + (v.chg ? ' · şarj oluyor' : '');
}
function renderCards() {
  $('#cards').innerHTML = S.specs.map(s => `
    <div class="card ${s.id === S.sel ? 'sel' : ''}" data-id="${esc(s.id)}" style="--c:${esc(s.color)}">
      <div class="t"><span class="dot" data-c="dot"></span><b>${esc(s.name)}</b><span class="badge ${s.class === 'UGV' ? 'ugv' : 'agv'}">${s.class}</span><span class="badge ${s.driver.type !== 'sim' ? 'real' : ''}">${DRV[s.driver.type]}</span></div>
      <div class="s" data-c="status">—</div>
      <div class="m"><span data-c="soc" class="mono">—</span><span class="bar"><i data-c="bar" style="width:0"></i></span><span data-c="v" class="mono">0.00 m/s</span></div>
      <div class="acts"><button data-cact="hide" title="Aracı bu haritada gizle / göster (gizli araç simüle edilmez)">Gizle</button><button data-cact="del" class="del" title="Aracı filodan sil">Sil</button></div>
    </div>`).join('') || '<div class="empty">Filo boş. “+ Araç” ile ekleyin.</div>';
  updateCards();
}
function updateCards() {
  for (const el of $('#cards').children) {
    const v = S.st.get(el.dataset.id); if (!v) continue;
    const q = c => el.querySelector(`[data-c="${c}"]`);
    const bad = !v.hid && (v.estop || v.coll || (v.type !== 'sim' && !v.conn));
    el.classList.toggle('hid', !!v.hid); el.querySelector('[data-cact="hide"]').textContent = v.hid ? 'Göster' : 'Gizle';
    q('dot').className = 'dot ' + (v.estop || v.coll ? 'bad' : (v.type !== 'sim' && !v.conn) ? 'off' : '');
    q('status').textContent = statusText(v); q('status').className = 's' + (bad ? ' warn' : '');
    q('soc').textContent = v.soc == null ? '—' : Math.round(v.soc * 100) + '%';
    q('bar').style.width = (v.soc == null ? 0 : v.soc * 100) + '%'; q('bar').style.background = v.soc != null && v.soc < 0.2 ? 'var(--danger)' : v.chg ? 'var(--blue)' : 'var(--ok)';
    q('v').textContent = fmt(Math.hypot(v.v || 0, v.vy || 0)) + ' m/s';
  }
}
$('#cards').onclick = async e => {
  const c = e.target.closest('.card'); if (!c) return;
  const act = e.target.closest('[data-cact]'), id = c.dataset.id, sp = spec(id), v = S.st.get(id);
  if (!act) return select(id);
  if (act.dataset.cact === 'hide') send({ op: 'hide', id, on: !(v && v.hid) });
  else if (await dialog('Aracı sil', `<div class="note"><b>${esc(sp.name)}</b> filodan kalıcı olarak kaldırılacak (tüm haritalardan). Yalnızca bu haritada görünmesin istiyorsan “Gizle”yi kullan.</div>`, 'Sil')) send({ op: 'removeVehicle', id });
};
$('#btnFleet').onclick = async () => {
  const r = await dialog('Filo', `<div class="menu"><button type="button" data-ret="show">Tüm araçları göster</button><button type="button" data-ret="hide">Tüm araçları gizle</button><button type="button" data-ret="hideSim">Simüle araçları gizle (yalnız gerçek araçlar kalsın)</button><button type="button" data-ret="delAll" class="danger">Tüm araçları sil…</button></div><div class="note">Gizleme haritaya özeldir: her haritada farklı araçlar etkin olabilir.</div>`, '');
  if (!r) return;
  if (r.ret === 'show' || r.ret === 'hide') for (const s of S.specs) send({ op: 'hide', id: s.id, on: r.ret === 'hide' });
  else if (r.ret === 'hideSim') for (const s of S.specs) send({ op: 'hide', id: s.id, on: s.driver.type === 'sim' });
  else if (r.ret === 'delAll' && await dialog('Tüm araçları sil', `<div class="note">Filodaki <b>${S.specs.length}</b> aracın tamamı silinecek. Haritalar kalır.</div>`, 'Hepsini sil')) send({ op: 'removeAllVehicles' });
};
function select(id) {
  if (id === S.sel) return;
  S.sel = id; S.dirty = false; S.hist = []; S.draft = selSpec() ? clone(selSpec()) : null;
  for (const el of $('#cards').children) el.classList.toggle('sel', el.dataset.id === id);
  renderPanel();
}

// ------------------------------------------------------------------ Harita cizimi
const cv = $('#map'), ctx = cv.getContext('2d');
const view = { s: 30, ox: 0, oy: 0 };
let cw = 0, ch = 0, drag = null, mouseW = null;
const X = x => view.ox + x * view.s, Y = y => view.oy - y * view.s;
const toW = (px, py) => ({ x: (px - view.ox) / view.s, y: (view.oy - py) / view.s });
const snap = p => ({ x: Math.round(p.x * 4) / 4, y: Math.round(p.y * 4) / 4 });
function resize() {
  const r = $('#stage').getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  cw = r.width; ch = r.height; cv.width = Math.round(cw * dpr); cv.height = Math.round(ch * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  if (!S.userView) fit();
}
function fit() {
  if (!S.world || !cw) return;
  view.s = Math.min((cw - 40) / S.world.width, (ch - 40) / S.world.height);
  view.ox = (cw - S.world.width * view.s) / 2; view.oy = (ch + S.world.height * view.s) / 2;
  S.userView = false;
}
new ResizeObserver(resize).observe($('#stage'));

function bodyPath(c, sp) {
  const off = sp.kinematics === 'ackermann' ? sp.wheelBase / 2 : 0, L = sp.length, W = sp.width, skid = sp.kinematics === 'skid';
  const r = sp.wheelRadius || 0.08, T = sp.trackWidth || W;
  c.fillStyle = '#05070a';
  if (skid) { const tw = W * 0.22; c.fillRect(off - L / 2, W / 2 - tw, L, tw); c.fillRect(off - L / 2, -W / 2, L, tw); }
  c.fillStyle = sp.color; c.globalAlpha = 0.92;
  const bw = skid ? W * 0.6 : W, bl = skid ? L * 0.9 : L, rad = Math.min(bl, bw) * 0.14;
  c.beginPath(); c.roundRect(off - bl / 2, -bw / 2, bl, bw, rad); c.fill(); c.globalAlpha = 1;
  c.fillStyle = '#05070a';
  const wheel = (x, y, a) => { c.save(); c.translate(x, y); c.rotate(a || 0); c.fillRect(-r, -0.03, 2 * r, 0.06); c.restore(); };
  if (sp.kinematics === 'diff') { wheel(0, T / 2 - 0.03); wheel(0, -T / 2 + 0.03); }
  else if (sp.kinematics === 'ackermann') { wheel(0, T / 2 - 0.03); wheel(0, -T / 2 + 0.03); wheel(sp.wheelBase, T / 2 - 0.03, sp._steer || 0); wheel(sp.wheelBase, -T / 2 + 0.03, sp._steer || 0); }
  else if (sp.kinematics === 'mecanum') { const wb = (sp.wheelBase || L * 0.7) / 2; for (const sx of [-1, 1]) for (const sy of [-1, 1]) wheel(sx * wb, sy * (T / 2 - 0.03)); }
  c.fillStyle = 'rgba(0,0,0,.5)';
  c.beginPath(); c.moveTo(off + bl / 2 - bl * 0.06, 0); c.lineTo(off + bl / 2 - bl * 0.3, bw * 0.3); c.lineTo(off + bl / 2 - bl * 0.3, -bw * 0.3); c.closePath(); c.fill();
}
function drawSensorsBody(c, sens, px) {
  // govde cercevesinde sensor yerlesimleri (px: bir pikselin metre karsiligi)
  for (const sn of sens) {
    const x = sn.x || 0, y = sn.y || 0, a = (sn.theta || 0) * DEG;
    if (sn.type === 'camera') { const f = (sn.fov || 76) * DEG / 2, R = sn.range || 5; c.fillStyle = 'rgba(90,169,255,.10)'; c.strokeStyle = 'rgba(90,169,255,.5)'; c.lineWidth = px; c.beginPath(); c.moveTo(x, y); c.arc(x, y, R, a - f, a + f); c.closePath(); c.fill(); c.stroke(); }
    else if (sn.type === 'ultrasonic') { const f = (sn.cone || 20) * DEG / 2, R = sn._d > 0 ? sn._d : (sn.range || 4); c.fillStyle = sn._d > 0 ? 'rgba(255,170,60,.28)' : 'rgba(255,170,60,.07)'; c.beginPath(); c.moveTo(x, y); c.arc(x, y, R, a - f, a + f); c.closePath(); c.fill(); }
    else if (sn.type === 'lidar') { c.fillStyle = '#ff5d5d'; c.beginPath(); c.arc(x, y, Math.max(3 * px, 0.035), 0, 7); c.fill(); if ((sn.fov || 360) < 359) { const f = sn.fov * DEG / 2; c.strokeStyle = 'rgba(255,93,93,.5)'; c.lineWidth = px; c.beginPath(); c.moveTo(x, y); c.arc(x, y, 0.6, a - f, a + f); c.closePath(); c.stroke(); } }
    else if (sn.type === 'line') { const n = sn.count || 8, spc = sn.spacing || 0.02; c.fillStyle = '#f5b301'; for (let i = 0; i < n; i++) c.fillRect(x - 1.5 * px, (i - (n - 1) / 2) * spc - 1.5 * px, 3 * px, 3 * px); }
  }
}
function draw() {
  requestAnimationFrame(draw);
  const w = S.world; if (!w || !cw) return;
  ctx.fillStyle = '#0a0c0f'; ctx.fillRect(0, 0, cw, ch);
  ctx.fillStyle = '#11161c'; ctx.fillRect(X(0), Y(w.height), w.width * view.s, w.height * view.s);
  for (const z of S.layers.site ? (w.zones || []) : []) { ctx.fillStyle = ZONE[z.type] ? ZONE[z.type].fill : 'transparent'; ctx.fillRect(X(z.x), Y(z.y + z.h), z.w * view.s, z.h * view.s); if (z.type !== 'roof' && view.s > 12) { ctx.fillStyle = 'rgba(255,255,255,.28)'; ctx.font = '10px sans-serif'; ctx.fillText(ZONE[z.type].name.toLowerCase(), X(z.x) + 4, Y(z.y + z.h) + 12); } }
  // izgara
  ctx.strokeStyle = 'rgba(255,255,255,.035)'; ctx.lineWidth = 1; ctx.beginPath();
  for (let x = 0; x <= w.width; x++) { ctx.moveTo(X(x) + .5, Y(0)); ctx.lineTo(X(x) + .5, Y(w.height)); }
  for (let y = 0; y <= w.height; y++) { ctx.moveTo(X(0), Y(y) + .5); ctx.lineTo(X(w.width), Y(y) + .5); }
  ctx.stroke();
  if (S.layers.scan && S.grid) { ctx.imageSmoothingEnabled = false; ctx.drawImage(S.grid.canvas, X(0), Y(S.grid.h * S.grid.res), S.grid.w * S.grid.res * view.s, S.grid.h * S.grid.res * view.s); }
  const site = S.layers.site;
  // serit
  ctx.strokeStyle = 'rgba(245,179,1,.55)'; ctx.lineWidth = Math.max(2, 0.05 * view.s); ctx.lineCap = 'round'; ctx.beginPath();
  for (const t of site ? w.tapes : []) { ctx.moveTo(X(t.x1), Y(t.y1)); ctx.lineTo(X(t.x2), Y(t.y2)); } ctx.stroke(); ctx.lineCap = 'butt';
  // engeller
  const dyn = new Map(S.dyn.map(d => [d.id, d]));
  for (const o of site ? w.obstacles : []) {
    const d = dyn.get(o.id), ox = d ? d.x : o.x, oy = d ? d.y : o.y;
    if (o.move) { ctx.strokeStyle = 'rgba(255,122,89,.25)'; ctx.setLineDash([4, 4]); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(X(o.x + o.w / 2), Y(o.y + o.h / 2)); ctx.lineTo(X(o.x + o.w / 2 + o.move.dx), Y(o.y + o.h / 2 + o.move.dy)); ctx.stroke(); ctx.setLineDash([]); }
    ctx.fillStyle = o.move ? '#7a3b2b' : '#2a3442'; ctx.strokeStyle = o.move ? '#ff7a59' : '#536379'; ctx.lineWidth = 1;
    ctx.fillRect(X(ox), Y(oy + o.h), o.w * view.s, o.h * view.s); ctx.strokeRect(X(ox) + .5, Y(oy + o.h) + .5, o.w * view.s, o.h * view.s);
  }
  // duvarlar + dis sinir
  ctx.strokeStyle = '#cdd6e1'; ctx.lineWidth = Math.max(2, 0.12 * view.s); ctx.lineCap = 'square'; ctx.beginPath();
  for (const s of site ? w.walls : []) { ctx.moveTo(X(s.x1), Y(s.y1)); ctx.lineTo(X(s.x2), Y(s.y2)); } ctx.stroke(); ctx.lineCap = 'butt';
  ctx.strokeStyle = '#6b7a8d'; ctx.lineWidth = 2; ctx.strokeRect(X(0), Y(w.height), w.width * view.s, w.height * view.s);
  // istasyonlar
  ctx.textAlign = 'center';
  for (const s of w.stations) {
    const col = s.type === 'charger' ? '#3ecf8e' : s.type === 'waypoint' ? '#f5b301' : '#5ad1e6', r = Math.max(6, 0.32 * view.s) * (s.type === 'waypoint' ? 0.7 : 1);
    ctx.strokeStyle = col; ctx.fillStyle = col + '22'; ctx.lineWidth = 1.5; ctx.beginPath();
    if (s.type === 'waypoint') { ctx.moveTo(X(s.x), Y(s.y) - r); ctx.lineTo(X(s.x) + r, Y(s.y)); ctx.lineTo(X(s.x), Y(s.y) + r); ctx.lineTo(X(s.x) - r, Y(s.y)); ctx.closePath(); } else ctx.arc(X(s.x), Y(s.y), r, 0, 7);
    ctx.fill(); ctx.stroke();
    if (s.theta != null) { ctx.beginPath(); ctx.moveTo(X(s.x), Y(s.y)); ctx.lineTo(X(s.x) + Math.cos(s.theta) * r * 1.6, Y(s.y) - Math.sin(s.theta) * r * 1.6); ctx.stroke(); }
    ctx.fillStyle = col; ctx.font = '600 11px sans-serif'; ctx.fillText((s.type === 'charger' ? '⚡ ' : '') + (view.s > 17 ? s.name : s.id), X(s.x), Y(s.y) + r + 13);
  }
  ctx.textAlign = 'left';
  // izler + yollar
  const hid = id => { const v = S.st.get(id); return !v || v.hid; };
  if (S.layers.trail) for (const [id, tr] of S.trails) { const sp = spec(id); if (!sp || tr.length < 2 || hid(id)) continue; ctx.strokeStyle = sp.color + '55'; ctx.lineWidth = 1.5; ctx.beginPath(); tr.forEach((p, i) => i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))); ctx.stroke(); }
  if (S.layers.path) for (const [id, p] of S.paths) { const sp = spec(id); if (!sp || !p || p.length < 2 || hid(id)) continue; ctx.strokeStyle = sp.color; ctx.lineWidth = id === S.sel ? 2.5 : 1.5; ctx.setLineDash([7, 5]); ctx.beginPath(); p.forEach((q, i) => i ? ctx.lineTo(X(q[0]), Y(q[1])) : ctx.moveTo(X(q[0]), Y(q[1]))); ctx.stroke(); ctx.setLineDash([]); const e = p[p.length - 1]; ctx.fillStyle = sp.color; ctx.beginPath(); ctx.arc(X(e[0]), Y(e[1]), 4, 0, 7); ctx.fill(); }
  // LiDAR
  if (S.layers.lidar) for (const [id, L] of S.lidar) {
    const sp = spec(id); if (!sp || hid(id)) continue;
    const selc = id === S.sel, c = Math.cos(L.pth), s = Math.sin(L.pth), mx = L.px + c * L.x - s * L.y, my = L.py + s * L.x + c * L.y;
    if (selc) { ctx.fillStyle = 'rgba(255,93,93,.05)'; ctx.beginPath(); ctx.moveTo(X(mx), Y(my)); for (let i = 0; i < L.ranges.length; i++) { const r = L.ranges[i] || L.range, a = L.pth + L.theta + L.angleMin + i * L.angleInc; ctx.lineTo(X(mx + r * Math.cos(a)), Y(my + r * Math.sin(a))); } ctx.closePath(); ctx.fill(); }
    ctx.fillStyle = selc ? '#ff5d5d' : sp.color + '88';
    for (let i = 0; i < L.ranges.length; i++) { const r = L.ranges[i]; if (!r) continue; const a = L.pth + L.theta + L.angleMin + i * L.angleInc; ctx.fillRect(X(mx + r * Math.cos(a)) - 1, Y(my + r * Math.sin(a)) - 1, 2, 2); }
  }
  // araclar
  for (const sp of S.specs) {
    const v = S.st.get(sp.id); if (!v || v.x == null || v.hid) continue;
    const selc = sp.id === S.sel, px = 1 / view.s;
    if (selc && sp.driver.type === 'sim' && v.odom && S.layers.sensors) { // odometri hayaleti
      ctx.save(); ctx.translate(X(v.odom[0]), Y(v.odom[1])); ctx.rotate(-v.odom[2]); ctx.scale(view.s, -view.s);
      const off = sp.kinematics === 'ackermann' ? sp.wheelBase / 2 : 0; ctx.strokeStyle = 'rgba(255,255,255,.4)'; ctx.lineWidth = px; ctx.setLineDash([4 * px, 3 * px]); ctx.strokeRect(off - sp.length / 2, -sp.width / 2, sp.length, sp.width); ctx.restore(); ctx.setLineDash([]);
    }
    ctx.save(); ctx.translate(X(v.x), Y(v.y)); ctx.rotate(-v.th); ctx.scale(view.s, -view.s);
    if (selc && S.layers.sensors) { const us = (v.sens.us || []); let ui = 0; drawSensorsBody(ctx, sp.sensors.map(sn => sn.type === 'ultrasonic' ? { ...sn, _d: (us[ui++] || {}).d || 0 } : sn), px); }
    sp._steer = v.steer || 0; bodyPath(ctx, sp); ctx.restore();
    const R = Math.max(11, Math.hypot(sp.length, sp.width) / 2 * view.s + 4);
    if (selc || v.estop || v.coll || v.safe === 'stop') { ctx.strokeStyle = v.estop || v.coll ? '#ff4d4f' : v.safe === 'stop' ? '#f5b301' : '#ffffffaa'; ctx.lineWidth = v.estop || v.coll ? 2.5 : 1.2; ctx.beginPath(); ctx.arc(X(v.x), Y(v.y), R, 0, 7); ctx.stroke(); }
    if (selc && v.sens.gps && v.sens.gps.fix && v.sens.gps.x != null && S.layers.sensors) { const gx = X(v.sens.gps.x), gy = Y(v.sens.gps.y); ctx.strokeStyle = '#7dd3fc'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(gx - 5, gy); ctx.lineTo(gx + 5, gy); ctx.moveTo(gx, gy - 5); ctx.lineTo(gx, gy + 5); ctx.stroke(); }
    ctx.font = '600 11px sans-serif'; ctx.textAlign = 'center'; ctx.fillStyle = '#000a'; const tw = ctx.measureText(sp.name).width; ctx.fillRect(X(v.x) - tw / 2 - 4, Y(v.y) - R - 17, tw + 8, 14);
    ctx.fillStyle = sp.driver.type === 'sim' ? '#e4eaf1' : '#ffb199'; ctx.fillText(sp.name, X(v.x), Y(v.y) - R - 6); ctx.textAlign = 'left';
  }
  // arac onizlemesi
  if (drag && drag.w0 && drag.cur) {
    const a = drag.w0, b = drag.cur; ctx.strokeStyle = '#f5b301'; ctx.fillStyle = 'rgba(245,179,1,.15)'; ctx.lineWidth = 2;
    if (drag.type === 'box' || drag.type === 'zone') { ctx.fillRect(X(Math.min(a.x, b.x)), Y(Math.max(a.y, b.y)), Math.abs(b.x - a.x) * view.s, Math.abs(b.y - a.y) * view.s); ctx.strokeRect(X(Math.min(a.x, b.x)), Y(Math.max(a.y, b.y)), Math.abs(b.x - a.x) * view.s, Math.abs(b.y - a.y) * view.s); }
    else if (drag.type !== 'pan') { ctx.beginPath(); ctx.moveTo(X(a.x), Y(a.y)); ctx.lineTo(X(b.x), Y(b.y)); ctx.stroke(); ctx.beginPath(); ctx.arc(X(a.x), Y(a.y), 5, 0, 7); ctx.fill(); ctx.stroke(); }
  }
  // olcek + imlec
  const m = mouseW; $('#hud').textContent = (m ? `x ${fmt(m.x, 2)}  y ${fmt(m.y, 2)} m   ·   ` : '') + `${fmt(view.s, 0)} px/m   ·   ${w.width}×${w.height} m`;
}

// ------------------------------------------------------------------ Harita etkilesimi
function vehicleAt(p) {
  let best = null, bd = Infinity;
  for (const sp of S.specs) { const v = S.st.get(sp.id); if (!v || v.hid) continue; const d = Math.hypot(v.x - p.x, v.y - p.y), r = Math.max(Math.hypot(sp.length, sp.width) / 2, 12 / view.s); if (d < r && d < bd) { bd = d; best = sp.id; } }
  return best;
}
function hint(text, ms) { const h = $('#hint'); h.textContent = text || ''; h.classList.toggle('show', !!text); clearTimeout(hint.t); if (text && ms) hint.t = setTimeout(() => h.classList.remove('show'), ms); }
function setTool(t) { S.tool = t; for (const b of $('#tools').querySelectorAll('button')) b.classList.toggle('on', b.dataset.tool === t); cv.style.cursor = t === 'select' ? 'default' : 'crosshair'; hint(TOOL_HINT[t]); }
$('#tools').onclick = e => { const b = e.target.closest('button'); if (b) setTool(b.dataset.tool); };
$('#layers').onchange = e => { S.layers[e.target.dataset.layer] = e.target.checked; };
const sendWorld = () => send({ op: 'world', world: S.world });
const ptr = e => { const r = cv.getBoundingClientRect(); return { x: e.clientX - r.left, y: e.clientY - r.top }; };
cv.oncontextmenu = e => e.preventDefault();
cv.onpointerdown = e => {
  const p = ptr(e), w = toW(p.x, p.y); cv.setPointerCapture(e.pointerId);
  if (e.button !== 0 || S.keys.has(' ')) { drag = { type: 'pan', px: p.x, py: p.y }; return; }
  const t = S.tool;
  if (t === 'select') { const id = vehicleAt(w); if (id) select(id); drag = { type: 'pan', px: p.x, py: p.y }; }
  else if (t === 'erase') eraseAt(w);
  else if ((t === 'goto' || t === 'pose') && !S.sel) hint('Önce bir araç seçin', 2000);
  else drag = { type: t, w0: (t === 'goto' || t === 'pose' || t === 'station') ? w : snap(w), cur: null };
};
cv.onpointermove = e => {
  const p = ptr(e); mouseW = toW(p.x, p.y);
  if (!drag) return;
  if (drag.type === 'pan') { view.ox += p.x - drag.px; view.oy += p.y - drag.py; drag.px = p.x; drag.py = p.y; S.userView = true; }
  else drag.cur = (drag.type === 'goto' || drag.type === 'pose' || drag.type === 'station') ? mouseW : snap(mouseW);
};
cv.onpointerup = async e => {
  const d = drag; drag = null; if (!d || d.type === 'pan') return;
  const a = d.w0, b = d.cur || a, len = Math.hypot(b.x - a.x, b.y - a.y), ang = Math.atan2(b.y - a.y, b.x - a.x);
  const W = S.world;
  if (d.type === 'goto') send({ op: 'goto', id: S.sel, x: a.x, y: a.y, theta: len > 0.3 ? ang : null });
  else if (d.type === 'pose') send({ op: 'setPose', id: S.sel, x: a.x, y: a.y, theta: len > 0.2 ? ang : (selSt() ? selSt().th : 0) });
  else if (d.type === 'wall' && len > 0.2) { W.walls.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y }); sendWorld(); }
  else if (d.type === 'tape' && len > 0.2) { W.tapes.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y }); sendWorld(); }
  else if ((d.type === 'box' || d.type === 'zone') && Math.abs(b.x - a.x) > 0.1 && Math.abs(b.y - a.y) > 0.1) {
    const r = { x: Math.min(a.x, b.x), y: Math.min(a.y, b.y), w: Math.abs(b.x - a.x), h: Math.abs(b.y - a.y) };
    if (d.type === 'zone') {
      const res = await dialog('Zemin bölgesi', `<div class="f"><label>Zemin tipi</label><select name="type">${Object.entries(ZONE).map(([k, z]) => `<option value="${k}">${z.name}</option>`).join('')}</select></div><div class="note">Çim, çakıl, kum ve çamur hızı düşürür, kaymayı ve enerji tüketimini artırır; arazi kabiliyeti yüksek araçlar daha az etkilenir. Kapalı alanda GPS çalışmaz.</div>`);
      if (res) { (W.zones = W.zones || []).push({ type: res.type, ...r }); sendWorld(); }
    } else {
      const o = { id: 'o' + Date.now().toString(36), ...r };
      if (e.shiftKey) {
        const res = await dialog('Hareketli engel', `<div class="row"><div class="f"><label>Gidiş Δx (m)</label><input name="dx" type="number" step="0.5" value="4"></div><div class="f"><label>Gidiş Δy (m)</label><input name="dy" type="number" step="0.5" value="0"></div><div class="f"><label>Hız (m/s)</label><input name="speed" type="number" step="0.1" value="0.6"></div></div>`);
        if (res) o.move = { dx: +res.dx || 0, dy: +res.dy || 0, speed: +res.speed || 0.5 };
      }
      W.obstacles.push(o); sendWorld();
    }
  } else if (d.type === 'station') {
    const res = await dialog('Yeni istasyon', `<div class="f"><label>Ad</label><input name="name" type="text" value="İstasyon ${W.stations.length + 1}"></div><div class="f"><label>Tip</label><select name="type"><option value="station">İstasyon (yükleme / boşaltma)</option><option value="waypoint">Devriye noktası</option><option value="charger">Şarj istasyonu</option></select></div>`);
    if (res && res.name.trim()) { let id = res.name.trim().split(/\s+/).map(x => x[0]).join('').toUpperCase().slice(0, 3) || 'N', i = 1, base = id; while (W.stations.some(s => s.id === id)) id = base + (++i); W.stations.push({ id, name: res.name.trim(), x: +a.x.toFixed(2), y: +a.y.toFixed(2), theta: len > 0.3 ? ang : null, type: res.type }); sendWorld(); renderPanel(); }
  }
};
cv.onwheel = e => { e.preventDefault(); const p = ptr(e), w = toW(p.x, p.y), k = Math.exp(-e.deltaY * 0.0012); view.s = clamp(view.s * k, 4, 400); view.ox = p.x - w.x * view.s; view.oy = p.y + w.y * view.s; S.userView = true; };
function segDist(p, s) { const ex = s.x2 - s.x1, ey = s.y2 - s.y1, l2 = ex * ex + ey * ey; const t = clamp(l2 ? ((p.x - s.x1) * ex + (p.y - s.y1) * ey) / l2 : 0, 0, 1); return Math.hypot(p.x - s.x1 - t * ex, p.y - s.y1 - t * ey); }
function eraseAt(p) {
  const W = S.world, tol = Math.max(0.25, 8 / view.s), inside = r => p.x >= r.x && p.x <= r.x + r.w && p.y >= r.y && p.y <= r.y + r.h;
  let i;
  if ((i = W.stations.findIndex(s => Math.hypot(s.x - p.x, s.y - p.y) < Math.max(0.45, tol))) >= 0) W.stations.splice(i, 1);
  else if ((i = W.obstacles.findIndex(inside)) >= 0) W.obstacles.splice(i, 1);
  else if ((i = W.walls.findIndex(s => segDist(p, s) < tol)) >= 0) W.walls.splice(i, 1);
  else if ((i = W.tapes.findIndex(s => segDist(p, s) < tol)) >= 0) W.tapes.splice(i, 1);
  else if (W.zones && (i = W.zones.map(inside).lastIndexOf(true)) >= 0) W.zones.splice(i, 1);
  else return;
  sendWorld(); renderPanel();
}

// ------------------------------------------------------------------ Pencere (modal)
function dialog(title, body, okLabel = 'Tamam', noCancel) {
  return new Promise(resolve => {
    const m = $('#modal'); m.hidden = false;
    m.innerHTML = `<form class="dialog"><h3>${esc(title)}</h3>${body}<div class="btns">${noCancel ? '' : '<button type="button" data-x>İptal</button>'}${okLabel ? `<button class="on" type="submit">${esc(okLabel)}</button>` : ''}</div></form>`;
    const f = m.firstChild, done = v => { m.hidden = true; m.innerHTML = ''; resolve(v); };
    f.onsubmit = e => { e.preventDefault(); const o = {}; for (const el of f.querySelectorAll('[name]')) o[el.name] = el.type === 'checkbox' ? el.checked : el.value; done(o); };
    f.onclick = e => { if (e.target.closest('[data-x]')) done(null); const r = e.target.closest('[data-ret]'); if (r) done({ ret: r.dataset.ret }); };
    m.onpointerdown = e => { if (e.target === m) done(null); };
    const first = f.querySelector('input,select'); if (first) { first.focus(); if (first.select) first.select(); }
  });
}
function download(name, data, type) { const a = document.createElement('a'); a.href = URL.createObjectURL(data instanceof Blob ? data : new Blob([data], { type: type || 'application/json' })); a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000); }
function pickFile() { return new Promise(res => { const i = document.createElement('input'); i.type = 'file'; i.accept = '.json'; i.onchange = () => { const f = i.files[0]; if (!f) return res(null); const r = new FileReader(); r.onload = () => { try { res(JSON.parse(r.result)); } catch (e) { hint('Geçersiz JSON dosyası', 2500); res(null); } }; r.readAsText(f); }; i.click(); }); }

// ------------------------------------------------------------------ Ust cubuk
$('#btnPlay').onclick = () => send({ op: 'sim', action: S.sim.running ? 'pause' : 'play' });
$('#selSpeed').onchange = e => send({ op: 'sim', speed: +e.target.value });
$('#btnReset').onclick = () => { send({ op: 'sim', action: 'reset' }); S.trails.clear(); };
$('#btnSave').onclick = () => send({ op: 'save' });
$('#btnFit').onclick = fit;
$('#btnEstop').onclick = () => send({ op: 'estop', id: 'all', on: ![...S.st.values()].some(v => v.estop) });
$('#btnLogToggle').onclick = () => { const c = $('#log').classList.toggle('closed'); $('#btnLogToggle').textContent = c ? '▴' : '▾'; };
$('#selMap').onchange = e => { send({ op: 'mapSelect', map: e.target.value }); e.target.blur(); };
const curMapName = () => (S.maps.find(m => m.id === S.activeMap) || { name: '' }).name;
async function newMapDialog() {
  const s = await dialog('Yeni harita', `<div class="f"><label>Harita adı</label><input name="name" type="text" value="Harita ${S.maps.length + 1}"></div>
    <div class="row"><div class="f"><label>Genişlik (m)</label><input name="w" type="number" value="30" min="5" max="300"></div><div class="f"><label>Yükseklik (m)</label><input name="h" type="number" value="20" min="5" max="300"></div></div>
    <div class="f"><label>Zemin</label><select name="kind"><option value="roof">Kapalı alan — fabrika / depo (GPS yok)</option><option value="grass">Açık arazi — çim zemin</option><option value="open">Açık alan — düz beton</option></select></div>
    <div class="note">Harita boş açılır; üstteki Duvar, Engel, İstasyon, Şerit ve Zemin araçlarıyla sıfırdan çizersin. Araçlar boş bir köşeye yerleştirilir; <b>Konum</b> aracıyla taşıyabilir, istemediklerini <b>Gizle</b> ile bu haritada devre dışı bırakabilirsin.</div>`, 'Oluştur');
  if (!s) return;
  const W = clamp(+s.w || 30, 5, 300), H = clamp(+s.h || 20, 5, 300);
  send({ op: 'mapNew', name: s.name.trim() || 'Yeni harita', world: { width: W, height: H, walls: [], obstacles: [], stations: [], tapes: [], zones: s.kind === 'open' ? [] : [{ type: s.kind, x: 0, y: 0, w: W, h: H }] } });
}
$('#btnMenu').onclick = async () => {
  const r = await dialog('Harita: ' + curMapName(), `<div class="menu">
    <h5>Haritalar (${S.maps.length})</h5>
    <button type="button" data-ret="new">Yeni harita oluştur…</button><button type="button" data-ret="copy">Bu haritayı çoğalt</button><button type="button" data-ret="rename">Yeniden adlandır…</button>
    <h5>Bu harita</h5>
    <button type="button" data-ret="clear">İçeriği temizle… (duvar, engel, istasyon, şerit, zemin)</button>
    <button type="button" data-ret="size">Boyutu değiştir…</button>
    <button type="button" data-ret="exp">Dışa aktar (JSON)</button><button type="button" data-ret="imp">İçe aktar… (yeni harita olarak)</button>
    <button type="button" data-ret="del" class="danger">Bu haritayı sil…</button>
    <h5>Proje</h5>
    <button type="button" data-ret="def" class="danger">Varsayılan projeye dön… (tüm haritalar + filo)</button></div>
    <div class="note">Proje dosyası: <code>${esc(S.dataDir)}/fleet.json</code> · Sürüm ${esc(S.version)}<br>Değişiklikler <b>Kaydet</b>'e basınca diske yazılır.</div>`, '');
  if (!r) return;
  const W = S.world;
  if (r.ret === 'new') newMapDialog();
  else if (r.ret === 'copy') send({ op: 'mapNew', name: curMapName() + ' kopya', copy: true });
  else if (r.ret === 'rename') { const s = await dialog('Haritayı yeniden adlandır', `<div class="f"><label>Ad</label><input name="name" type="text" value="${esc(curMapName())}"></div>`); if (s && s.name.trim()) send({ op: 'mapRename', map: S.activeMap, name: s.name.trim() }); }
  else if (r.ret === 'exp') download((curMapName() || 'harita').replace(/[^\w-]+/g, '_') + '.json', JSON.stringify(W, null, 2));
  else if (r.ret === 'imp') { const w = await pickFile(); if (w && w.width && w.height) send({ op: 'mapNew', name: 'İçe aktarılan harita', world: w }); else if (w) hint('Bu dosya bir harita değil', 2500); }
  else if (r.ret === 'clear') {
    const n = { walls: W.walls.length, obstacles: W.obstacles.length, stations: W.stations.length, tapes: W.tapes.length, zones: (W.zones || []).length };
    const L = { walls: 'Duvarlar', obstacles: 'Engeller', stations: 'İstasyonlar', tapes: 'Şeritler', zones: 'Zemin bölgeleri' };
    const s = await dialog('İçeriği temizle', `<div class="checks">${Object.keys(L).map(k => `<label><input type="checkbox" name="${k}" checked> ${L[k]} (${n[k]})</label>`).join('')}</div><div class="note">İşaretli öğeler <b>${esc(curMapName())}</b> haritasından silinir. Harita ve araçlar kalır.</div>`, 'Temizle');
    if (s) { for (const k of Object.keys(L)) if (s[k]) W[k] = []; sendWorld(); renderPanel(); }
  }
  else if (r.ret === 'size') { const s = await dialog('Harita boyutu', `<div class="row"><div class="f"><label>Genişlik (m)</label><input name="w" type="number" value="${W.width}"></div><div class="f"><label>Yükseklik (m)</label><input name="h" type="number" value="${W.height}"></div></div><div class="note">Sol alt köşe sabit kalır. Küçültürsen dışarıda kalan öğeler silinmez; görünmez olur.</div>`); if (s) { W.width = clamp(+s.w || W.width, 5, 300); W.height = clamp(+s.h || W.height, 5, 300); sendWorld(); S.userView = false; setTimeout(fit, 150); } }
  else if (r.ret === 'del') {
    if (S.maps.length < 2) return hint('Son harita silinemez — önce yeni bir harita oluştur', 3000);
    if (await dialog('Haritayı sil', `<div class="note"><b>${esc(curMapName())}</b> haritası tüm içeriğiyle silinecek. Araçlar silinmez.</div>`, 'Haritayı sil')) send({ op: 'mapDelete', map: S.activeMap });
  }
  else if (r.ret === 'def') { if (await dialog('Varsayılan projeye dön', `<div class="note">Tüm haritaların ve filon silinip örnek proje yüklenecek. Bu işlem geri alınamaz.</div>`, 'Sıfırla')) { send({ op: 'factoryReset' }); S.trails.clear(); } }
};
$('#btnAdd').onclick = async () => {
  const r = await dialog('Araç ekle', `<div class="f"><label>Şablon</label><select name="p">${S.presets.map((p, i) => `<option value="${i}">${esc(p.label)}</option>`).join('')}</select></div><div class="f"><label>Ad (boş bırakılırsa şablon adı)</label><input name="name" type="text" value=""></div><div class="note">Gerçek araç şablonlarında bağlantı ayarını “Bağlantı” sekmesinden yapıp <b>Bağlan</b>’a basın. Araç, simüle araçlarla aynı haritada görünür.</div>`, 'Ekle');
  if (!r) return;
  const sp = clone(S.presets[+r.p].spec), c = toW(cw / 2, ch / 2);
  if (r.name.trim()) sp.name = r.name.trim();
  sp.start = { x: clamp(c.x, 1, S.world.width - 1), y: clamp(c.y, 1, S.world.height - 1), theta: 0 };
  const before = new Set(S.specs.map(s => s.id));
  send({ op: 'addVehicle', spec: sp });
  setTimeout(() => { const n = S.specs.find(s => !before.has(s.id)); if (n) select(n.id); }, 250);
};
$('#tabs').onclick = e => { const b = e.target.closest('button'); if (!b) return; S.tab = b.dataset.tab; for (const x of $('#tabs').children) x.classList.toggle('on', x === b); if (S.tab === 'link' && !S.portsLoaded) send({ op: 'listPorts' }); renderPanel(); };

// ------------------------------------------------------------------ Sag panel
const panel = $('#panel');
function head(sp) { return `<div class="title"><span class="badge ${sp.class === 'UGV' ? 'ugv' : 'agv'}">${sp.class}</span><b>${esc(sp.name)}</b><span class="badge ${sp.driver.type !== 'sim' ? 'real' : ''}">${DRV[sp.driver.type]}</span></div>`; }
function renderPanel() {
  const sp = selSpec();
  if (!sp) { panel.innerHTML = '<div class="empty">Soldan bir araç seçin ya da “+ Araç” ile yeni araç ekleyin.</div>'; return; }
  const keep = panel.scrollTop;
  panel.innerHTML = ({ control: renderControl, sensors: renderSensors, design: renderDesign, code: renderCode, link: renderLink })[S.tab](sp);
  panel.scrollTop = keep;
  if (S.tab === 'control') initJoy();
  if (S.tab === 'design') drawPreview();
  if (S.tab === 'code') { const c = $('#console'); (S.consoles[sp.id] || []).forEach(appendConsole); c.scrollTop = c.scrollHeight; }
  updatePanel(true);
}
const stepLabel = s => s.type === 'goto' ? `Git → ${esc((S.world.stations.find(x => x.id === s.station) || { name: s.station }).name)}` : s.type === 'wait' ? `Bekle ${s.sec} sn` : s.type === 'load' ? `Yük al ${s.kg} kg` : 'Yük bırak';
function renderControl(sp) {
  const M = S.missions[sp.id] = S.missions[sp.id] || { steps: [], loop: true }, sim = sp.driver.type === 'sim';
  const kin = sp.kinematics;
  return head(sp) + `
  <div class="statusline"><span data-live="status">—</span></div>
  <div class="seg">${[['manual', 'Manuel'], ['auto', 'Otonom'], ['script', 'Kod']].map(([m, l]) => `<button data-act="mode:${m}" data-mode="${m}">${l}</button>`).join('')}</div>
  <div class="row"><button class="estop" data-act="vestop" id="vEstop">ACİL DURDUR</button></div>
  <h4>Manuel sürüş</h4>
  <div class="joy"><canvas id="joy" width="140" height="140"></canvas><div class="keys"><kbd>W</kbd><kbd>S</kbd> ileri / geri<br><kbd>A</kbd><kbd>D</kbd> ${kin === 'ackermann' ? 'direksiyon' : 'dönüş'}${kin === 'mecanum' ? '<br><kbd>Q</kbd><kbd>E</kbd> yana kayma' : ''}<br><kbd>Boşluk</kbd> dur / rotayı iptal<br><span data-live="telehint"></span></div></div>
  <div class="row"><span class="fix">Hız sınırı</span><input type="range" id="spd" min="10" max="100" value="${S.speedPct}"><span class="fix mono" id="spdV">${S.speedPct}%</span></div>
  ${sim ? `<h4>Yük <span class="mono" data-live="payload"></span></h4><input type="range" id="payload" min="0" max="${sp.maxPayload}" step="${sp.maxPayload > 20 ? 5 : 0.1}" value="0">` : ''}
  <h4>Hedefe git</h4>
  <div class="wrap">${S.world.stations.map(s => `<button data-act="goto:${esc(s.id)}" title="${esc(s.name)}">${s.type === 'charger' ? '⚡ ' : ''}${esc(s.name)}</button>`).join('') || '<span class="note">Haritada istasyon yok. “İstasyon” aracıyla ekleyin.</span>'}</div>
  <div class="row"><button data-act="navcancel">Rotayı iptal et</button></div>
  <div class="note">Haritada serbest bir noktaya göndermek için üstteki <b>Git</b> aracını kullanın.</div>
  <h4>Görev <span class="mono" data-live="mission"></span></h4>
  <div class="steps">${M.steps.map((s, i) => `<div class="step" data-step="${i}"><i>${i + 1}</i><span>${stepLabel(s)}</span><button class="x" data-act="mdel:${i}">×</button></div>`).join('') || '<div class="note">Adım ekleyin: istasyona git, bekle, yük al / bırak.</div>'}</div>
  <div class="row"><select id="mType">${S.world.stations.map(s => `<option value="goto:${esc(s.id)}">Git → ${esc(s.name)}</option>`).join('')}<option value="wait">Bekle (sn)</option>${sim ? '<option value="load">Yük al (kg)</option><option value="unload">Yük bırak</option>' : ''}</select><input id="mArg" type="number" class="fix" style="width:64px" value="5" title="Süre (sn) / yük (kg)"><button class="fix" data-act="madd">Ekle</button></div>
  <div class="row"><label class="fix"><input type="checkbox" id="mLoop" ${M.loop ? 'checked' : ''}> Döngü</label><span></span><button class="fix accent" data-act="mstart">Görevi başlat</button><button class="fix" data-act="mclear">Temizle</button></div>`;
}
function renderSensors(sp) {
  return head(sp) + `
  <div class="kv">
    <span>Konum</span><span data-live="pos"></span><span>Hız (v, ω)</span><span data-live="vel"></span><span>Komut (v, ω)</span><span data-live="cmd"></span>
    <span>Odometri</span><span data-live="odom"></span><span>Odometri hatası</span><span data-live="oerr"></span><span>Kat edilen yol</span><span data-live="dist"></span>
    <span>Batarya</span><span data-live="batt"></span><span>Yük</span><span data-live="payload"></span><span>Zemin</span><span data-live="terrain"></span><span>Öndeki engel</span><span data-live="gap"></span>
  </div>
  <h4>LiDAR <span class="mono" data-live="lidarinfo"></span></h4><canvas id="polar" class="block" width="250" height="250"></canvas>
  <h4>LiDAR haritalama <span class="mono" data-live="mapinfo"></span></h4>
  ${sp.sensors.some(x => x.type === 'lidar') ? `<div class="row"><button data-act="maptoggle" id="mapBtn">Haritalamayı başlat</button><button data-act="mapclear">Haritayı temizle</button></div>
  ${sp.driver.type === 'sim' ? `<div class="row"><span class="fix">Konum kaynağı</span><select id="mapSrc"><option value="pose">Gerçek konum (ideal lokalizasyon)</option><option value="odom">Odometri (kayma haritaya yansır)</option></select></div>` : ''}
  <div class="row"><button data-act="mappgm">Dışa aktar: PGM + YAML</button><button data-act="mappng" class="fix">PNG</button></div>
  <div class="note">Başlat, sonra aracı elle sür ya da hedefe gönder: her LiDAR taraması doluluk ızgarasına işlenir (açık mavi = boş, beyaz = engel). Üstteki <b>Saha</b> katmanını kapatırsan yalnızca aracın çıkardığı haritayı görürsün. PGM + YAML çıktısı ROS 2 <code>map_server</code> / Nav2 biçimindedir. Birden çok araç aynı haritaya katkı verebilir.</div>` : '<div class="note">Bu araçta LiDAR yok. Tasarım sekmesinden LiDAR ekleyince haritalama açılır.</div>'}
  <h4>Hız grafiği <span style="text-transform:none;letter-spacing:0"><span style="color:#3ecf8e">gerçek</span> · <span style="color:#f5b301">komut</span> · <span style="color:#5aa9ff">ω</span></span></h4><canvas id="chart" class="block" width="340" height="96"></canvas>
  <h4>Sensör verileri</h4><div id="sensList"></div>`;
}
const FIELDS = [
  ['Genel'], ['name', 'Ad', 'text'], ['class', 'Sınıf', 'select', [['AGV', 'AGV (iç lojistik)'], ['UGV', 'UGV (arazi)']]], ['color', 'Renk', 'color'],
  ['kinematics', 'Sürüş tipi', 'select', [['diff', 'Diferansiyel'], ['skid', 'Paletli / skid-steer'], ['ackermann', 'Ackermann'], ['mecanum', 'Mecanum (omni)']]],
  ['Gövde'], ['length', 'Uzunluk (m)'], ['width', 'Genişlik (m)'], ['mass', 'Boş kütle (kg)'], ['maxPayload', 'Azami yük (kg)'],
  ['Tahrik'], ['wheelRadius', 'Teker / tambur yarıçapı (m)'], ['trackWidth', 'İz genişliği (m)'], ['wheelBase', 'Dingil mesafesi (m)'], ['maxSteer', 'Azami direksiyon açısı (°)'],
  ['maxSpeed', 'Azami hız (m/s)'], ['maxAngular', 'Azami dönüş hızı (rad/s)'], ['maxAccel', 'Azami ivme (m/s²)'], ['maxAngAccel', 'Açısal ivme (rad/s²)'], ['motorTau', 'Motor tepki süresi (s)'],
  ['offroad', 'Arazi kabiliyeti (0–1)'], ['turnSlip', 'Dönüş kayması (0–0.8, paletli)'],
  ['Enerji'], ['battery.voltage', 'Nominal gerilim (V)'], ['battery.capacityWh', 'Kapasite (Wh)'], ['battery.chargeW', 'Şarj gücü (W)'], ['idlePower', 'Boşta güç (W)'],
  ['Enkoder'], ['encoder.ticksPerRev', 'Tik / tur (0 = enkoder yok)'], ['encoder.noise', 'Gürültü (oran)'],
  ['Güvenlik'], ['safety.enabled', 'Engelde otomatik yavaşla / dur', 'check'], ['safety.stopDist', 'Durma mesafesi (m)'],
];
const SF = {
  lidar: [['x', 'x (m)'], ['y', 'y (m)'], ['theta', 'açı (°)'], ['range', 'menzil (m)'], ['fov', 'FOV (°)'], ['rays', 'ışın'], ['noise', 'gürültü (m)'], ['rate', 'Hz']],
  ultrasonic: [['x', 'x (m)'], ['y', 'y (m)'], ['theta', 'açı (°)'], ['range', 'menzil (m)'], ['cone', 'koni (°)']],
  camera: [['x', 'x (m)'], ['y', 'y (m)'], ['theta', 'açı (°)'], ['fov', 'FOV (°)'], ['range', 'menzil (m)']],
  line: [['x', 'x (m)'], ['count', 'göz sayısı'], ['spacing', 'aralık (m)']], gps: [['noise', 'gürültü (m)'], ['rate', 'Hz']], imu: [['noise', 'gürültü']], bumper: [],
};
const SNEW = {
  lidar: { type: 'lidar', name: 'LiDAR', x: 0, y: 0, theta: 0, range: 12, fov: 360, rays: 360, noise: 0.01, rate: 8 }, ultrasonic: { type: 'ultrasonic', name: 'Ultrasonik', x: 0.3, y: 0, theta: 0, range: 4, cone: 20 },
  camera: { type: 'camera', name: 'Kamera', x: 0.3, y: 0, theta: 0, fov: 76, range: 5 }, line: { type: 'line', name: 'Şerit sensörü', x: 0.2, count: 8, spacing: 0.015 },
  gps: { type: 'gps', name: 'GPS', noise: 0.6, rate: 5 }, imu: { type: 'imu', name: 'IMU', noise: 0.004 }, bumper: { type: 'bumper', name: 'Tampon' },
};
const STYPE = { lidar: 'LiDAR', ultrasonic: 'Ultrasonik', camera: 'Kamera', line: 'Şerit', gps: 'GPS', imu: 'IMU', bumper: 'Tampon' };
function field(path, label, type, opts, d) {
  const v = getPath(d, path);
  let inp;
  if (type === 'select') inp = `<select data-path="${path}" data-re="1">${opts.map(o => `<option value="${o[0]}" ${o[0] === v ? 'selected' : ''}>${o[1]}</option>`).join('')}</select>`;
  else if (type === 'check') inp = `<input type="checkbox" data-path="${path}" ${v ? 'checked' : ''} style="justify-self:start;width:auto">`;
  else if (type === 'color') inp = `<input type="color" data-path="${path}" value="${esc(v || '#f5b301')}" style="padding:0;height:26px">`;
  else if (type === 'text') inp = `<input type="text" data-path="${path}" value="${esc(v == null ? '' : v)}">`;
  else inp = `<input type="number" step="any" data-path="${path}" value="${v == null ? '' : v}">`;
  return `<label>${label}</label>${inp}`;
}
function derived(d) {
  const m = +d.mass, P = +d.maxPayload, r = +d.wheelRadius, v = +d.maxSpeed, a0 = +d.maxAccel, tpr = +(d.encoder || {}).ticksPerRev || 0;
  const a = a0 * clamp(m / (m + P) * 1.6, 0.25, 1), F = (m + P) * (a0 + 9.81 * 0.025), pw = (+d.idlePower || 12) + (m + P / 2) * 9.81 * 0.025 * (v * 0.6) / 0.7, hrs = (+(d.battery || {}).capacityWh || 0) / pw;
  const rows = [
    ['Teker devri (azami hızda)', fmt(v / (2 * Math.PI * r) * 60, 0) + ' rpm'],
    ['Gerekli çekiş torku (tam yük, 2 tahrik tekeri)', fmt(F * r / 2, 2) + ' N·m / teker'],
    ['Fren mesafesi (tam yük, azami hız)', fmt(v * v / (2 * a), 2) + ' m'],
    ['Enkoder çözünürlüğü', tpr > 0 ? fmt(2 * Math.PI * r / tpr * 1000, 3) + ' mm / tik' : 'enkoder yok → açık çevrim odometri'],
    ['Tahmini çalışma süresi (yarı yük, %60 hız)', fmt(hrs, 1) + ' sa ≈ ' + fmt(hrs * v * 0.6 * 3.6, 1) + ' km'],
    ['Planlayıcı güvenlik yarıçapı', fmt(0.5 * Math.hypot(d.length, d.width) * 0.85 + 0.08, 2) + ' m'],
  ];
  if (d.kinematics === 'ackermann') rows.push(['Asgari dönüş yarıçapı', fmt(d.wheelBase / Math.tan((d.maxSteer || 30) * DEG), 2) + ' m']);
  return rows.map(x => `${x[0]}: <b>${x[1]}</b>`).join('<br>');
}
function renderDesign(sp) {
  const d = S.draft || sp;
  const show = p => !((p === 'maxSteer' && d.kinematics !== 'ackermann') || (p === 'wheelBase' && (d.kinematics === 'diff' || d.kinematics === 'skid')) || (p === 'turnSlip' && d.kinematics !== 'skid') || (p === 'maxAngAccel' && d.kinematics === 'ackermann'));
  let html = head(sp) + `<canvas id="preview" class="block" width="340" height="170"></canvas><div class="derived" id="derived">${derived(d)}</div>`;
  let open = false;
  for (const f of FIELDS) {
    if (f.length === 1) { html += (open ? '</div>' : '') + `<h4>${f[0]}</h4><div class="form">`; open = true; }
    else if (show(f[0])) html += field(f[0], f[1], f[2], f[3], d);
  }
  html += `</div><h4>Sensörler <select id="sAdd" style="font-size:12px;text-transform:none;letter-spacing:0"><option value="">+ sensör ekle…</option>${Object.keys(SNEW).map(k => `<option value="${k}">${STYPE[k]}</option>`).join('')}</select></h4>`;
  html += d.sensors.map((sn, i) => `<div class="sensor"><div class="sh"><span class="badge">${STYPE[sn.type] || sn.type}</span><input type="text" data-path="sensors.${i}.name" value="${esc(sn.name || '')}" placeholder="ad"><button class="x" data-act="sdel:${i}" title="Sensörü kaldır">×</button></div>
    <div class="sg">${(SF[sn.type] || []).map(f => `<label>${f[1]}<input type="number" step="any" data-path="sensors.${i}.${f[0]}" value="${sn[f[0]] == null ? '' : sn[f[0]]}"></label>`).join('')}</div></div>`).join('') || '<div class="note">Sensör yok.</div>';
  html += `<div class="note">Konumlar gövde çerçevesindedir: x ileri, y sola (m); referans noktası tahrik aksının ortası.</div>
  <div class="row"><button class="on" data-act="apply" id="btnApply">Tasarımı uygula</button></div>
  <div class="row"><button data-act="dup">Çoğalt</button><button data-act="export">Dışa aktar</button><button data-act="import">İçe aktar</button><button class="danger" data-act="del">Sil</button></div>`;
  return html;
}
function drawPreview() {
  const c = $('#preview'); if (!c || !S.draft) return;
  const g = c.getContext('2d'), d = S.draft, s = Math.min(150 / Math.max(0.2, +d.length + 0.5), 110 / Math.max(0.2, +d.width + 0.4));
  g.setTransform(1, 0, 0, 1, 0, 0); g.clearRect(0, 0, c.width, c.height);
  g.save(); g.translate(c.width * 0.4, c.height / 2); g.scale(s, -s);
  const cap = Math.max(+d.length, 0.3) * 1.6, near = d.sensors.map(x => (x.type === 'camera' || x.type === 'ultrasonic') ? { ...x, range: Math.min(x.range || 4, cap) } : x);
  try { drawSensorsBody(g, near, 1 / s); bodyPath(g, { ...d, _steer: 0 }); drawSensorsBody(g, d.sensors.filter(x => x.type === 'lidar' || x.type === 'line'), 1 / s); } catch (e) { }
  g.restore(); g.fillStyle = '#8894a4'; g.font = '11px sans-serif'; g.fillText(`${fmt(d.length, 2)} × ${fmt(d.width, 2)} m · ön →`, 8, c.height - 8);
}
function renderCode(sp) {
  if (S.code[sp.id] == null) S.code[sp.id] = sp.script || '';
  return head(sp) + `
  <div class="row"><select id="exSel"><option value="">Örnek yükle…</option>${window.EXAMPLES.map((e, i) => `<option value="${i}">${esc(e.name)}</option>`).join('')}</select></div>
  <textarea id="code" spellcheck="false">${esc(S.code[sp.id])}</textarea>
  <div class="row"><button class="on" data-act="run">▶ Çalıştır</button><button data-act="stop">■ Durdur</button><span class="mono fix" data-live="run"></span></div>
  <h4>Konsol <button class="x" data-act="cclear" title="Temizle">⌫</button></h4><div id="console"></div>
  <div class="note"><b>API</b> — <code>loop(agv, dt)</code> 20 Hz çağrılır; isteğe bağlı <code>setup(agv)</code>. <code>agv</code>, <code>ugv</code> ve <code>robot</code> aynı nesnedir.<br>
  Okuma: <code>pose</code> <code>odom</code> <code>velocity</code> <code>lidar</code> <code>lidarMin(a°, b°)</code> <code>ultrasonic[]</code> <code>line</code> <code>camera[]</code> <code>imu</code> <code>gps</code> <code>encoders</code> <code>bumper</code> <code>battery</code> <code>payload</code> <code>terrain</code> <code>stations</code> <code>navigating</code> <code>time</code> <code>spec</code><br>
  Komut: <code>setVelocity(v, w, vy)</code> <code>stop()</code> <code>goTo('A')</code> / <code>goTo(x, y, θ)</code> <code>setPayload(kg)</code> <code>log(...)</code><br>
  Aynı betik gerçek araçta da çalışır (sürücü bağlıysa). Python / ROS 2 kodunu test etmek için “Bağlantı” sekmesine bakın.</div>`;
}
function appendConsole(m) { const c = $('#console'); if (!c) return; const el = document.createElement('div'); el.className = m.level; el.textContent = m.msg; c.appendChild(el); while (c.children.length > 200) c.firstChild.remove(); c.scrollTop = c.scrollHeight; }
function renderLink(sp) {
  const d = S.draft || sp, t = d.driver.type;
  let h = head(sp) + `<h4>Sürücü</h4><div class="form">${field('driver.type', 'Bağlantı tipi', 'select', [['sim', 'Simülasyon'], ['rosbridge', 'ROS 2 (rosbridge)'], ['serial', 'Seri port (Arduino)'], ['udp', 'UDP (ESP32 / Pi köprüsü)']], d)}`;
  if (t === 'rosbridge') h += field('driver.url', 'rosbridge adresi', 'text', null, d) + field('driver.namespace', 'Namespace', 'text', null, d) + ['cmd_vel', 'odom', 'scan', 'battery', 'pose', 'gps'].map(k => field('driver.topics.' + k, k === 'pose' ? 'pose (AMCL, isteğe bağlı)' : k === 'gps' ? 'gps (NavSatFix, isteğe bağlı)' : k, 'text', null, d)).join('');
  else if (t === 'serial') h += `<label>Port <button class="x" data-act="ports" title="Portları yenile">⟳</button></label><select data-path="driver.port"><option value="">— seçin —</option>${S.ports.map(p => `<option value="${esc(p.path)}" ${p.path === d.driver.port ? 'selected' : ''}>${esc(p.label)}</option>`).join('')}${d.driver.port && !S.ports.some(p => p.path === d.driver.port) ? `<option selected value="${esc(d.driver.port)}">${esc(d.driver.port)}</option>` : ''}</select>` + field('driver.baud', 'Baud', null, null, d);
  else if (t === 'udp') h += field('driver.host', 'Araç IP / ad', 'text', null, d) + field('driver.port', 'Araç portu', null, null, d) + field('driver.listenPort', 'Dinleme portu (PC)', null, null, d) + field('driver.oneWay', 'Telemetri yok (yalnız komut)', 'check', null, d);
  else h += field('rosExpose.enabled', 'Simüle aracı ROS 2’ye aç', 'check', null, d) + field('rosExpose.url', 'rosbridge adresi', 'text', null, d) + field('rosExpose.namespace', 'Namespace', 'text', null, d);
  h += `</div><div class="row"><button class="on" data-act="apply">Ayarları uygula</button></div>`;
  if (t !== 'sim') h += `<div class="row"><button class="accent" data-act="connect">Bağlan</button><button data-act="disconnect">Bağlantıyı kes</button></div><div class="statusline"><span data-live="conn">—</span></div>`;
  const notes = {
    sim: `Açıkken bu simüle araç <code>&lt;ns&gt;/scan</code>, <code>&lt;ns&gt;/odom</code> ve <code>/tf</code> yayınlar, <code>&lt;ns&gt;/cmd_vel</code> dinler (araç <b>Manuel</b> moddayken). Böylece kendi ROS 2 düğümlerini (Nav2, slam_toolbox, kendi kontrolcün) gerçek araca yüklemeden simülasyona karşı çalıştırabilirsin. Durum: <span data-live="ros"></span>`,
    rosbridge: `Araçta: <code>ros2 launch rosbridge_server rosbridge_websocket_launch.xml</code> (port 9090). Panel <code>cmd_vel</code> yayınlar; <code>odom</code>, <code>scan</code>, <code>battery_state</code> dinler. <code>pose</code> alanına <code>/amcl_pose</code> yazarsan harita konumu doğrudan oradan alınır. Kurulum: <code>ros2/</code> klasörü.`,
    serial: `Protokol (115200 baud): PC→araç <code>V,&lt;mm/s&gt;,&lt;mrad/s&gt;</code>; araç→PC <code>T,encL,encR,vbat_mV,bumper,us1_mm,…</code>. Hazır Arduino Mega kodu: <code>firmware/arduino_mega_agv</code>. Enkoder yoksa “Tik / tur” değerini 0 yap; konum komuttan tahmin edilir.`,
    udp: `JSON datagram: PC→araç <code>{"t":"cmd","v":0.5,"w":0.2}</code>; araç→PC <code>{"t":"tel","enc":[l,r],"bat":11.8,"us":[0.8],"lat":..,"lon":..}</code>. ESP32 kodu: <code>firmware/esp32_udp_agv</code>. Raspberry Pi + Arduino Mega için: <code>examples/pi_udp_serial_bridge.py</code>.`,
  };
  h += `<div class="note">${notes[t]}</div>`;
  if (t !== 'sim') h += `<div class="note">Gerçek aracın haritadaki yerini üstteki <b>Konum</b> aracıyla bir kez işaretle; sonrası odometriyle takip edilir.</div>`;
  h += `<h4>Kendi kodundan kontrol (WebSocket API)</h4><div class="note">Simüle ya da gerçek her araç aynı API ile sürülür. Örnek istemci: <code>examples/python_client.py</code></div><pre>ws://${esc(location.host)}/ws
→ {"op":"teleop","id":"${esc(sp.id)}","v":0.4,"w":0.2}
→ {"op":"goto","id":"${esc(sp.id)}","station":"B"}
← {"op":"state","vehicles":[{"id":…,"x":…,"sens":{…}}]}</pre>`;
  return h;
}

// Canli alanlar
function updatePanel(force) {
  const sp = selSpec(), v = selSt(); if (!sp || !v) return;
  const live = (k, t) => { const el = panel.querySelector(`[data-live="${k}"]`); if (el && el.textContent !== t) el.textContent = t; };
  if (S.tab === 'control') {
    live('status', statusText(v) + (v.safe === 'slow' ? ' · engel: yavaşlıyor' : '') + (v.terrain ? ' · zemin: ' + ZONE[v.terrain].name.toLowerCase() : ''));
    for (const b of panel.querySelectorAll('[data-mode]')) b.classList.toggle('on', b.dataset.mode === v.mode);
    const eb = $('#vEstop'); if (eb) { eb.classList.toggle('active', v.estop); eb.textContent = v.estop ? 'DURDURMAYI KALDIR' : 'ACİL DURDUR'; }
    live('payload', fmt(v.payload, v.payload < 20 ? 1 : 0) + ' kg');
    const pl = $('#payload'); if (pl && document.activeElement !== pl) pl.value = v.payload;
    live('mission', v.mission ? `adım ${Math.min(v.mission.i + 1, v.mission.n)}/${v.mission.n} · tur ${v.mission.laps}` : '');
    for (const el of panel.querySelectorAll('.step')) el.classList.toggle('cur', !!v.mission && +el.dataset.step === v.mission.i);
    live('telehint', v.mode !== 'manual' ? 'Sürmek için Manuel moda geçin' : '');
  } else if (S.tab === 'sensors') {
    live('pos', `${fmt(v.x)}, ${fmt(v.y)} m · ${fmt(v.th / DEG, 0)}°`); live('vel', `${fmt(v.v)} m/s · ${fmt(v.w)} rad/s`); live('cmd', `${fmt(v.cv)} m/s · ${fmt(v.cw)} rad/s`);
    live('odom', `${fmt(v.odom[0])}, ${fmt(v.odom[1])} m · ${fmt(v.odom[2] / DEG, 0)}°`);
    live('oerr', v.type === 'sim' ? fmt(Math.hypot(v.odom[0] - v.x, v.odom[1] - v.y)) + ' m' : '— (gerçek araç)');
    live('dist', fmt(v.dist, 1) + ' m'); live('batt', v.soc == null ? '—' : `${fmt(v.soc * 100, 1)} % · ${fmt(v.volt, 1)} V${v.pw != null ? ' · ' + fmt(v.pw, 0) + ' W' : ''}${v.chg ? ' ⚡' : ''}`);
    live('payload', fmt(v.payload, 1) + ' kg'); live('terrain', v.terrain ? ZONE[v.terrain].name : 'düz zemin'); live('gap', v.gap == null ? '—' : fmt(v.gap) + ' m' + (v.safe !== 'ok' ? (v.safe === 'stop' ? ' · DUR' : ' · yavaş') : ''));
    const mb = $('#mapBtn'); if (mb) { mb.classList.toggle('on', !!v.map); mb.textContent = v.map ? '■ Haritalamayı durdur' : '▶ Haritalamayı başlat'; }
    const ms = $('#mapSrc'); if (ms && document.activeElement !== ms && ms.value !== v.mapSrc) ms.value = v.mapSrc;
    live('mapinfo', S.grid ? `kapsama %${fmt(S.grid.known, 1)} · ${fmt(S.grid.res * 100, 0)} cm/hücre` : '');
    if (S.tickN % 2 === 0 || force) { drawPolar(); drawChart(); }
    if (S.tickN % 4 === 0 || force) $('#sensList').innerHTML = sensList(sp, v);
  } else if (S.tab === 'code') live('run', v.run ? '● çalışıyor' : '○ durdu');
  else if (S.tab === 'link') { live('conn', v.conn ? 'Bağlı — veri alınıyor' : 'Bağlı değil'); live('ros', v.ros == null ? 'kapalı' : v.ros ? 'rosbridge’e bağlı' : 'bağlanmaya çalışıyor…'); }
}
function sensList(sp, v) {
  const s = v.sens; let h = '';
  if (s.us) h += s.us.map(u => `<div class="usbar"><span>${esc(u.name || 'US')}</span><span class="bar"><i style="width:${u.d > 0 ? clamp(u.d / (u.range || 4) * 100, 2, 100) : 0}%;background:${u.d > 0 && u.d < 0.5 ? 'var(--danger)' : 'var(--accent)'}"></i></span><span>${u.d > 0 ? fmt(u.d) + ' m' : '∞'}</span></div>`).join('');
  if (s.line) h += `<div class="usbar"><span>Şerit</span><span class="bits">${s.line.bits.map(b => `<i class="${b ? 'on' : ''}"></i>`).join('')}</span><span>${s.line.offset == null ? 'yok' : fmt(s.line.offset * 1000, 0) + ' mm'}</span></div>`;
  h += '<div class="kv" style="margin-top:6px">';
  if (s.gps) h += `<span>GPS</span><span>${s.gps.fix ? `${fmt(s.gps.lat, 6)}, ${fmt(s.gps.lon, 6)}${s.gps.x != null ? ` (x ${fmt(s.gps.x, 1)}, y ${fmt(s.gps.y, 1)})` : ''}` : 'fix yok (kapalı alan)'}</span>`;
  if (s.imu) h += `<span>IMU</span><span>ω ${fmt(s.imu.yawRate, 3)} rad/s · yaw ${fmt(s.imu.yaw / DEG, 1)}°${s.imu.heading != null ? ' · pusula ' + fmt(s.imu.heading / DEG, 0) + '°' : ''}</span>`;
  h += `<span>Enkoder</span><span>${s.enc ? s.enc.join(' · ') : 'yok'}</span>`;
  if (s.bumper != null) h += `<span>Tampon</span><span style="color:${s.bumper ? 'var(--danger)' : 'inherit'}">${s.bumper ? 'TEMAS' : 'serbest'}</span>`;
  if (v.steer != null && sp.kinematics === 'ackermann') h += `<span>Direksiyon</span><span>${fmt(v.steer / DEG, 1)}°</span>`;
  if (s.cam) s.cam.forEach(c => { h += `<span>${esc(c.name || 'Kamera')}</span><span>${(c.detections || []).length ? c.detections.map(d => `${esc(d.name)} ${fmt(d.distance, 1)} m / ${fmt(d.bearing / DEG, 0)}°`).join('<br>') : 'tespit yok'}</span>`; });
  return h + '</div>';
}
function drawPolar() {
  const c = $('#polar'); if (!c) return; const g = c.getContext('2d'), L = S.lidar.get(S.sel), cx = c.width / 2, cy = c.height / 2, R = cx - 8;
  g.clearRect(0, 0, c.width, c.height);
  const el = panel.querySelector('[data-live="lidarinfo"]');
  if (!L) { g.fillStyle = '#8894a4'; g.font = '12px sans-serif'; g.textAlign = 'center'; g.fillText('Bu araçta LiDAR yok', cx, cy); g.textAlign = 'left'; if (el) el.textContent = ''; return; }
  let max = 0, min = Infinity; for (const r of L.ranges) if (r) { if (r > max) max = r; if (r < min) min = r; }
  const scale = Math.min(L.range, Math.max(2, Math.ceil(max))), k = R / scale;
  g.strokeStyle = '#28313d'; g.lineWidth = 1; for (let i = 1; i <= 4; i++) { g.beginPath(); g.arc(cx, cy, R * i / 4, 0, 7); g.stroke(); }
  g.beginPath(); g.moveTo(cx, cy - R); g.lineTo(cx, cy + R); g.moveTo(cx - R, cy); g.lineTo(cx + R, cy); g.stroke();
  g.fillStyle = '#ff5d5d';
  for (let i = 0; i < L.ranges.length; i++) { const r = L.ranges[i]; if (!r) continue; const a = L.theta + L.angleMin + i * L.angleInc; g.fillRect(cx - Math.sin(a) * r * k - 1, cy - Math.cos(a) * r * k - 1, 2, 2); }
  g.fillStyle = '#f5b301'; g.beginPath(); g.moveTo(cx, cy - 8); g.lineTo(cx + 5, cy + 5); g.lineTo(cx - 5, cy + 5); g.closePath(); g.fill();
  g.fillStyle = '#8894a4'; g.font = '10px monospace'; g.fillText(scale + ' m', cx + 3, cy - R + 10);
  if (el) el.textContent = `${L.ranges.length} ışın · en yakın ${min === Infinity ? '—' : fmt(min) + ' m'}`;
}
function drawChart() {
  const c = $('#chart'); if (!c) return; const g = c.getContext('2d'), sp = selSpec(), H = c.height, W = c.width, n = 220;
  g.clearRect(0, 0, W, H); g.strokeStyle = '#28313d'; g.beginPath(); g.moveTo(0, H / 2 + .5); g.lineTo(W, H / 2 + .5); g.stroke();
  const line = (idx, col, max) => { g.strokeStyle = col; g.lineWidth = 1.4; g.beginPath(); S.hist.forEach((h, i) => { const x = i / (n - 1) * W, y = H / 2 - clamp((h[idx] || 0) / max, -1, 1) * (H / 2 - 4); i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke(); };
  line(2, '#5aa9ff', sp.maxAngular); line(1, '#f5b301', sp.maxSpeed); line(0, '#3ecf8e', sp.maxSpeed);
}

// Panel olaylari
panel.oninput = e => {
  const t = e.target;
  if (t.id === 'spd') { S.speedPct = +t.value; $('#spdV').textContent = t.value + '%'; send({ op: 'speedScale', id: S.sel, value: S.speedPct / 100 }); }
  else if (t.id === 'payload') send({ op: 'payload', id: S.sel, kg: +t.value });
  else if (t.id === 'code') { S.code[S.sel] = t.value; }
  else if (t.dataset.path && S.draft) {
    const v = t.type === 'checkbox' ? t.checked : t.type === 'number' ? (t.value === '' ? null : +t.value) : t.value;
    setPath(S.draft, t.dataset.path, v); S.dirty = true;
    if (S.tab === 'design') { drawPreview(); const dv = $('#derived'); if (dv) dv.innerHTML = derived(S.draft); }
  }
};
panel.onchange = e => {
  const t = e.target;
  if (t.id === 'mLoop') S.missions[S.sel].loop = t.checked;
  else if (t.id === 'mapSrc') send({ op: 'mapping', id: S.sel, source: t.value });
  else if (t.id === 'code') send({ op: 'script', id: S.sel, code: t.value });
  else if (t.id === 'exSel' && t.value !== '') { S.code[S.sel] = window.EXAMPLES[+t.value].code; $('#code').value = S.code[S.sel]; send({ op: 'script', id: S.sel, code: S.code[S.sel] }); t.value = ''; }
  else if (t.id === 'sAdd' && t.value) { S.draft.sensors.push(clone(SNEW[t.value])); S.dirty = true; renderPanel(); }
  else if (t.dataset.path === 'driver.type') {
    const def = { sim: { type: 'sim' }, rosbridge: { type: 'rosbridge', url: 'ws://agv.local:9090', namespace: '', topics: { cmd_vel: '/cmd_vel', odom: '/odom', scan: '/scan', battery: '/battery_state', pose: '', gps: '' } }, serial: { type: 'serial', port: '', baud: 115200 }, udp: { type: 'udp', host: '192.168.1.50', port: 4210, listenPort: 4211, oneWay: false } };
    S.draft.driver = def[t.value]; S.dirty = true; if (t.value === 'serial' && !S.portsLoaded) send({ op: 'listPorts' }); renderPanel();
  } else if (t.dataset.re) renderPanel();
};
panel.onclick = async e => {
  const b = e.target.closest('[data-act]'); if (!b) return;
  const [act, arg] = b.dataset.act.split(':'), id = S.sel, sp = selSpec(), M = S.missions[id];
  if (act === 'mode') send({ op: 'mode', id, mode: arg });
  else if (act === 'vestop') send({ op: 'estop', id, on: !selSt().estop });
  else if (act === 'goto') send({ op: 'goto', id, station: arg });
  else if (act === 'navcancel') send({ op: 'cancel', id });
  else if (act === 'madd') { const t = $('#mType').value, a = +$('#mArg').value || 0; if (!t) return; M.steps.push(t.startsWith('goto:') ? { type: 'goto', station: t.slice(5) } : t === 'wait' ? { type: 'wait', sec: a } : t === 'load' ? { type: 'load', kg: Math.min(a, sp.maxPayload) } : { type: 'unload' }); renderPanel(); }
  else if (act === 'mdel') { M.steps.splice(+arg, 1); renderPanel(); }
  else if (act === 'mclear') { M.steps = []; send({ op: 'cancel', id }); renderPanel(); }
  else if (act === 'mstart') { if (!M.steps.length) return hint('Önce göreve adım ekleyin', 2000); send({ op: 'mission', id, mission: M }); }
  else if (act === 'apply') { S.dirty = false; send({ op: 'updateVehicle', id, spec: { ...S.draft, script: S.code[id] != null ? S.code[id] : S.draft.script } }); hint('Uygulandı', 1200); }
  else if (act === 'sdel') { S.draft.sensors.splice(+arg, 1); S.dirty = true; renderPanel(); }
  else if (act === 'dup') { const c = clone(S.draft), v = selSt(); c.id = ''; c.name = sp.name + ' kopya'; c.start = { x: clamp(v.x + 1.5, 1, S.world.width - 1), y: v.y, theta: v.th }; send({ op: 'addVehicle', spec: c }); }
  else if (act === 'export') download((sp.name || 'arac').replace(/[^\w-]+/g, '_') + '.json', JSON.stringify({ ...S.draft, script: S.code[id] != null ? S.code[id] : S.draft.script }, null, 2));
  else if (act === 'import') { const o = await pickFile(); if (o && o.kinematics) { S.draft = { ...o, id, start: sp.start }; S.dirty = true; if (typeof o.script === 'string') S.code[id] = o.script; renderPanel(); hint('İçe aktarıldı — “Tasarımı uygula” ile kaydedin', 3000); } }
  else if (act === 'del') { if (await dialog('Aracı sil', `<div class="note"><b>${esc(sp.name)}</b> filodan kaldırılacak.</div>`, 'Sil')) send({ op: 'removeVehicle', id }); }
  else if (act === 'run') { const code = $('#code').value; S.code[id] = code; send({ op: 'script', id, code, run: true }); }
  else if (act === 'stop') send({ op: 'script', id, run: false });
  else if (act === 'cclear') { S.consoles[id] = []; $('#console').innerHTML = ''; }
  else if (act === 'maptoggle') { const on = !selSt().map; send({ op: 'mapping', id, on }); if (on && !S.layers.scan) { S.layers.scan = true; document.querySelector('[data-layer="scan"]').checked = true; } }
  else if (act === 'mapclear') send({ op: 'mappingClear' });
  else if (act === 'mappgm' || act === 'mappng') {
    const G = S.grid; if (!G || !G.known) return hint('Önce haritalamayı başlatıp aracı gezdir', 2500);
    const base = ((S.maps.find(m => m.id === S.activeMap) || { name: 'harita' }).name.replace(/[^\w-]+/g, '_') || 'harita') + '_lidar';
    if (act === 'mappgm') {
      download(base + '.pgm', gridPgm(), 'image/x-portable-graymap');
      setTimeout(() => download(base + '.yaml', `image: ${base}.pgm\nmode: trinary\nresolution: ${G.res}\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n`, 'text/yaml'), 400);
    } else {
      const c = document.createElement('canvas'); c.width = G.w; c.height = G.h; const g = c.getContext('2d'), img = g.createImageData(G.w, G.h);
      for (let y = 0; y < G.h; y++) for (let x = 0; x < G.w; x++) { const v = G.st[y * G.w + x], o = ((G.h - 1 - y) * G.w + x) * 4, col = v === 2 ? 0 : v === 1 ? 255 : 128; img.data[o] = img.data[o + 1] = img.data[o + 2] = col; img.data[o + 3] = 255; }
      g.putImageData(img, 0, 0); c.toBlob(b => download(base + '.png', b));
    }
  }
  else if (act === 'connect') send({ op: 'connect', id });
  else if (act === 'disconnect') send({ op: 'disconnect', id });
  else if (act === 'ports') send({ op: 'listPorts' });
};
panel.addEventListener('keydown', e => { // kod editorunde Tab = 2 bosluk
  if (e.target.id === 'code' && e.key === 'Tab') { e.preventDefault(); const t = e.target, s = t.selectionStart; t.value = t.value.slice(0, s) + '  ' + t.value.slice(t.selectionEnd); t.selectionStart = t.selectionEnd = s + 2; S.code[S.sel] = t.value; }
});

// ------------------------------------------------------------------ Manuel surus: joystick + klavye
function initJoy() {
  const c = $('#joy'); if (!c) return; const g = c.getContext('2d'), R = 62, cx = 70, cy = 70;
  const paint = () => {
    g.clearRect(0, 0, 140, 140); g.strokeStyle = '#28313d'; g.lineWidth = 1.5; g.fillStyle = '#0f1318'; g.beginPath(); g.arc(cx, cy, R, 0, 7); g.fill(); g.stroke();
    g.beginPath(); g.moveTo(cx, cy - R); g.lineTo(cx, cy + R); g.moveTo(cx - R, cy); g.lineTo(cx + R, cy); g.stroke();
    g.fillStyle = S.joy.on ? '#f5b301' : '#3d4a5b'; g.beginPath(); g.arc(cx + S.joy.x * (R - 16), cy - S.joy.y * (R - 16), 16, 0, 7); g.fill();
  };
  const set = e => { const r = c.getBoundingClientRect(); let x = (e.clientX - r.left - cx) / (R - 16), y = -(e.clientY - r.top - cy) / (R - 16); const l = Math.hypot(x, y); if (l > 1) { x /= l; y /= l; } S.joy.x = x; S.joy.y = y; paint(); };
  c.onpointerdown = e => { c.setPointerCapture(e.pointerId); S.joy.on = true; set(e); };
  c.onpointermove = e => { if (S.joy.on) set(e); };
  c.onpointerup = c.onpointercancel = () => { S.joy = { x: 0, y: 0, on: false }; paint(); };
  paint();
}
const typing = () => { const a = document.activeElement; return a && (a.tagName === 'TEXTAREA' || a.tagName === 'SELECT' || (a.tagName === 'INPUT' && a.type !== 'range' && a.type !== 'checkbox')); };
window.addEventListener('keydown', e => {
  if (e.ctrlKey && e.key.toLowerCase() === 's') { e.preventDefault(); send({ op: 'save' }); return; }
  if (!$('#modal').hidden || typing() || e.ctrlKey || e.metaKey) return;
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (['w', 'a', 's', 'd', 'q', 'e', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(k)) { e.preventDefault(); S.keys.add(k); if (k === ' ' && S.sel && !e.repeat) send({ op: 'cancel', id: S.sel }); }
  else if (k === 'p') $('#btnPlay').click(); else if (k === 'f') fit();
  else if (k === '1') setTool('select'); else if (k === '2') setTool('goto'); else if (k === '3') setTool('pose');
  else if (k === 'Escape') { drag = null; setTool('select'); }
});
window.addEventListener('keyup', e => { S.keys.delete(e.key.length === 1 ? e.key.toLowerCase() : e.key); });
window.addEventListener('blur', () => S.keys.clear());
let wasActive = false;
setInterval(() => {
  const sp = selSpec(), v = selSt(); if (!sp || !v) return;
  const K = S.keys, on = k => K.has(k) ? 1 : 0;
  let fwd = on('w') + on('ArrowUp') - on('s') - on('ArrowDown'), turn = on('a') + on('ArrowLeft') - on('d') - on('ArrowRight'), side = on('q') - on('e');
  if (S.joy.on) { fwd += S.joy.y; turn -= S.joy.x; }
  const active = fwd !== 0 || turn !== 0 || side !== 0;
  if (active && v.mode !== 'manual') { hint('Elle sürmek için Manuel moda geçin', 1500); return; }
  if (active || wasActive) { const k = S.speedPct / 100; send({ op: 'teleop', id: sp.id, v: clamp(fwd, -1, 1) * sp.maxSpeed * k, w: clamp(turn, -1, 1) * sp.maxAngular * Math.max(0.5, k), vy: clamp(side, -1, 1) * sp.maxSpeed * k }); }
  wasActive = active;
}, 100);

resize(); connect(); requestAnimationFrame(draw);
