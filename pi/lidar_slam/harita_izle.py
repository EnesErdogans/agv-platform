#!/usr/bin/env python3
"""AGV birlesik panel: canli LiDAR haritasi + kamera, tek sayfa.  http://<Pi-IP>:8080

ROS 2 /map, /scan ve TF (map->base_link, map->laser) okur. Kamera goruntusu ve tespitler tarayicida
dogrudan host'taki kamera servisinden (:8000) alinir.
  GET  /state      harita/konum/tarama durumu (JSON)
  GET  /map.png    harita goruntusu
  GET  /maps       kayitli haritalar
  POST /save       {"name": "..."} -> map_saver_cli -> /maps/<name>.pgm/.yaml
  POST /reset      Cartographer sureclerini durdurur (launch yeniden baslatir) -> bos harita
HTTP kismi ve durum rclpy olmadan import edilebilir (test icin).
"""
import collections
import json
import math
import os
import re
import shutil
import signal
import socketserver
import struct
import subprocess
import threading
import time
import zlib
from http import server

PORT = int(os.environ.get("AGV_PANEL_PORT", "8080"))
MAP_DIR = os.environ.get("AGV_MAP_DIR", "/maps")
CARTO_PROCS = ("cartographer_node", "cartographer_occupancy_grid_node")

lock = threading.Lock()
state = {
    "map_ver": 0,
    "map_png": None,
    "info": None,       # {"res","w","h","ox","oy"}
    "pose": None,       # {"x","y","yaw"}
    "laser": None,      # {"x","y","yaw"}
    "scan": None,       # {"a0","da","r":[...]}
    "reset_id": 0,
    "reset_t": 0.0,     # reset sonrasi kisa sure eski TF konumunu gosterme
    "tf_clear": False,  # ROS dugumune: TF tamponunu temizle
}
scan_stamps = collections.deque(maxlen=50)

# OccupancyGrid int8 -> gri ton (0 dolu/siyah, 254 bos/beyaz, 205 bilinmeyen)
_TABLE = bytearray(256)
for _v in range(256):
    _s = _v - 256 if _v > 127 else _v
    if _s < 0:
        _TABLE[_v] = 205
    else:
        _TABLE[_v] = int(round(254 * (1.0 - min(_s, 100) / 100.0)))
_TABLE = bytes(_TABLE)


def grid_to_png(width, height, raw_int8_bytes):
    """Satir 0 = en yuksek y (map_server .pgm ile ayni yon)."""
    gray = raw_int8_bytes.translate(_TABLE)
    rows = [gray[r * width:(r + 1) * width] for r in range(height - 1, -1, -1)]
    data = b"".join(b"\x00" + row for row in rows)

    def chunk(tag, payload):
        return (struct.pack(">I", len(payload)) + tag + payload
                + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(data, 6))
            + chunk(b"IEND", b""))


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ----------------------------------------------------------------- durum guncelleme (ROS dugumu + test)
def set_map(w, h, res, ox, oy, raw_int8_bytes):
    png = grid_to_png(w, h, raw_int8_bytes)
    with lock:
        state["map_png"] = png
        state["info"] = {"res": res, "w": w, "h": h, "ox": ox, "oy": oy}
        state["map_ver"] += 1


def set_scan(a0, da, ranges):
    with lock:
        state["scan"] = {"a0": a0, "da": da, "r": ranges}
        scan_stamps.append(time.time())


def set_pose(pose, laser):
    with lock:
        if time.time() - state["reset_t"] < 3.0:
            pose = laser = None
        state["pose"] = pose
        state["laser"] = laser


def scan_rate():
    """(Hz, son taramanin yasi sn). Veri yoksa (0, None)."""
    now = time.time()
    st = list(scan_stamps)
    if not st:
        return 0.0, None
    age = now - st[-1]
    recent = [t for t in st if now - t < 3.0]
    hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) >= 2 and recent[-1] > recent[0] else 0.0
    return round(hz, 1), round(age, 2)


# ----------------------------------------------------------------- islemler
_TR = str.maketrans("çğıöşüÇĞİÖŞÜ", "cgiosuCGIOSU")


def sanitize_name(name):
    name = str(name or "").strip().translate(_TR).replace(" ", "_")
    name = re.sub(r"[^A-Za-z0-9_-]", "", name)[:60]
    return name or time.strftime("harita_%Y%m%d_%H%M%S")


def save_map(name):
    name = sanitize_name(name)
    path = os.path.join(MAP_DIR, name)
    if os.path.exists(path + ".yaml"):
        return False, f"'{name}' adlı harita zaten var, başka bir ad gir."
    with lock:
        has_map = state["info"] is not None
    if not has_map:
        return False, "Kaydedilecek harita yok (henüz harita oluşmadı)."
    if shutil.which("ros2") is None:
        return False, "Kaydetme HATASI: ros2 / map_saver_cli bulunamadı (konteyner içinde mi çalışıyor?)."
    try:
        os.makedirs(MAP_DIR, exist_ok=True)
        res = subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path],
                             capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return False, "Kaydetme HATASI: map_saver_cli 60 sn içinde bitmedi."
    except Exception as e:  # noqa: BLE001
        return False, f"Kaydetme HATASI: {e}"
    if res.returncode == 0 and os.path.exists(path + ".yaml"):
        return True, f"Kaydedildi: ~/maps/{name}.pgm + .yaml"
    return False, "Kaydetme HATASI: " + ((res.stderr or res.stdout or "").strip()[-300:] or f"kod {res.returncode}")


def find_carto_pids():
    """/proc/<pid>/cmdline icinde cartographer_node / cartographer_occupancy_grid_node olan surecler."""
    pids = []
    me = os.getpid()
    for d in os.listdir("/proc"):
        if not d.isdigit() or int(d) == me:
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                argv = f.read().split(b"\0")
        except OSError:
            continue
        if not argv or not argv[0]:
            continue
        # argv[0] veya (python ile calistirilmissa) argv[1] programin kendisi olmali
        heads = [os.path.basename(a.decode("utf-8", "replace")) for a in argv[:2]]
        if any(h in CARTO_PROCS for h in heads):
            pids.append(int(d))
    return pids


def reset_map():
    pids = find_carto_pids()
    killed = []
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
            killed.append(pid)
        except OSError:
            pass
    with lock:
        state["map_png"] = None
        state["info"] = None
        state["pose"] = None
        state["laser"] = None
        state["map_ver"] += 1
        state["reset_id"] += 1
        state["reset_t"] = time.time()
        state["tf_clear"] = True
    if not killed:
        return False, "Cartographer süreci bulunamadı; görüntü temizlendi ama SLAM sıfırlanmadı."
    return True, f"Harita silindi, SLAM yeniden başlıyor ({len(killed)} süreç). Birkaç saniye bekle."


def list_maps():
    items = []
    try:
        names = os.listdir(MAP_DIR)
    except OSError:
        return items
    for fn in names:
        if not fn.endswith(".yaml"):
            continue
        p = os.path.join(MAP_DIR, fn)
        try:
            with open(p, "r", errors="replace") as f:
                if "image:" not in f.read(4096):
                    continue  # stations.yaml vb. harita degil
            mt = os.path.getmtime(p)
        except OSError:
            continue
        items.append({"name": fn[:-5], "t": time.strftime("%d.%m.%Y %H:%M", time.localtime(mt)), "ts": mt})
    items.sort(key=lambda x: -x["ts"])
    return items


PAGE = r"""<!doctype html><html lang="tr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AGV Panel</title>
<style>
:root{--bg:#121417;--pan:#1b1e23;--pan2:#23272e;--line:#323842;--tx:#e4e6ea;--mut:#8b929c;
 --ok:#2ecc71;--warn:#f5a623;--bad:#ff4d5e;--acc:#3fa9f5}
*{box-sizing:border-box}
html,body{margin:0;height:100%;background:var(--bg);color:var(--tx);font:14px system-ui,-apple-system,"Segoe UI",sans-serif}
body{display:flex;flex-direction:column;overflow:hidden}
header{display:flex;align-items:center;gap:8px;flex-wrap:wrap;padding:8px 10px;background:var(--pan);border-bottom:1px solid var(--line)}
.brand{font-weight:700;letter-spacing:.5px;margin-right:4px}
.chips{display:flex;gap:6px;flex-wrap:wrap;flex:1 1 auto}
.chip{display:inline-flex;align-items:center;gap:6px;background:var(--pan2);border:1px solid var(--line);border-radius:999px;
 padding:3px 10px;font-size:13px;white-space:nowrap}
.chip b{font-weight:600;color:var(--mut)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--mut);flex:none}
.dot.ok{background:var(--ok)}.dot.warn{background:var(--warn)}.dot.bad{background:var(--bad)}
.mono{font-family:ui-monospace,Consolas,monospace}
.btns{display:flex;gap:6px;flex-wrap:wrap;position:relative}
button{background:var(--pan2);color:var(--tx);border:1px solid var(--line);border-radius:6px;padding:6px 11px;font:inherit;cursor:pointer}
button:hover{background:#2d323a}
button.pri{background:#1f5f99;border-color:#2b78bd}button.pri:hover{background:#2470b3}
button.dng{background:#5c1f27;border-color:#8a2c38}button.dng:hover{background:#74252f}
main{flex:1 1 auto;display:flex;min-height:0}
#mapbox{position:relative;flex:0 0 60%;min-width:0;min-height:0;border-right:1px solid var(--line)}
#c{position:absolute;inset:0;width:100%;height:100%;cursor:grab;touch-action:none;display:block}
.mapctl{position:absolute;top:8px;left:8px;display:flex;gap:6px;flex-wrap:wrap;align-items:center}
.mapctl button,.mapctl label{background:#1b1e23e0;border:1px solid var(--line);border-radius:6px;padding:5px 9px;font-size:13px;color:var(--tx)}
.mapctl label{display:inline-flex;gap:5px;align-items:center;cursor:pointer}
#maphint{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);color:var(--mut);text-align:center;pointer-events:none}
#cambox{flex:1 1 40%;min-width:0;display:flex;flex-direction:column;background:var(--pan)}
.camwrap{position:relative;background:#000;aspect-ratio:4/3;width:100%;flex:none}
#cam{width:100%;height:100%;object-fit:contain;display:block}
#camoff{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;color:var(--mut);text-align:center;padding:10px}
.sect{padding:8px 10px 4px;font-weight:600;color:var(--mut);font-size:12px;text-transform:uppercase;letter-spacing:.6px;display:flex;justify-content:space-between}
#dets{list-style:none;margin:0;padding:0 10px 10px;overflow:auto;flex:1 1 auto;min-height:60px}
#dets li{display:flex;align-items:center;gap:8px;padding:5px 8px;margin-bottom:4px;border-radius:6px;background:var(--pan2);border:1px solid var(--line)}
#dets li.person{background:#4a1820;border-color:var(--bad);font-weight:600}
#dets .lb{flex:1 1 auto}
#dets .bar{width:70px;height:6px;background:#0005;border-radius:3px;overflow:hidden;flex:none}
#dets .bar i{display:block;height:100%;background:var(--ok)}
#dets li.person .bar i{background:var(--bad)}
#dets .sc{width:38px;text-align:right;flex:none}
#dets .empty{background:none;border:none;color:var(--mut)}
#maps{display:none;position:absolute;right:0;top:calc(100% + 6px);z-index:20;background:var(--pan);border:1px solid var(--line);
 border-radius:8px;min-width:260px;max-width:90vw;max-height:50vh;overflow:auto;box-shadow:0 8px 24px #0008}
#maps.open{display:block}
#maps div{padding:6px 12px;border-bottom:1px solid var(--line);display:flex;justify-content:space-between;gap:12px}
#maps div span{color:var(--mut);white-space:nowrap}
#toasts{position:fixed;right:12px;bottom:12px;display:flex;flex-direction:column;gap:8px;z-index:50;max-width:min(420px,calc(100vw - 24px))}
.toast{background:#1f3a24;border:1px solid #2e7d43;padding:9px 12px;border-radius:8px;box-shadow:0 6px 18px #0008;transition:opacity .4s}
.toast.err{background:#3d1a1f;border-color:#a33}
@media (max-width:820px){
 body{overflow:auto;height:auto;min-height:100%}
 main{flex-direction:column}
 #mapbox{flex:none;height:56vh;border-right:none;border-bottom:1px solid var(--line)}
 #cambox{flex:none}
 #dets{max-height:none}
 header{gap:6px}
 .chips{flex-basis:100%;order:2}
 .btns{order:3;flex-basis:100%}
 .btns button{flex:1 1 auto}
 #maps{left:0;right:0}
}
</style></head><body>
<header>
 <span class="brand">AGV</span>
 <div class="chips">
  <span class="chip"><i class="dot" id="d_lidar"></i><b>LiDAR</b><span id="c_lidar" class="mono">—</span></span>
  <span class="chip"><i class="dot" id="d_cam"></i><b>Kamera</b><span id="c_cam">—</span></span>
  <span class="chip"><i class="dot" id="d_map"></i><b>Harita</b><span id="c_map" class="mono">—</span></span>
  <span class="chip"><b>Konum</b><span id="c_pose" class="mono">—</span></span>
 </div>
 <div class="btns">
  <button class="pri" id="b_save">Haritayı kaydet</button>
  <button class="dng" id="b_reset">Haritayı sil</button>
  <button id="b_maps">Kayıtlı haritalar ▾</button>
  <div id="maps"></div>
 </div>
</header>
<main>
 <section id="mapbox">
  <canvas id="c"></canvas>
  <div id="maphint">harita bekleniyor…</div>
  <div class="mapctl">
   <label><input type="checkbox" id="follow" checked> Takip et</label>
   <button id="fit">Sığdır</button>
   <button id="zin" title="Yakınlaştır">+</button><button id="zout" title="Uzaklaştır">−</button>
   <button id="clear">İzi temizle</button>
  </div>
 </section>
 <section id="cambox">
  <div class="camwrap"><img id="cam" alt=""><div id="camoff">Kamera bağlanıyor…</div></div>
  <div class="sect"><span>Tespit edilen nesneler</span><span id="camfps" class="mono"></span></div>
  <ul id="dets"><li class="empty">—</li></ul>
 </section>
</main>
<div id="toasts"></div>
<script>
const $=id=>document.getElementById(id);
const CAM='http://'+location.hostname+':8000';
const TR={person:'İnsan',bicycle:'Bisiklet',car:'Araba',motorcycle:'Motosiklet',bus:'Otobüs',truck:'Kamyon',
 chair:'Sandalye',couch:'Kanepe',bottle:'Şişe',cup:'Bardak',laptop:'Dizüstü',tv:'Ekran','cell phone':'Telefon',
 backpack:'Sırt çantası',handbag:'El çantası',suitcase:'Bavul',dog:'Köpek',cat:'Kedi',book:'Kitap',
 'dining table':'Masa',keyboard:'Klavye',mouse:'Fare',clock:'Saat','potted plant':'Saksı',bench:'Bank'};
function toast(msg,err){const t=document.createElement('div');t.className='toast'+(err?' err':'');t.textContent=msg;
 $('toasts').appendChild(t);setTimeout(()=>{t.style.opacity=0;setTimeout(()=>t.remove(),450)},err?7000:4000);}
function chip(id,cls,txt){$('d_'+id).className='dot '+cls;$('c_'+id).textContent=txt;}

/* ---------------- harita ---------------- */
const box=$('mapbox'),cv=$('c'),ctx=cv.getContext('2d');
let S=null,img=null,ver=-1,resetId=null,scale=40,cam={x:0,y:0},trail=[],drag=null,pinch=null,lastOk=0,fitted=false;
const ptrs=new Map();
function resize(){const r=box.getBoundingClientRect();cv.width=Math.max(1,r.width*devicePixelRatio);cv.height=Math.max(1,r.height*devicePixelRatio);}
new ResizeObserver(resize).observe(box);resize();
const W2S=(x,y)=>[cv.width/2+(x-cam.x)*scale,cv.height/2-(y-cam.y)*scale];
function fit(){if(!S||!S.info)return;const i=S.info,mw=i.w*i.res,mh=i.h*i.res;
 scale=Math.max(2,Math.min(cv.width/mw,cv.height/mh)*0.9);cam={x:i.ox+mw/2,y:i.oy+mh/2};$('follow').checked=false;}
function zoomAt(f,mx,my){const wx=cam.x+(mx-cv.width/2)/scale,wy=cam.y-(my-cv.height/2)/scale;
 scale=Math.max(2,Math.min(800,scale*f));cam.x=wx-(mx-cv.width/2)/scale;cam.y=wy+(my-cv.height/2)/scale;}
$('fit').onclick=fit;$('clear').onclick=()=>{trail=[]};
$('zin').onclick=()=>zoomAt(1.4,cv.width/2,cv.height/2);$('zout').onclick=()=>zoomAt(1/1.4,cv.width/2,cv.height/2);
cv.addEventListener('wheel',e=>{e.preventDefault();zoomAt(e.deltaY<0?1.15:1/1.15,e.offsetX*devicePixelRatio,e.offsetY*devicePixelRatio);},{passive:false});
cv.addEventListener('pointerdown',e=>{cv.setPointerCapture(e.pointerId);ptrs.set(e.pointerId,{x:e.clientX,y:e.clientY});
 $('follow').checked=false;if(ptrs.size===1)drag={x:e.clientX,y:e.clientY};else{drag=null;pinch=null;}});
cv.addEventListener('pointermove',e=>{if(!ptrs.has(e.pointerId))return;ptrs.set(e.pointerId,{x:e.clientX,y:e.clientY});const d=devicePixelRatio;
 if(ptrs.size===2){const[a,b]=[...ptrs.values()];const dist=Math.hypot(a.x-b.x,a.y-b.y);const r=cv.getBoundingClientRect();
  if(pinch)zoomAt(dist/pinch,((a.x+b.x)/2-r.left)*d,((a.y+b.y)/2-r.top)*d);pinch=dist;return;}
 if(!drag)return;cam.x-=(e.clientX-drag.x)*d/scale;cam.y+=(e.clientY-drag.y)*d/scale;drag={x:e.clientX,y:e.clientY};});
function pend(e){ptrs.delete(e.pointerId);pinch=null;drag=null;if(ptrs.size===1){const[p]=ptrs.values();drag={x:p.x,y:p.y};}}
cv.addEventListener('pointerup',pend);cv.addEventListener('pointercancel',pend);

async function poll(){
 try{const r=await fetch('/state',{cache:'no-store'});S=await r.json();lastOk=Date.now();
  if(resetId!==null&&S.reset_id!==resetId){trail=[];img=null;fitted=false;}
  resetId=S.reset_id;
  if(S.map_ver!==ver){ver=S.map_ver;if(S.info){const im=new Image(),v=ver;
   im.onload=()=>{if(v!==ver)return;img=im;if(!fitted){fitted=true;if(!S.pose)fit();}};im.src='/map.png?v='+v;}else img=null;}
  if(S.pose){const p=S.pose,l=trail[trail.length-1];
   if(!l||Math.hypot(p.x-l[0],p.y-l[1])>0.05)trail.push([p.x,p.y]);if(trail.length>5000)trail.shift();
   if($('follow').checked){cam.x=p.x;cam.y=p.y;}}
 }catch(e){}
 setTimeout(poll,200);}
poll();

function draw(){
 ctx.setTransform(1,0,0,1,0,0);ctx.fillStyle='#121417';ctx.fillRect(0,0,cv.width,cv.height);
 if(S&&S.info&&img){const i=S.info;const[sx,sy]=W2S(i.ox,i.oy+i.h*i.res);
  ctx.imageSmoothingEnabled=false;ctx.drawImage(img,sx,sy,i.w*i.res*scale,i.h*i.res*scale);}
 if(scale>15){ctx.strokeStyle='#ffffff14';ctx.lineWidth=1;const x0=Math.floor(cam.x-cv.width/2/scale),
  x1=Math.ceil(cam.x+cv.width/2/scale),y0=Math.floor(cam.y-cv.height/2/scale),y1=Math.ceil(cam.y+cv.height/2/scale);
  ctx.beginPath();for(let x=x0;x<=x1;x++){const[a]=W2S(x,0);ctx.moveTo(a,0);ctx.lineTo(a,cv.height);}
  for(let y=y0;y<=y1;y++){const[,b]=W2S(0,y);ctx.moveTo(0,b);ctx.lineTo(cv.width,b);}ctx.stroke();}
 if(trail.length>1){ctx.strokeStyle='#3fa9f5';ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();
  trail.forEach((p,k)=>{const[a,b]=W2S(p[0],p[1]);k?ctx.lineTo(a,b):ctx.moveTo(a,b);});ctx.stroke();}
 const live=S&&S.scan_age!==null&&S.scan_age<2;
 if(live&&S.scan&&S.laser){const L=S.laser,sc=S.scan;ctx.fillStyle='#ff3b5c';const r=Math.max(1.5,2*devicePixelRatio);
  for(let k=0;k<sc.r.length;k++){const d=sc.r[k];if(!d)continue;const a=L.yaw+sc.a0+k*sc.da;
   const[px,py]=W2S(L.x+d*Math.cos(a),L.y+d*Math.sin(a));ctx.fillRect(px-r/2,py-r/2,r,r);}}
 if(S&&S.pose){const p=S.pose,[x,y]=W2S(p.x,p.y);ctx.save();ctx.translate(x,y);ctx.rotate(-p.yaw);
  ctx.fillStyle='#ffb300aa';ctx.strokeStyle='#ffb300';ctx.lineWidth=2;
  ctx.fillRect(-0.5*scale,-0.4*scale,1.0*scale,0.8*scale);ctx.strokeRect(-0.5*scale,-0.4*scale,1.0*scale,0.8*scale);
  ctx.fillStyle='#000';ctx.beginPath();ctx.moveTo(0.45*scale,0);ctx.lineTo(0.05*scale,-0.25*scale);
  ctx.lineTo(0.05*scale,0.25*scale);ctx.closePath();ctx.fill();ctx.restore();}
 // durum cipleri
 const conn=S&&Date.now()-lastOk<2000;
 if(!conn){chip('lidar','bad',S?'panel bağlantısı yok':'bağlanıyor…');chip('map','bad','—');$('c_pose').textContent='—';
  $('maphint').textContent=S?'Panel sunucusuna bağlantı yok':'bağlanıyor…';$('maphint').style.display='block';}
 else{
  if(live)chip('lidar',S.scan_hz>=3?'ok':'warn',S.scan_hz.toFixed(1)+' Hz');else chip('lidar','bad','veri yok');
  const i=S.info;if(i)chip('map','ok',(i.w*i.res).toFixed(1)+'×'+(i.h*i.res).toFixed(1)+' m');else chip('map','warn','bekleniyor');
  const p=S.pose;$('c_pose').textContent=p?`x ${p.x.toFixed(2)}  y ${p.y.toFixed(2)}  ${(((p.yaw*180/Math.PI)%360+360)%360).toFixed(0)}°`:'bekleniyor';
  $('maphint').style.display=(i&&img)?'none':'block';
  $('maphint').textContent=live?'harita oluşuyor… aracı yavaşça hareket ettir':'harita bekleniyor… (LiDAR verisi yok)';}
 requestAnimationFrame(draw);}
draw();

/* ---------------- kamera ---------------- */
const camImg=$('cam');let camOnline=false,camRetry=null,detOk=0;
function camLoad(){clearTimeout(camRetry);camImg.src=CAM+'/stream.mjpg?t='+Date.now();}
camImg.onload=()=>{camImg.style.visibility='visible';$('camoff').style.display='none';};
camImg.onerror=()=>{camImg.style.visibility='hidden';$('camoff').style.display='flex';$('camoff').textContent='Kamera yok — 3 sn sonra tekrar deneniyor…';
 clearTimeout(camRetry);camRetry=setTimeout(camLoad,3000);};
camLoad();
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
async function pollDets(){
 try{const ctl=new AbortController();const to=setTimeout(()=>ctl.abort(),1500);
  const r=await fetch(CAM+'/detections.json',{cache:'no-store',signal:ctl.signal});clearTimeout(to);
  const d=await r.json();detOk=Date.now();delete $('dets').dataset.off;
  if(!camOnline){camOnline=true;camLoad();}
  const L=(d.detections||[]).slice().sort((a,b)=>(b.label==='person')-(a.label==='person')||b.score-a.score);
  $('dets').innerHTML=L.length?L.map(x=>`<li class="${x.label==='person'?'person':''}"><span class="lb">${esc(TR[x.label]||x.label)}</span>`+
   `<span class="bar"><i style="width:${Math.round(x.score*100)}%"></i></span><span class="sc mono">${Math.round(x.score*100)}%</span></li>`).join('')
   :`<li class="empty">${d.ai===false?'AI kapalı (sadece görüntü)':'Nesne yok'}</li>`;
  $('camfps').textContent=(d.fps||0).toFixed(0)+' fps';
 }catch(e){}
 if(Date.now()-detOk>2000){if(camOnline||!$('dets').dataset.off){camOnline=false;$('dets').dataset.off='1';$('dets').innerHTML='<li class="empty">Kamera servisi kapalı</li>';$('camfps').textContent='';
   $('camoff').style.display='flex';$('camoff').textContent='Kamera çevrimdışı (port 8000) — bekleniyor…';}
  chip('cam','bad','çevrimdışı');}
 else{const n=document.querySelectorAll('#dets li.person').length;chip('cam','ok',n?`çevrimiçi · ${n} insan`:'çevrimiçi');}
 setTimeout(pollDets,250);}
pollDets();

/* ---------------- butonlar ---------------- */
function defName(){const d=new Date(),p=n=>String(n).padStart(2,'0');
 return `harita_${d.getFullYear()}${p(d.getMonth()+1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;}
async function post(url,body){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});return r.json();}
$('b_save').onclick=async()=>{let n=prompt('Harita adı (harf, rakam, _ ve - kullan):',defName());if(n===null)return;
 const T={'ç':'c','ğ':'g','ı':'i','ö':'o','ş':'s','ü':'u','Ç':'C','Ğ':'G','İ':'I','Ö':'O','Ş':'S','Ü':'U'};
 const clean=n.trim().replace(/[çğıöşüÇĞİÖŞÜ]/g,c=>T[c]).replace(/ /g,'_').replace(/[^A-Za-z0-9_-]/g,'').slice(0,60);
 if(clean!==n.trim())toast(`Ad düzeltildi: ${clean||'(otomatik)'}`);
 toast('Kaydediliyor…');try{const r=await post('/save',{name:clean});toast(r.msg,!r.ok);if(r.ok)loadMaps();}catch(e){toast('Kaydetme isteği gönderilemedi',true);}};
$('b_reset').onclick=async()=>{if(!confirm('Mevcut harita silinip sıfırdan başlanacak. Emin misin?\n(Kaydetmediysen önce "Haritayı kaydet".)'))return;
 try{const r=await post('/reset');trail=[];img=null;fitted=false;toast(r.msg,!r.ok);}catch(e){toast('Sıfırlama isteği gönderilemedi',true);}};
async function loadMaps(){try{const r=await fetch('/maps',{cache:'no-store'});const L=await r.json();
 $('maps').innerHTML=L.length?L.map(m=>`<div><b>${esc(m.name)}</b><span>${esc(m.t)}</span></div>`).join(''):'<div><span>Kayıtlı harita yok</span></div>';}
 catch(e){$('maps').innerHTML='<div><span>Liste alınamadı</span></div>';}}
$('b_maps').onclick=e=>{e.stopPropagation();const m=$('maps');m.classList.toggle('open');if(m.classList.contains('open'))loadMaps();};
document.addEventListener('click',e=>{if(!$('maps').contains(e.target))$('maps').classList.remove('open');});
</script></body></html>"""


class Handler(server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, "application/json; charset=utf-8", json.dumps(obj, ensure_ascii=False).encode("utf-8"))

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        elif path == "/state":
            hz, age = scan_rate()
            with lock:
                body = {k: state[k] for k in ("map_ver", "info", "pose", "laser", "scan", "reset_id")}
            body["scan_hz"], body["scan_age"] = hz, age
            self._json(body)
        elif path == "/map.png":
            with lock:
                png = state["map_png"]
            if png is None:
                self._send(404, "text/plain", b"harita yok")
            else:
                self._send(200, "image/png", png)
        elif path == "/maps":
            self._json(list_maps())
        else:
            self._send(404, "text/plain", b"yok")

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(n) or b"{}") if n else {}
        except (ValueError, json.JSONDecodeError):
            return {}

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/save":
            ok, msg = save_map(self._body().get("name"))
            self._json({"ok": ok, "msg": msg})
        elif path == "/reset":
            ok, msg = reset_map()
            self._json({"ok": ok, "msg": msg})
        else:
            self._send(404, "text/plain", b"yok")


class Srv(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def start_http(port=PORT):
    srv = Srv(("", port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener

    class Viewer(Node):
        def __init__(self):
            super().__init__("harita_izle")
            self.tf_buf = Buffer()
            self.tf_lis = TransformListener(self.tf_buf, self)
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
            self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
            self.laser_frame = "laser"
            self.create_timer(0.1, self.on_timer)

        def on_map(self, msg):
            w, h = msg.info.width, msg.info.height
            if w == 0 or h == 0:
                return
            with lock:
                if time.time() - state["reset_t"] < 2.0:
                    return  # sifirlamadan hemen once gelen eski harita
            data = msg.data
            raw = data.tobytes() if hasattr(data, "tobytes") else bytes((v & 0xFF) for v in data)
            o = msg.info.origin.position
            set_map(w, h, msg.info.resolution, o.x, o.y, raw)

        def on_scan(self, msg):
            self.laser_frame = msg.header.frame_id or "laser"
            step = max(1, len(msg.ranges) // 720)
            rs = [round(r, 3) if (math.isfinite(r) and msg.range_min <= r <= msg.range_max) else 0
                  for r in msg.ranges[::step]]
            set_scan(msg.angle_min, msg.angle_increment * step, rs)

        def lookup(self, child):
            try:
                t = self.tf_buf.lookup_transform("map", child, Time())
            except Exception:  # noqa: BLE001
                return None
            tr = t.transform
            return {"x": tr.translation.x, "y": tr.translation.y, "yaw": yaw_from_q(tr.rotation)}

        def on_timer(self):
            with lock:
                clear = state["tf_clear"]
                state["tf_clear"] = False
            if clear:
                try:
                    self.tf_buf.clear()
                except Exception:  # noqa: BLE001
                    pass
            set_pose(self.lookup("base_link"), self.lookup(self.laser_frame))

    start_http()
    rclpy.init()
    node = Viewer()
    node.get_logger().info(f"AGV panel hazir: http://<Pi-IP>:{PORT}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
