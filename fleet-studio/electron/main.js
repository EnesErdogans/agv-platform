'use strict';
// Masaustu kabugu: yerel sunucuyu baslatir ve paneli bir pencerede acar
const { app, BrowserWindow, Menu, shell, dialog } = require('electron');
const path = require('path');
const { startServer } = require('../server/index');

let win = null, srv = null;
const res = name => app.isPackaged ? path.join(process.resourcesPath, name) : path.join(__dirname, '..', name);

async function createWindow() {
  if (!srv) srv = await startServer({ dataDir: app.getPath('userData'), lan: process.argv.includes('--lan') });
  win = new BrowserWindow({
    width: 1640, height: 980, minWidth: 1180, minHeight: 700, backgroundColor: '#0d1014', title: 'AGV-UGV Fleet Studio',
    webPreferences: { contextIsolation: true, nodeIntegration: false },
  });
  win.on('closed', () => { win = null; });
  await win.loadURL(srv.url);
}

function buildMenu() {
  const send = js => win && win.webContents.executeJavaScript(js).catch(() => { });
  Menu.setApplicationMenu(Menu.buildFromTemplate([
    {
      label: 'Dosya', submenu: [
        { label: 'Kaydet', accelerator: 'CmdOrCtrl+S', click: () => send("send({op:'save'})") },
        { label: 'Veri klasörünü aç', click: () => shell.openPath(app.getPath('userData')) },
        { type: 'separator' }, { role: 'quit', label: 'Çıkış' },
      ],
    },
    {
      label: 'Görünüm', submenu: [
        { label: 'Paneli tarayıcıda aç', click: () => shell.openExternal(srv.url) },
        { role: 'reload', label: 'Yenile' }, { role: 'togglefullscreen', label: 'Tam ekran' }, { role: 'toggleDevTools', label: 'Geliştirici araçları' },
        { type: 'separator' }, { role: 'zoomIn', label: 'Yakınlaştır' }, { role: 'zoomOut', label: 'Uzaklaştır' }, { role: 'resetZoom', label: 'Yakınlaştırmayı sıfırla' },
      ],
    },
    {
      label: 'Araç kodları', submenu: [
        { label: 'Firmware klasörü (Arduino / ESP32)', click: () => shell.openPath(res('firmware')) },
        { label: 'ROS 2 kurulum dosyaları', click: () => shell.openPath(res('ros2')) },
        { label: 'Örnek istemciler (Python)', click: () => shell.openPath(res('examples')) },
      ],
    },
    {
      label: 'Yardım', submenu: [
        { label: 'Kılavuz (README)', click: () => shell.openPath(res('README.md')) },
        { label: 'Hakkında', click: () => dialog.showMessageBox(win, { type: 'info', title: 'Hakkında', message: 'AGV-UGV Fleet Studio ' + app.getVersion(), detail: 'AGV ve UGV simülasyonu + gerçek araç filo kontrol paneli.\nPanel adresi: ' + srv.url + '\nVeri klasörü: ' + app.getPath('userData') }) },
      ],
    },
  ]));
}

if (!app.requestSingleInstanceLock()) app.quit();
else {
  app.on('second-instance', () => { if (win) { if (win.isMinimized()) win.restore(); win.focus(); } });
  app.whenReady().then(async () => {
    try { await createWindow(); buildMenu(); }
    catch (e) { dialog.showErrorBox('AGV-UGV Fleet Studio başlatılamadı', String(e && e.stack || e)); app.quit(); }
  });
  app.on('window-all-closed', () => { if (srv) srv.close(); app.quit(); });
}
