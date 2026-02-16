const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
let SerialPort, ReadlineParser;

try {
  ({ SerialPort } = require('serialport'));
  ({ ReadlineParser } = require('@serialport/parser-readline'));
} catch (e) {
  console.warn('serialport not available — running in demo mode');
}

let mainWindow = null;
let port = null;
let parser = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1024,
    minHeight: 700,
    title: 'DevClaw Studio',
    backgroundColor: '#0f172a',
    webPreferences: {
      preload: path.join(__dirname, 'preload.cjs'),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  if (process.env.VITE_DEV_SERVER_URL) {
    mainWindow.loadURL(process.env.VITE_DEV_SERVER_URL);
  } else {
    mainWindow.loadFile(path.join(__dirname, '..', 'dist', 'index.html'));
  }
}

app.whenReady().then(createWindow);
app.on('window-all-closed', () => { if (process.platform !== 'darwin') app.quit(); });
app.on('activate', () => { if (BrowserWindow.getAllWindows().length === 0) createWindow(); });

// ── Serial Port IPC ──────────────────────────────────────────────────────

ipcMain.handle('serial:list', async () => {
  if (!SerialPort) return [];
  try {
    const ports = await SerialPort.list();
    return ports.map(p => ({ path: p.path, manufacturer: p.manufacturer || '', vendorId: p.vendorId || '' }));
  } catch { return []; }
});

ipcMain.handle('serial:connect', async (_e, portPath, baudRate) => {
  if (!SerialPort) return { ok: false, error: 'serialport module not available' };
  if (port && port.isOpen) {
    try { port.close(); } catch {}
  }
  return new Promise((resolve) => {
    port = new SerialPort({ path: portPath, baudRate: baudRate || 115200 }, (err) => {
      if (err) return resolve({ ok: false, error: err.message });
      parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));
      parser.on('data', (line) => {
        if (mainWindow && !mainWindow.isDestroyed()) {
          mainWindow.webContents.send('serial:data', line.trim());
        }
      });
      port.on('close', () => {
        if (mainWindow && !mainWindow.isDestroyed()) {
          mainWindow.webContents.send('serial:disconnected');
        }
      });
      resolve({ ok: true });
    });
  });
});

ipcMain.handle('serial:disconnect', async () => {
  if (port && port.isOpen) {
    return new Promise((resolve) => {
      port.close((err) => resolve({ ok: !err, error: err?.message }));
    });
  }
  return { ok: true };
});

ipcMain.handle('serial:send', async (_e, command) => {
  if (!port || !port.isOpen) return { ok: false, error: 'Not connected' };
  return new Promise((resolve) => {
    port.write(command + '\n', (err) => {
      resolve({ ok: !err, error: err?.message });
    });
  });
});

ipcMain.handle('serial:isOpen', () => {
  return port ? port.isOpen : false;
});
