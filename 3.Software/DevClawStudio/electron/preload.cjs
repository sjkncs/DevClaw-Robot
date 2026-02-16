const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  serial: {
    list: () => ipcRenderer.invoke('serial:list'),
    connect: (path, baudRate) => ipcRenderer.invoke('serial:connect', path, baudRate),
    disconnect: () => ipcRenderer.invoke('serial:disconnect'),
    send: (cmd) => ipcRenderer.invoke('serial:send', cmd),
    isOpen: () => ipcRenderer.invoke('serial:isOpen'),
    onData: (cb) => {
      const handler = (_e, data) => cb(data);
      ipcRenderer.on('serial:data', handler);
      return () => ipcRenderer.removeListener('serial:data', handler);
    },
    onDisconnected: (cb) => {
      const handler = () => cb();
      ipcRenderer.on('serial:disconnected', handler);
      return () => ipcRenderer.removeListener('serial:disconnected', handler);
    },
  },
});
