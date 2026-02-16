import { useState, useEffect, useCallback, useRef } from 'react';
import {
  Usb, Unplug, Send, Terminal, Home, Moon, OctagonX, RotateCcw,
  Activity, ShieldCheck, ChevronRight, Trash2, Settings, Gauge
} from 'lucide-react';
import PHASES, { CONTROL_MODES, QUICK_COMMANDS } from './data/commands';
import TelemetryChart from './components/TelemetryChart';
import CommandPanel from './components/CommandPanel';
import JointGauges from './components/JointGauges';

const ICON_MAP = { Home, Moon, OctagonX, RotateCcw, Activity, ShieldCheck };
const MAX_LOG = 500;

function useSerial() {
  const [connected, setConnected] = useState(false);
  const [ports, setPorts] = useState([]);
  const [log, setLog] = useState([]);
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0, 0]);
  const [robotState, setRobotState] = useState('UNKNOWN');
  const [telemetry, setTelemetry] = useState([]);
  const cleanupRef = useRef([]);

  const api = typeof window !== 'undefined' && window.electronAPI?.serial;

  const refreshPorts = useCallback(async () => {
    if (!api) return;
    const list = await api.list();
    setPorts(list);
  }, [api]);

  const connect = useCallback(async (path, baud) => {
    if (!api) return;
    const res = await api.connect(path, parseInt(baud));
    if (res.ok) {
      setConnected(true);
      addLog('SYS', `Connected to ${path} @ ${baud}`);
    } else {
      addLog('ERR', `Connect failed: ${res.error}`);
    }
  }, [api]);

  const disconnect = useCallback(async () => {
    if (!api) return;
    await api.disconnect();
    setConnected(false);
    addLog('SYS', 'Disconnected');
  }, [api]);

  const send = useCallback(async (cmd) => {
    if (!api) {
      addLog('TX', cmd);
      addLog('RX', `[demo] OK — ${cmd}`);
      simulateResponse(cmd);
      return;
    }
    addLog('TX', cmd);
    const res = await api.send(cmd);
    if (!res.ok) addLog('ERR', res.error);
  }, [api]);

  function simulateResponse(cmd) {
    if (cmd === 'get_robot_state') {
      setRobotState('IDLE');
    }
    if (cmd.startsWith('move_j') || cmd === 'homing') {
      const parts = cmd.split(' ').slice(1).map(Number);
      if (parts.length >= 6) setJointAngles(parts.slice(0, 6));
      else setJointAngles([0, 0, 0, 0, 0, 0]);
    }
  }

  function addLog(type, msg) {
    const entry = { time: new Date().toLocaleTimeString('en-GB'), type, msg };
    setLog(prev => [entry, ...prev].slice(0, MAX_LOG));
  }

  useEffect(() => {
    if (!api) return;
    const unsub1 = api.onData((line) => {
      addLog('RX', line);
      // Parse telemetry: "TEL j1 j2 j3 j4 j5 j6 state"
      if (line.startsWith('TEL')) {
        const parts = line.split(/\s+/).slice(1).map(Number);
        if (parts.length >= 6) {
          setJointAngles(parts.slice(0, 6));
          const now = Date.now();
          setTelemetry(prev => [...prev.slice(-199), {
            t: now, j1: parts[0], j2: parts[1], j3: parts[2],
            j4: parts[3], j5: parts[4], j6: parts[5]
          }]);
        }
      }
      if (line.startsWith('STATE')) {
        setRobotState(line.split(/\s+/)[1] || 'UNKNOWN');
      }
    });
    const unsub2 = api.onDisconnected(() => {
      setConnected(false);
      addLog('SYS', 'Connection lost');
    });
    cleanupRef.current = [unsub1, unsub2];
    return () => cleanupRef.current.forEach(fn => fn?.());
  }, [api]);

  useEffect(() => { refreshPorts(); }, [refreshPorts]);

  return { connected, ports, log, jointAngles, robotState, telemetry, refreshPorts, connect, disconnect, send, setLog };
}

export default function App() {
  const serial = useSerial();
  const [activeTab, setActiveTab] = useState('dashboard');
  const [selectedPort, setSelectedPort] = useState('');
  const [baudRate, setBaudRate] = useState('115200');
  const [cmdInput, setCmdInput] = useState('');
  const [activePhase, setActivePhase] = useState('p1');

  const handleSendRaw = () => {
    if (!cmdInput.trim()) return;
    serial.send(cmdInput.trim());
    setCmdInput('');
  };

  const tabs = [
    { id: 'dashboard', label: 'Dashboard', icon: Gauge },
    { id: 'commands', label: 'Commands (43)', icon: Terminal },
    { id: 'telemetry', label: 'Telemetry', icon: Activity },
    { id: 'log', label: 'Console', icon: Terminal },
  ];

  return (
    <div className="h-screen flex flex-col bg-slate-950">
      {/* Title Bar */}
      <header className="flex items-center justify-between px-4 py-2 bg-slate-900 border-b border-slate-800 shrink-0">
        <div className="flex items-center gap-3">
          <span className="text-lg font-bold text-brand-400">🦾 DevClaw Studio</span>
          <span className="text-xs text-slate-500">v1.1.0</span>
        </div>
        <div className="flex items-center gap-2">
          {/* Connection */}
          <select value={selectedPort} onChange={e => setSelectedPort(e.target.value)}
            className="bg-slate-800 text-sm rounded px-2 py-1 border border-slate-700 text-slate-300 w-36">
            <option value="">Select Port</option>
            {serial.ports.map(p => <option key={p.path} value={p.path}>{p.path} {p.manufacturer && `(${p.manufacturer})`}</option>)}
          </select>
          <button onClick={serial.refreshPorts} className="text-xs text-slate-400 hover:text-white px-1">Refresh</button>
          <select value={baudRate} onChange={e => setBaudRate(e.target.value)}
            className="bg-slate-800 text-sm rounded px-2 py-1 border border-slate-700 text-slate-300 w-24">
            {[9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600].map(b =>
              <option key={b} value={b}>{b}</option>
            )}
          </select>
          {serial.connected ? (
            <button onClick={serial.disconnect}
              className="flex items-center gap-1 bg-red-600/20 text-red-400 px-3 py-1 rounded text-sm hover:bg-red-600/30">
              <Unplug size={14} /> Disconnect
            </button>
          ) : (
            <button onClick={() => serial.connect(selectedPort, baudRate)}
              disabled={!selectedPort}
              className="flex items-center gap-1 bg-green-600/20 text-green-400 px-3 py-1 rounded text-sm hover:bg-green-600/30 disabled:opacity-30">
              <Usb size={14} /> Connect
            </button>
          )}
          <div className={`w-2 h-2 rounded-full ${serial.connected ? 'bg-green-500 animate-pulse' : 'bg-slate-600'}`} />
        </div>
      </header>

      {/* Quick Actions Bar */}
      <div className="flex items-center gap-1 px-4 py-1.5 bg-slate-900/60 border-b border-slate-800/50 shrink-0">
        {QUICK_COMMANDS.map(qc => {
          const Icon = ICON_MAP[qc.icon] || Activity;
          return (
            <button key={qc.cmd} onClick={() => serial.send(qc.cmd)}
              className={`flex items-center gap-1 px-2.5 py-1 rounded text-xs font-medium transition
                ${qc.color === 'red' ? 'bg-red-600/20 text-red-400 hover:bg-red-600/40' :
                  qc.color === 'yellow' ? 'bg-yellow-600/20 text-yellow-400 hover:bg-yellow-600/40' :
                  'bg-slate-800 text-slate-300 hover:bg-slate-700'}`}>
              <Icon size={12} /> {qc.label}
            </button>
          );
        })}
        <div className="flex-1" />
        <span className="text-xs text-slate-500">State:</span>
        <span className={`text-xs font-mono font-bold px-2 py-0.5 rounded ${
          serial.robotState === 'IDLE' ? 'bg-green-900/40 text-green-400' :
          serial.robotState === 'ESTOP' ? 'bg-red-900/40 text-red-400' :
          'bg-slate-800 text-slate-400'
        }`}>{serial.robotState}</span>
      </div>

      {/* Tab Bar */}
      <div className="flex border-b border-slate-800 shrink-0">
        {tabs.map(t => (
          <button key={t.id} onClick={() => setActiveTab(t.id)}
            className={`flex items-center gap-1.5 px-4 py-2 text-sm transition
              ${activeTab === t.id ? 'tab-active font-medium' : 'text-slate-500 hover:text-slate-300'}`}>
            <t.icon size={14} /> {t.label}
          </button>
        ))}
      </div>

      {/* Main Content */}
      <div className="flex-1 overflow-hidden">
        {activeTab === 'dashboard' && (
          <div className="h-full grid grid-cols-3 gap-3 p-3 overflow-auto">
            {/* Joint Gauges */}
            <div className="col-span-2 bg-slate-900 rounded-lg border border-slate-800 p-4">
              <h3 className="text-sm font-semibold text-slate-400 mb-3">Joint Angles (deg)</h3>
              <JointGauges angles={serial.jointAngles} />
            </div>
            {/* Mode Selector */}
            <div className="bg-slate-900 rounded-lg border border-slate-800 p-4">
              <h3 className="text-sm font-semibold text-slate-400 mb-3">Control Mode</h3>
              <div className="space-y-1.5">
                {CONTROL_MODES.map(m => (
                  <button key={m.id} onClick={() => serial.send(`request_mode ${m.id}`)}
                    className="w-full flex items-center justify-between px-3 py-1.5 rounded text-xs bg-slate-800 hover:bg-slate-700 transition">
                    <span className="font-mono font-bold text-brand-400">{m.name}</span>
                    <span className="text-slate-500">{m.desc}</span>
                  </button>
                ))}
              </div>
            </div>
            {/* Mini Telemetry */}
            <div className="col-span-2 bg-slate-900 rounded-lg border border-slate-800 p-4">
              <h3 className="text-sm font-semibold text-slate-400 mb-2">Joint Telemetry</h3>
              <TelemetryChart data={serial.telemetry} height={180} />
            </div>
            {/* Quick Info */}
            <div className="bg-slate-900 rounded-lg border border-slate-800 p-4 space-y-3">
              <h3 className="text-sm font-semibold text-slate-400">Robot Info</h3>
              <div className="text-xs space-y-1">
                <div className="flex justify-between"><span className="text-slate-500">DOF</span><span>6</span></div>
                <div className="flex justify-between"><span className="text-slate-500">Core Loop</span><span>1 kHz</span></div>
                <div className="flex justify-between"><span className="text-slate-500">Motor Loop</span><span>20 kHz</span></div>
                <div className="flex justify-between"><span className="text-slate-500">Commands</span><span>43</span></div>
                <div className="flex justify-between"><span className="text-slate-500">Algorithm Modules</span><span>21</span></div>
                <div className="flex justify-between"><span className="text-slate-500">Bus</span><span>CAN 2.0</span></div>
              </div>
              <h3 className="text-sm font-semibold text-slate-400 pt-2">DH Parameters (m)</h3>
              <div className="text-xs space-y-1 font-mono">
                <div className="flex justify-between"><span className="text-slate-500">L_BS</span><span>0.109</span></div>
                <div className="flex justify-between"><span className="text-slate-500">D_BS</span><span>0.035</span></div>
                <div className="flex justify-between"><span className="text-slate-500">L_AM</span><span>0.146</span></div>
                <div className="flex justify-between"><span className="text-slate-500">L_FA</span><span>0.115</span></div>
                <div className="flex justify-between"><span className="text-slate-500">D_EW</span><span>0.052</span></div>
                <div className="flex justify-between"><span className="text-slate-500">L_WT</span><span>0.072</span></div>
              </div>
            </div>
          </div>
        )}

        {activeTab === 'commands' && (
          <div className="h-full flex">
            {/* Phase Sidebar */}
            <div className="w-40 bg-slate-900 border-r border-slate-800 p-2 shrink-0 overflow-auto">
              {PHASES.map(ph => (
                <button key={ph.id} onClick={() => setActivePhase(ph.id)}
                  className={`w-full text-left px-3 py-2 rounded text-sm mb-1 transition
                    ${activePhase === ph.id ? 'bg-slate-800 text-white font-medium' : 'text-slate-500 hover:text-slate-300 hover:bg-slate-800/50'}`}>
                  <span className="inline-block w-2 h-2 rounded-full mr-2" style={{ backgroundColor: ph.color }} />
                  {ph.label}
                  <span className="text-xs text-slate-600 ml-1">({ph.commands.length})</span>
                </button>
              ))}
            </div>
            {/* Command Cards */}
            <div className="flex-1 overflow-auto p-3">
              <CommandPanel
                phase={PHASES.find(p => p.id === activePhase)}
                onSend={serial.send}
              />
            </div>
          </div>
        )}

        {activeTab === 'telemetry' && (
          <div className="h-full p-4 overflow-auto">
            <div className="bg-slate-900 rounded-lg border border-slate-800 p-4">
              <div className="flex items-center justify-between mb-3">
                <h3 className="text-sm font-semibold text-slate-400">Real-Time Joint Telemetry</h3>
                <button onClick={() => serial.send('config_telemetry 35 10')}
                  className="text-xs bg-slate-800 px-2 py-1 rounded text-slate-400 hover:text-white">
                  Start Stream (10 Hz)
                </button>
              </div>
              <TelemetryChart data={serial.telemetry} height={400} />
            </div>
          </div>
        )}

        {activeTab === 'log' && (
          <div className="h-full flex flex-col">
            <div className="flex-1 overflow-auto p-3 font-mono text-xs">
              {serial.log.length === 0 && <p className="text-slate-600 text-center mt-10">No log entries yet. Send a command to start.</p>}
              {serial.log.map((entry, i) => (
                <div key={i} className="flex gap-2 py-0.5 hover:bg-slate-900/50">
                  <span className="text-slate-600 w-20 shrink-0">{entry.time}</span>
                  <span className={`w-8 shrink-0 font-bold ${
                    entry.type === 'TX' ? 'text-blue-400' :
                    entry.type === 'RX' ? 'text-green-400' :
                    entry.type === 'ERR' ? 'text-red-400' : 'text-yellow-400'
                  }`}>{entry.type}</span>
                  <span className="text-slate-300 break-all">{entry.msg}</span>
                </div>
              ))}
            </div>
            {/* Raw Command Input */}
            <div className="flex items-center gap-2 p-2 border-t border-slate-800 shrink-0">
              <span className="text-brand-400 font-mono text-sm">{'>'}</span>
              <input value={cmdInput} onChange={e => setCmdInput(e.target.value)}
                onKeyDown={e => e.key === 'Enter' && handleSendRaw()}
                placeholder="Type raw command..."
                className="flex-1 bg-slate-900 text-sm font-mono text-slate-200 px-3 py-1.5 rounded border border-slate-700 outline-none focus:border-brand-500" />
              <button onClick={handleSendRaw} className="bg-brand-600 text-white px-3 py-1.5 rounded text-sm hover:bg-brand-700">
                <Send size={14} />
              </button>
              <button onClick={() => serial.setLog([])} className="text-slate-500 hover:text-red-400 px-1">
                <Trash2 size={14} />
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
