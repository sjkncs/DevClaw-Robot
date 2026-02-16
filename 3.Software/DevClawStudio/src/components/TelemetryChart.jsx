import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer, Legend } from 'recharts';

const JOINT_COLORS = ['#3b82f6', '#8b5cf6', '#ec4899', '#f59e0b', '#10b981', '#06b6d4'];
const JOINT_KEYS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'];

export default function TelemetryChart({ data, height = 300 }) {
  if (!data || data.length === 0) {
    return (
      <div className="flex items-center justify-center text-slate-600 text-sm" style={{ height }}>
        No telemetry data yet. Click "Start Stream" or send <code className="text-brand-400 mx-1">config_telemetry 35 10</code> to begin.
      </div>
    );
  }

  const formatted = data.map(d => ({
    ...d,
    time: new Date(d.t).toLocaleTimeString('en-GB', { hour12: false, minute: '2-digit', second: '2-digit' })
  }));

  return (
    <ResponsiveContainer width="100%" height={height}>
      <LineChart data={formatted} margin={{ top: 5, right: 10, left: 0, bottom: 5 }}>
        <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
        <XAxis dataKey="time" tick={{ fontSize: 10, fill: '#64748b' }} />
        <YAxis tick={{ fontSize: 10, fill: '#64748b' }} domain={['auto', 'auto']} />
        <Tooltip
          contentStyle={{ backgroundColor: '#0f172a', border: '1px solid #334155', borderRadius: '8px', fontSize: '12px' }}
          labelStyle={{ color: '#94a3b8' }}
        />
        <Legend wrapperStyle={{ fontSize: '11px' }} />
        {JOINT_KEYS.map((key, i) => (
          <Line key={key} type="monotone" dataKey={key} name={`J${i + 1}`}
            stroke={JOINT_COLORS[i]} dot={false} strokeWidth={1.5} isAnimationActive={false} />
        ))}
      </LineChart>
    </ResponsiveContainer>
  );
}
