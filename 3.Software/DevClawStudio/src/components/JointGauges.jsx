const JOINT_COLORS = ['#3b82f6', '#8b5cf6', '#ec4899', '#f59e0b', '#10b981', '#06b6d4'];
const JOINT_LABELS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
const JOINT_RANGES = [
  [-180, 180], [-90, 90], [-120, 120], [-180, 180], [-90, 90], [-180, 180]
];

export default function JointGauges({ angles }) {
  return (
    <div className="grid grid-cols-6 gap-3">
      {angles.map((angle, i) => {
        const [min, max] = JOINT_RANGES[i];
        const range = max - min;
        const pct = ((angle - min) / range) * 100;
        const clampedPct = Math.max(0, Math.min(100, pct));

        return (
          <div key={i} className="flex flex-col items-center gap-2">
            <span className="text-xs font-bold" style={{ color: JOINT_COLORS[i] }}>{JOINT_LABELS[i]}</span>

            {/* Arc gauge */}
            <div className="relative w-20 h-12">
              <svg viewBox="0 0 80 48" className="w-full h-full">
                {/* Background arc */}
                <path d="M 8 44 A 36 36 0 0 1 72 44" fill="none" stroke="#1e293b" strokeWidth="5" strokeLinecap="round" />
                {/* Value arc */}
                <path d="M 8 44 A 36 36 0 0 1 72 44" fill="none" stroke={JOINT_COLORS[i]} strokeWidth="5" strokeLinecap="round"
                  strokeDasharray={`${clampedPct * 1.05} 200`} opacity="0.8" />
                {/* Needle */}
                {(() => {
                  const needleAngle = -180 + (clampedPct / 100) * 180;
                  const rad = needleAngle * (Math.PI / 180);
                  const cx = 40, cy = 44, r = 28;
                  const nx = cx + r * Math.cos(rad);
                  const ny = cy + r * Math.sin(rad);
                  return <line x1={cx} y1={cy} x2={nx} y2={ny} stroke={JOINT_COLORS[i]} strokeWidth="1.5" strokeLinecap="round" />;
                })()}
                <circle cx="40" cy="44" r="3" fill={JOINT_COLORS[i]} />
              </svg>
            </div>

            {/* Value */}
            <span className="text-lg font-mono font-bold text-white">
              {angle.toFixed(1)}°
            </span>
            <span className="text-xs text-slate-600">[{min}° .. {max}°]</span>
          </div>
        );
      })}
    </div>
  );
}
