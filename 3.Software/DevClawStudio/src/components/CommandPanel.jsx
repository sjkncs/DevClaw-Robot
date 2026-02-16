import { useState } from 'react';
import { Send, ChevronDown, ChevronUp } from 'lucide-react';

export default function CommandPanel({ phase, onSend }) {
  if (!phase) return null;

  return (
    <div className="space-y-3">
      <div className="flex items-center gap-2 mb-2">
        <span className="w-3 h-3 rounded-full" style={{ backgroundColor: phase.color }} />
        <h2 className="text-lg font-bold text-white">{phase.label}</h2>
        <span className="text-sm text-slate-500">{phase.commands.length} commands</span>
      </div>
      {phase.commands.map(cmd => (
        <CommandCard key={cmd.id} cmd={cmd} color={phase.color} onSend={onSend} />
      ))}
    </div>
  );
}

function CommandCard({ cmd, color, onSend }) {
  const [expanded, setExpanded] = useState(false);
  const [args, setArgs] = useState(() =>
    Object.fromEntries(cmd.args.map(a => [a.name, a.default]))
  );

  const buildCommand = () => {
    const argValues = cmd.args.map(a => args[a.name]);
    return argValues.length > 0 ? `${cmd.name} ${argValues.join(' ')}` : cmd.name;
  };

  const handleSend = () => {
    onSend(buildCommand());
  };

  const updateArg = (name, val) => {
    setArgs(prev => ({ ...prev, [name]: val }));
  };

  return (
    <div className="bg-slate-900 rounded-lg border border-slate-800 overflow-hidden hover:border-slate-700 transition">
      <div className="flex items-center gap-3 px-4 py-2.5 cursor-pointer" onClick={() => setExpanded(!expanded)}>
        <span className="w-1.5 h-8 rounded-full" style={{ backgroundColor: color }} />
        <div className="flex-1 min-w-0">
          <div className="flex items-center gap-2">
            <code className="text-sm font-bold text-white">{cmd.name}</code>
            {cmd.args.length > 0 && (
              <span className="text-xs text-slate-600">{cmd.args.length} args</span>
            )}
          </div>
          <p className="text-xs text-slate-500 truncate">{cmd.description}</p>
        </div>
        <button onClick={(e) => { e.stopPropagation(); handleSend(); }}
          className="flex items-center gap-1 bg-brand-600/20 text-brand-400 px-3 py-1.5 rounded text-xs font-medium hover:bg-brand-600/40 transition shrink-0">
          <Send size={12} /> Send
        </button>
        {cmd.args.length > 0 && (
          expanded ? <ChevronUp size={14} className="text-slate-500" /> : <ChevronDown size={14} className="text-slate-500" />
        )}
      </div>

      {expanded && cmd.args.length > 0 && (
        <div className="px-4 pb-3 border-t border-slate-800/50">
          <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-2 mt-2">
            {cmd.args.map(a => (
              <div key={a.name} className="flex flex-col gap-0.5">
                <label className="text-xs text-slate-500">
                  {a.name} {a.unit && <span className="text-slate-600">({a.unit})</span>}
                </label>
                {a.type === 'bool' ? (
                  <select value={args[a.name]} onChange={e => updateArg(a.name, parseInt(e.target.value))}
                    className="bg-slate-800 text-sm rounded px-2 py-1 border border-slate-700 text-slate-300">
                    <option value={0}>Off (0)</option>
                    <option value={1}>On (1)</option>
                  </select>
                ) : (
                  <input type="number" value={args[a.name]}
                    onChange={e => updateArg(a.name, parseFloat(e.target.value) || 0)}
                    className="bg-slate-800 text-sm font-mono rounded px-2 py-1 border border-slate-700 text-slate-300 w-full outline-none focus:border-brand-500" />
                )}
              </div>
            ))}
          </div>
          <div className="mt-2 flex items-center gap-2">
            <code className="text-xs text-slate-500 bg-slate-800 px-2 py-1 rounded flex-1 font-mono truncate">
              {buildCommand()}
            </code>
          </div>
        </div>
      )}
    </div>
  );
}
