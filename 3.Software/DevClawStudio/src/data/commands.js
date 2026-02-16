/**
 * All 43 DevClaw protocol commands organized by phase.
 * Each command: { id, name, args[], description, category }
 */
const PHASES = [
  {
    id: 'p1', label: 'P1 Motion', color: '#3b82f6',
    commands: [
      { id: 'move_j_scurve', name: 'move_j_scurve', description: 'S-Curve joint motion', args: [
        { name: 'j1', type: 'number', default: 0, unit: 'deg' },
        { name: 'j2', type: 'number', default: 0, unit: 'deg' },
        { name: 'j3', type: 'number', default: 0, unit: 'deg' },
        { name: 'j4', type: 'number', default: 0, unit: 'deg' },
        { name: 'j5', type: 'number', default: 0, unit: 'deg' },
        { name: 'j6', type: 'number', default: 0, unit: 'deg' },
        { name: 'maxVel', type: 'number', default: 1.0, unit: 'rad/s' },
        { name: 'maxAcc', type: 'number', default: 1.0, unit: 'rad/s2' },
      ]},
      { id: 'move_l_cart', name: 'move_l_cart', description: 'Cartesian linear motion', args: [
        { name: 'x', type: 'number', default: 200, unit: 'mm' },
        { name: 'y', type: 'number', default: 0, unit: 'mm' },
        { name: 'z', type: 'number', default: 300, unit: 'mm' },
        { name: 'rx', type: 'number', default: 0, unit: 'deg' },
        { name: 'ry', type: 'number', default: 0, unit: 'deg' },
        { name: 'rz', type: 'number', default: 0, unit: 'deg' },
      ]},
      { id: 'set_dynamics_enabled', name: 'set_dynamics_enabled', description: 'Enable/disable dynamics model', args: [
        { name: 'enable', type: 'bool', default: 1 },
      ]},
      { id: 'get_manipulability', name: 'get_manipulability', description: 'Query current manipulability index', args: [] },
    ]
  },
  {
    id: 'p2', label: 'P2 Robust', color: '#8b5cf6',
    commands: [
      { id: 'set_dob_enabled', name: 'set_dob_enabled', description: 'Enable/disable disturbance observer', args: [
        { name: 'enable', type: 'bool', default: 1 },
      ]},
      { id: 'set_dob_cutoff', name: 'set_dob_cutoff', description: 'Set DOB cutoff frequency', args: [
        { name: 'freq', type: 'number', default: 50, unit: 'Hz' },
      ]},
      { id: 'start_auto_tune', name: 'start_auto_tune', description: 'Start relay auto-tune for a joint', args: [
        { name: 'joint', type: 'number', default: 0 },
      ]},
      { id: 'start_identification', name: 'start_identification', description: 'Start Fourier excitation trajectory', args: [] },
      { id: 'run_identification', name: 'run_identification', description: 'Run WLS parameter identification', args: [] },
    ]
  },
  {
    id: 'p3', label: 'P3 Interact', color: '#ec4899',
    commands: [
      { id: 'set_impedance_mode', name: 'set_impedance_mode', description: 'Set impedance paradigm (0=admittance,1=impedance,2=variable)', args: [
        { name: 'mode', type: 'number', default: 0 },
      ]},
      { id: 'set_stiffness', name: 'set_stiffness', description: 'Set joint stiffness (all joints)', args: [
        { name: 'stiffness', type: 'number', default: 500, unit: 'Nm/rad' },
      ]},
      { id: 'set_collision_reaction', name: 'set_collision_reaction', description: 'Set collision reaction (0-4)', args: [
        { name: 'reaction', type: 'number', default: 1 },
      ]},
      { id: 'reset_collision', name: 'reset_collision', description: 'Reset collision flag', args: [] },
      { id: 'start_dmp_record', name: 'start_dmp_record', description: 'Start DMP recording', args: [] },
      { id: 'stop_dmp_record', name: 'stop_dmp_record', description: 'Stop DMP recording & encode weights', args: [] },
      { id: 'execute_dmp', name: 'execute_dmp', description: 'Execute DMP to goal position', args: [
        { name: 'j1', type: 'number', default: 0, unit: 'deg' },
        { name: 'j2', type: 'number', default: 30, unit: 'deg' },
        { name: 'j3', type: 'number', default: -60, unit: 'deg' },
        { name: 'j4', type: 'number', default: 0, unit: 'deg' },
        { name: 'j5', type: 'number', default: 0, unit: 'deg' },
        { name: 'j6', type: 'number', default: 0, unit: 'deg' },
      ]},
    ]
  },
  {
    id: 'p4', label: 'P4 Model', color: '#f59e0b',
    commands: [
      { id: 'set_ctc_enabled', name: 'set_ctc_enabled', description: 'Enable computed torque control', args: [
        { name: 'enable', type: 'bool', default: 1 },
      ]},
      { id: 'set_ctc_frequency', name: 'set_ctc_frequency', description: 'Set CTC update frequency', args: [
        { name: 'freq', type: 'number', default: 1000, unit: 'Hz' },
      ]},
      { id: 'set_friction_comp', name: 'set_friction_comp', description: 'Enable LuGre friction compensation', args: [
        { name: 'enable', type: 'bool', default: 1 },
      ]},
      { id: 'calibrate_force', name: 'calibrate_force', description: 'Zero force sensor baseline', args: [] },
      { id: 'get_ext_force', name: 'get_ext_force', description: 'Get external force/torque estimate', args: [] },
    ]
  },
  {
    id: 'p5', label: 'P5 Teach', color: '#10b981',
    commands: [
      { id: 'set_teach_mode', name: 'set_teach_mode', description: 'Enable/disable teach mode', args: [
        { name: 'enable', type: 'bool', default: 1 },
      ]},
      { id: 'save_waypoint', name: 'save_waypoint', description: 'Save current joint position as waypoint', args: [] },
      { id: 'set_teach_recording', name: 'set_teach_recording', description: 'Start/stop continuous recording', args: [
        { name: 'enable', type: 'bool', default: 1 },
      ]},
      { id: 'move_j_minjerk', name: 'move_j_minjerk', description: 'Minimum-jerk trajectory to target', args: [
        { name: 'j1', type: 'number', default: 0, unit: 'deg' },
        { name: 'j2', type: 'number', default: 0, unit: 'deg' },
        { name: 'j3', type: 'number', default: 0, unit: 'deg' },
        { name: 'j4', type: 'number', default: 0, unit: 'deg' },
        { name: 'j5', type: 'number', default: 0, unit: 'deg' },
        { name: 'j6', type: 'number', default: 0, unit: 'deg' },
      ]},
      { id: 'set_hybrid_axis', name: 'set_hybrid_axis', description: 'Set force/position selection axes', args: [
        { name: 'sx', type: 'bool', default: 0 },
        { name: 'sy', type: 'bool', default: 0 },
        { name: 'sz', type: 'bool', default: 1 },
      ]},
      { id: 'set_force_ref', name: 'set_force_ref', description: 'Set force reference for hybrid control', args: [
        { name: 'fx', type: 'number', default: 0, unit: 'N' },
        { name: 'fy', type: 'number', default: 0, unit: 'N' },
        { name: 'fz', type: 'number', default: 5, unit: 'N' },
      ]},
    ]
  },
  {
    id: 'p6', label: 'P6 Safety', color: '#ef4444',
    commands: [
      { id: 'safety_check', name: 'safety_check', description: 'Run safety self-check', args: [] },
      { id: 'reset_estop', name: 'reset_estop', description: 'Reset emergency stop latch', args: [] },
      { id: 'request_mode', name: 'request_mode', description: 'Request control mode change', args: [
        { name: 'mode', type: 'number', default: 0, unit: '0-7' },
      ]},
      { id: 'set_tcp_speed_limit', name: 'set_tcp_speed_limit', description: 'Set TCP speed limit', args: [
        { name: 'speed', type: 'number', default: 100, unit: 'mm/s' },
      ]},
      { id: 'config_telemetry', name: 'config_telemetry', description: 'Configure telemetry streaming', args: [
        { name: 'channels', type: 'number', default: 35 },
        { name: 'rate', type: 'number', default: 10, unit: 'Hz' },
      ]},
      { id: 'get_robot_state', name: 'get_robot_state', description: 'Query full robot state', args: [] },
    ]
  },
  {
    id: 'p7', label: 'P7 Calib', color: '#06b6d4',
    commands: [
      { id: 'analyze_workspace', name: 'analyze_workspace', description: 'Analyze workspace quality at current pose', args: [] },
      { id: 'record_grav_sample', name: 'record_grav_sample', description: 'Record gravity calibration sample', args: [] },
      { id: 'run_grav_calib', name: 'run_grav_calib', description: 'Run WLS gravity calibration', args: [] },
      { id: 'record_kin_sample', name: 'record_kin_sample', description: 'Record kinematic calibration sample', args: [] },
      { id: 'run_kin_calib', name: 'run_kin_calib', description: 'Run DH offset calibration', args: [] },
      { id: 'is_near_singular', name: 'is_near_singular', description: 'Check singularity proximity', args: [] },
    ]
  },
];

export const CONTROL_MODES = [
  { id: 0, name: 'POSITION', desc: 'Standard DCE position control' },
  { id: 1, name: 'CTC_TORQUE', desc: 'Computed torque control' },
  { id: 2, name: 'IMPEDANCE', desc: 'Admittance/impedance/variable stiffness' },
  { id: 3, name: 'TEACH', desc: 'Lead-through teaching' },
  { id: 4, name: 'DMP_EXEC', desc: 'DMP trajectory replay' },
  { id: 5, name: 'HYBRID', desc: 'Force/position hybrid' },
  { id: 6, name: 'MINJERK', desc: 'Minimum-jerk trajectory' },
  { id: 7, name: 'STOPPING', desc: 'Controlled deceleration' },
];

export const QUICK_COMMANDS = [
  { label: 'Home', cmd: 'homing', icon: 'Home', color: 'blue' },
  { label: 'Rest', cmd: 'resting', icon: 'Moon', color: 'indigo' },
  { label: 'E-Stop', cmd: 'estop', icon: 'OctagonX', color: 'red' },
  { label: 'Reset', cmd: 'reset_estop', icon: 'RotateCcw', color: 'yellow' },
  { label: 'Status', cmd: 'get_robot_state', icon: 'Activity', color: 'green' },
  { label: 'Safety', cmd: 'safety_check', icon: 'ShieldCheck', color: 'emerald' },
];

export default PHASES;
