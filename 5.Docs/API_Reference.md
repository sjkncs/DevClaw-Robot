# DevClaw Robot — API Reference

> Complete reference for all 43 protocol commands across 7 optimization phases.
> Commands are sent via **USB**, **UART**, **CAN**, or **Feishu Bot** (`/claw` prefix).

---

## Command Format

```
command_name [arg1] [arg2] ...
```

- Text-based ASCII protocol over USB/UART (115200 baud, 8N1)
- Each command terminated by `\n`
- Response: single line (value, `OK`, or `ERROR: description`)
- Built on [Fibre](https://github.com/samuelsadok/fibre) RPC framework

---

## Phase 1 — Core Motion Planning (4 commands)

### `move_j_scurve`

Move all joints using 7-segment S-Curve velocity profile with multi-axis synchronization.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `j1`–`j6` | float | deg | Target joint angles |
| `speed` | float | r/s | Max joint speed (optional) |
| `accel` | float | r/s² | Max acceleration (optional) |

```
move_j_scurve 0 30 -60 0 0 0 1.0 1.0
```

### `move_l_cart`

Linear Cartesian motion with SLERP orientation interpolation.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `x` `y` `z` | float | mm | Target TCP position |
| `a` `b` `c` | float | deg | Target TCP orientation (Euler ZYX) |

```
move_l_cart 200 0 300 0 90 0
```

### `set_dynamics_enabled`

Enable/disable RNEA-based dynamics computation (gravity, Coriolis, mass matrix).

| Param | Type | Description |
|-------|------|-------------|
| `enabled` | int | 1 = enable, 0 = disable |

```
set_dynamics_enabled 1
```

### `get_manipulability`

Query Yoshikawa manipulability index at current configuration.

| Return | Type | Description |
|--------|------|-------------|
| value | float | w = sqrt(det(J·J^T)), 0 at singularity |

```
get_manipulability
# → 0.0342
```

---

## Phase 2 — Robust Control & Identification (5 commands)

### `set_dob_enabled`

Enable/disable Disturbance Observer for all joints.

| Param | Type | Description |
|-------|------|-------------|
| `enabled` | int | 1 = enable, 0 = disable |

### `set_dob_cutoff`

Set DOB low-pass filter cutoff frequency.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `freq` | float | Hz | Cutoff frequency (default 50 Hz) |

### `start_auto_tune`

Start relay-feedback auto-tuning for PID gains.

| Param | Type | Description |
|-------|------|-------------|
| `joint` | int | Joint index (1–6) |

### `start_identification`

Start Fourier excitation trajectory for dynamic parameter identification.

| Param | Type | Description |
|-------|------|-------------|
| `joint` | int | Joint index (1–6), 0 = all |

### `run_identification`

Execute WLS regression on collected identification data.

| Return | Type | Description |
|--------|------|-------------|
| value | float | Condition number of regression |

---

## Phase 3 — Safe Interaction & Learning (7 commands)

### `set_impedance_mode`

Set impedance controller operating mode.

| Param | Type | Description |
|-------|------|-------------|
| `mode` | int | 0 = admittance, 1 = impedance, 2 = variable stiffness |

### `set_stiffness`

Set Cartesian stiffness for impedance control.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `kx` | float | N/m | Translational stiffness (all axes, or per-axis if 6 values) |

### `set_collision_reaction`

Set collision detection reaction strategy.

| Param | Type | Description |
|-------|------|-------------|
| `reaction` | int | 0=ignore, 1=warning, 2=stop, 3=retract, 4=float |

### `reset_collision`

Clear collision detection flag and resume normal operation.

### `start_dmp_record`

Begin recording joint trajectory for DMP weight learning.

### `stop_dmp_record`

Stop DMP recording, compute LWR weights.

### `execute_dmp`

Replay learned DMP trajectory to a new goal.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `j1`–`j6` | float | deg | New goal joint angles |

```
execute_dmp 0 45 -30 0 0 0
```

---

## Phase 4 — Model-Based Control & Force Sensing (5 commands)

### `set_ctc_enabled`

Enable/disable Computed Torque Control with feedback linearization.

| Param | Type | Description |
|-------|------|-------------|
| `enabled` | int | 1 = enable, 0 = disable |

### `set_ctc_frequency`

Set CTC computation frequency.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `freq` | float | Hz | CTC update rate (default 1000) |

### `set_friction_comp`

Enable/disable LuGre dynamic friction compensation.

| Param | Type | Description |
|-------|------|-------------|
| `enabled` | int | 1 = enable, 0 = disable |

### `calibrate_force`

Calibrate sensorless force estimator bias at current static pose.

### `get_ext_force`

Query estimated external Cartesian force/torque.

| Return | Type | Unit | Description |
|--------|------|------|-------------|
| `fx fy fz tx ty tz` | float×6 | N, Nm | External wrench at TCP |

```
get_ext_force
# → 0.12 -0.34 2.15 0.01 -0.02 0.00
```

---

## Phase 5 — Teaching, Assembly & Trajectory (6 commands)

### `set_teach_mode`

Enable/disable lead-through teaching with gravity + friction compensation.

| Param | Type | Description |
|-------|------|-------------|
| `enabled` | int | 1 = enable (zero-G mode), 0 = disable (hold position) |

### `save_waypoint`

Save current joint configuration as a waypoint in the teach buffer.

| Return | Type | Description |
|--------|------|-------------|
| value | bool | true if saved, false if buffer full |

### `set_teach_recording`

Start/stop continuous trajectory recording for DMP encoding.

| Param | Type | Description |
|-------|------|-------------|
| `recording` | int | 1 = start, 0 = stop & encode |

### `move_j_minjerk`

Execute minimum-jerk (5th-order polynomial) joint trajectory.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `j1`–`j6` | float | deg | Target joint angles |
| `duration` | float | s | Motion duration |

```
move_j_minjerk 0 30 -60 0 0 0 2.0
```

### `set_hybrid_axis`

Configure an axis for force or position control in hybrid mode.

| Param | Type | Description |
|-------|------|-------------|
| `axis` | int | Axis index (0=x, 1=y, 2=z, 3=rx, 4=ry, 5=rz) |
| `force_mode` | int | 1 = force control, 0 = position control |

### `set_force_ref`

Set Cartesian force reference for hybrid force/position control.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `fx` `fy` `fz` | float | N | Desired contact force |

```
set_force_ref 0 0 -5.0
```

---

## Phase 6 — Safety, State Machine & Telemetry (6 commands)

### `safety_check`

Run comprehensive safety check (position, velocity, current, thermal, watchdog).

| Return | Type | Description |
|--------|------|-------------|
| value | uint32 | Safety level: 0=OK, 1=WARNING, 2=SPEED_REDUCE, 3=STOP, 4=ESTOP |

### `reset_estop`

Reset emergency stop latch and transition to IDLE.

| Return | Type | Description |
|--------|------|-------------|
| value | bool | true if reset successful |

### `request_mode`

Request transition to a specific control mode via the state machine.

| Param | Type | Description |
|-------|------|-------------|
| `mode` | uint32 | 0=IDLE, 1=POSITION, 2=CTC, 3=IMPEDANCE, 4=TEACH, 5=DMP, 6=HYBRID, 7=MINJERK |

### `set_tcp_speed_limit`

Set maximum TCP Cartesian speed for safety monitoring.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `speed` | float | mm/s | TCP speed limit (default 250) |

### `config_telemetry`

Configure real-time telemetry streaming channels and rate.

| Param | Type | Description |
|-------|------|-------------|
| `channels` | uint32 | Bitmask: bit0=pos, bit1=vel, bit2=torque, bit3=force, bit4=temp, bit5=safety, ... |
| `rate` | float | Hz | Telemetry push rate |

```
config_telemetry 35 100
```

### `get_robot_state`

Query current state machine state.

| Return | Type | Description |
|--------|------|-------------|
| value | uint32 | State enum (0–11, see State Machine section) |

---

## Phase 7 — Calibration & Workspace Analysis (6 commands)

### `analyze_workspace`

Run full workspace analysis at current configuration.

| Return | Type | Description |
|--------|------|-------------|
| value | float | Yoshikawa manipulability index |

### `record_grav_sample`

Record a gravity calibration sample (current joints + motor currents).

| Return | Type | Description |
|--------|------|-------------|
| value | bool | true if recorded, false if buffer full (max 50) |

**Procedure**: Move robot to 10+ diverse static poses, call this at each pose.

### `run_grav_calib`

Execute WLS gravity calibration on all recorded samples.

| Return | Type | Unit | Description |
|--------|------|------|-------------|
| value | float | Nm | Residual RMS error, -1 if failed |

### `record_kin_sample`

Record a kinematic calibration sample with externally measured TCP position.

| Param | Type | Unit | Description |
|-------|------|------|-------------|
| `x` `y` `z` | float | mm | Measured TCP position (from laser tracker / touch probe) |

### `run_kin_calib`

Execute joint offset calibration using regularized least-squares.

| Return | Type | Unit | Description |
|--------|------|------|-------------|
| value | float | mm | Residual RMS position error, -1 if failed |

### `is_near_singular`

Check if current configuration is near a singularity.

| Return | Type | Description |
|--------|------|-------------|
| value | bool | true if condition number > threshold (default 50) |

---

## State Machine States

| ID | State | Description |
|----|-------|-------------|
| 0 | `IDLE` | No active control, motors enabled |
| 1 | `POSITION` | Standard DCE position control |
| 2 | `CTC_TORQUE` | Computed torque with feedback linearization |
| 3 | `IMPEDANCE` | Impedance / admittance / variable stiffness |
| 4 | `TEACH` | Lead-through teaching (gravity comp) |
| 5 | `DMP_EXEC` | DMP trajectory replay |
| 6 | `HYBRID` | Force/position hybrid control |
| 7 | `MINJERK` | Minimum-jerk trajectory execution |
| 8 | `STOPPING` | Controlled deceleration (2s timeout) |
| 9 | `ESTOP` | Emergency stop (latched, requires manual reset) |
| 10 | `FAULT` | Hardware or software fault |
| 11 | `INITIALIZING` | Power-on initialization |

---

## Safety Levels

| Level | Name | Action |
|-------|------|--------|
| 0 | `SAFETY_OK` | Normal operation |
| 1 | `SAFETY_WARNING` | Log warning, continue |
| 2 | `SAFETY_SPEED_REDUCE` | Reduce speed scale (0.5×) |
| 3 | `SAFETY_STOP` | Controlled stop, decelerate to zero |
| 4 | `SAFETY_ESTOP` | Immediate motor disable, latch |

---

## Telemetry Channel Bitmask

| Bit | Channel | Data |
|-----|---------|------|
| 0 | `CH_JOINT_POS` | 6× joint position (deg) |
| 1 | `CH_JOINT_VEL` | 6× joint velocity (deg/s) |
| 2 | `CH_JOINT_TORQUE` | 6× joint torque (Nm) |
| 3 | `CH_MOTOR_CURRENT` | 6× motor current (A) |
| 4 | `CH_MOTOR_TEMP` | 6× estimated temperature (°C) |
| 5 | `CH_EXT_FORCE` | 6× external wrench (N, Nm) |
| 6 | `CH_TCP_POSE` | 6× TCP pose (mm, deg) |
| 7 | `CH_TCP_SPEED` | 1× TCP speed (mm/s) |
| 8 | `CH_SAFETY_STATUS` | Safety level + violation flags |
| 9 | `CH_STATE_MACHINE` | Current state + transition info |

Binary frame format: `[0xAA][0x55][seq][len][channels...][CRC16-CCITT]`

---

## Hardware Constants

| Parameter | Value | Unit |
|-----------|-------|------|
| DH L_BS | 0.109 | m |
| DH D_BS | 0.035 | m |
| DH L_AM | 0.146 | m |
| DH L_FA | 0.115 | m |
| DH D_EW | 0.052 | m |
| DH L_WT | 0.072 | m |
| J1 reduction | 50:1 | — |
| J2 reduction | 30:1 | — |
| J3 reduction | 30:1 | — |
| J4 reduction | 24:1 | — |
| J5 reduction | 30:1 | — |
| J6 reduction | 50:1 | — |
| Torque constant Kt | 0.4 | Nm/A |
| Core loop | 1000 | Hz |
| Motor loop | 20000 | Hz |
| CAN bus | 2.0 | — |

---

> **DevClaw Robot** — 43 commands · 7 phases · Built on [Dummy Robot](https://github.com/peng-zhihui/Dummy-Robot) by [稚晖君](https://github.com/peng-zhihui)
