# DevClaw Feishu Bot Protocol Reference

## Overview

The Feishu bot provides a natural language interface to DevClaw's 43 protocol commands.
All commands use the `/claw` prefix in Feishu group chat or direct messages.

## Architecture

```
User (Feishu App)
    │
    ▼ (text message)
Feishu Cloud
    │
    ▼ (webhook POST, JSON)
Bridge Server (Python/Flask, port 8090)
    │
    ├── Authentication (token verify)
    ├── User permission check
    ├── Command parsing (/claw prefix)
    ├── Safety validation (joint limits, speed caps)
    ├── Protocol translation
    │
    ▼ (ASCII serial)
DevClaw Controller (STM32F4, USB/UART)
    │
    ▼ (response)
Bridge Server
    │
    ▼ (Feishu card / text reply)
User
```

## Command Reference

### Motion Commands

| Feishu Command | Protocol | Args | Description |
|---|---|---|---|
| `/claw home` | `homing` | — | Return all joints to home (zero) position |
| `/claw rest` | `resting` | — | Move to predefined rest pose |
| `/claw movej j1 j2 j3 j4 j5 j6 [spd] [acc]` | `move_j_scurve` | 6 angles (deg) + optional speed/accel | Joint space motion with S-Curve profile |
| `/claw movel x y z a b c` | `move_l_cart` | 6 Cartesian values (mm/deg) | Linear Cartesian motion with SLERP |
| `/claw minjerk j1 j2 j3 j4 j5 j6 dur` | `move_j_minjerk` | 6 angles + duration (s) | Biologically plausible minimum-jerk trajectory |

### Teaching & Learning

| Feishu Command | Protocol | Description |
|---|---|---|
| `/claw teach on` | `set_teach_mode 1` | Enable lead-through teaching (gravity + friction comp) |
| `/claw teach off` | `set_teach_mode 0` | Disable teach mode, hold position |
| `/claw save` | `save_waypoint` | Save current joint position as waypoint |
| `/claw record start` | `set_teach_recording 1` | Start continuous trajectory recording |
| `/claw record stop` | `set_teach_recording 0` | Stop recording, encode as DMP |
| `/claw replay j1 j2 j3 j4 j5 j6` | `execute_dmp` | Replay DMP to target joint goal |
| `/claw dmp record` | `start_dmp_record` | Start DMP demonstration recording |
| `/claw dmp stop` | `stop_dmp_record` | Stop DMP recording |

### Control Mode Switching

| Feishu Command | Protocol | Description |
|---|---|---|
| `/claw mode 0` | `request_mode 0` | IDLE — No active control |
| `/claw mode 1` | `request_mode 1` | POSITION — Standard motor DCE |
| `/claw mode 2` | `request_mode 2` | CTC — Computed torque control |
| `/claw mode 3` | `request_mode 3` | IMPEDANCE — Impedance/admittance |
| `/claw mode 4` | `request_mode 4` | TEACH — Lead-through teaching |
| `/claw mode 5` | `request_mode 5` | DMP — Execute learned trajectory |
| `/claw mode 6` | `request_mode 6` | HYBRID — Force/position hybrid |
| `/claw mode 7` | `request_mode 7` | MINJERK — Trajectory optimization |

### Force & Impedance Control

| Feishu Command | Protocol | Description |
|---|---|---|
| `/claw stiffness val` | `set_stiffness val` | Set impedance stiffness (N/m or Nm/rad) |
| `/claw impedance mode` | `set_impedance_mode mode` | 0=admittance, 1=impedance, 2=variable |
| `/claw hybrid axis mode` | `set_hybrid_axis axis mode` | Set axis force(1)/position(0) mode |
| `/claw forceref fx fy fz` | `set_force_ref fx fy fz` | Set Cartesian force reference (N) |
| `/claw force` | `get_ext_force` | Query estimated external force |
| `/claw collision id` | `set_collision_reaction id` | Set collision reaction (0-4) |

### Safety & Status

| Feishu Command | Protocol | Description |
|---|---|---|
| `/claw status` | `get_robot_state` | Query state (returns interactive card) |
| `/claw safety` | `safety_check` | Run full safety check |
| `/claw estop` | *(special: disable all)* | **Emergency stop** — disables all motors |
| `/claw reset` | `reset_estop` | Reset E-Stop, return to IDLE |
| `/claw speed val` | `set_tcp_speed_limit val` | Set TCP speed limit (mm/s) |

### Advanced Control

| Feishu Command | Protocol | Description |
|---|---|---|
| `/claw dob on` | `set_dob_enabled 1` | Enable disturbance observer |
| `/claw dob off` | `set_dob_enabled 0` | Disable disturbance observer |
| `/claw ctc on` | `set_ctc_enabled 1` | Enable computed torque control |
| `/claw ctc off` | `set_ctc_enabled 0` | Disable computed torque control |
| `/claw friction val` | `set_friction_comp val` | Set LuGre friction compensation |

### Calibration & Diagnostics

| Feishu Command | Protocol | Description |
|---|---|---|
| `/claw workspace` | `analyze_workspace` | Compute manipulability & joint margins |
| `/claw singular` | `is_near_singular` | Check singularity proximity |
| `/claw calib gravity` | `record_grav_sample` | Record gravity calibration sample |
| `/claw calib run` | `run_grav_calib` | Execute gravity WLS calibration |
| `/claw calib kin x y z` | `record_kin_sample x y z` | Record kinematic calibration point |
| `/claw calib kinrun` | `run_kin_calib` | Execute kinematic offset calibration |
| `/claw telemetry ch rate` | `config_telemetry ch rate` | Configure telemetry streaming |

## Safety Constraints (Remote Mode)

When `safety_mode` is enabled in config:

| Parameter | Default | Description |
|---|---|---|
| `max_speed_remote` | 50 mm/s | Maximum TCP speed via Feishu |
| `max_joint_step_deg` | 30° | Maximum single-command joint step |
| Joint range | ±180° | Per-joint angle limit check |
| Heartbeat | 30s | Periodic `safety_check` to detect disconnection |

Commands that violate safety constraints are rejected with an error message.

## Response Format

- **Text reply**: `✅ command_name → response`
- **Status card**: Interactive Feishu card with joints, force, safety, manipulability
- **Error**: `⚠️ Safety violation: description`

## Setup Checklist

1. Create Feishu custom app at https://open.feishu.cn/
2. Enable Bot capability
3. Subscribe to `im.message.receive_v1` event
4. Set webhook URL: `https://your-server/devclaw/webhook`
5. Edit `feishu_config.json` with your credentials
6. `pip install -r requirements.txt`
7. `python feishu_bridge.py --config feishu_config.json`
8. Connect DevClaw via USB serial
9. Send `/claw status` in Feishu to verify connection
