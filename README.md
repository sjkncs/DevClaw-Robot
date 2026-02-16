<div align="center">

> **⚠️ Derivative Work Notice / 衍生项目声明**
>
> This project is a **derivative work** (二次开发) based on [**Dummy Robot**](https://github.com/peng-zhihui/Dummy-Robot) by [**稚晖君 (peng-zhihui)**](https://github.com/peng-zhihui).
> The original hardware design, mechanical models, core firmware framework, and motor driver are the intellectual property of the original author.
> DevClaw Robot extends the original with 7 phases of algorithm development, 43 protocol commands, and a Feishu bot integration.
> We deeply appreciate 稚晖君's outstanding open-source contribution to the robotics community.
> See [`ATTRIBUTION.md`](ATTRIBUTION.md) for a detailed breakdown of original vs. new components.
>
> 本项目基于 [**稚晖君**](https://github.com/peng-zhihui) 的 [**Dummy Robot**](https://github.com/peng-zhihui/Dummy-Robot) 开源项目进行二次开发。
> 原始硬件设计、机械模型、核心固件框架和电机驱动均为原作者的知识产权。
> DevClaw Robot 在此基础上扩展了 7 个阶段的算法开发、43 条协议指令和飞书机器人集成。
> 衷心感谢稚晖君对开源机器人社区的杰出贡献。详见 [`ATTRIBUTION.md`](ATTRIBUTION.md)。

<img src="5.Docs/1.Images/dummy1.jpg" width="600" alt="DevClaw Robot"/>

# 🦾 DevClaw Robot

### A Feature-Rich 6-DOF Robotic Arm Platform — From Dynamics to Remote Control

<a href="#cn">中文</a> | <a href="#en">English</a>

<br/>

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](LICENSE)
[![Firmware: STM32](https://img.shields.io/badge/Firmware-STM32F4%20%7C%20STM32F1-blue?style=for-the-badge&logo=stmicroelectronics)](2.Firmware/)
[![Protocol: CAN](https://img.shields.io/badge/Bus-CAN%202.0-green?style=for-the-badge)](5.Docs/API_Reference.md)
[![Commands: 43](https://img.shields.io/badge/API%20Commands-43-purple?style=for-the-badge)](5.Docs/API_Reference.md)
[![Algorithms: 21](https://img.shields.io/badge/Algorithm%20Modules-21-red?style=for-the-badge)](5.Docs/API_Reference.md)
[![Feishu Bot](https://img.shields.io/badge/Feishu-Bot%20Control-00D09C?style=for-the-badge&logo=bytedance)](3.Software/FeishuBot/)

<br/>

<a href="https://github.com/peng-zhihui/Dummy-Robot/releases">
  <img src="https://img.shields.io/badge/⬇_Download_Release-blue?style=for-the-badge&logo=github" alt="Download"/>
</a>
<a href="https://www.bilibili.com/video/BV12341117rG">
  <img src="https://img.shields.io/badge/📺_Bilibili_Video-FF6699?style=for-the-badge&logo=bilibili&logoColor=white" alt="Bilibili"/>
</a>
<a href="https://www.youtube.com/watch?v=F29vrvUwqS4">
  <img src="https://img.shields.io/badge/▶_YouTube_Demo-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="YouTube"/>
</a>

</div>

---

<a name="en"></a>

## 📖 Table of Contents

- [What is DevClaw?](#-what-is-devclaw)
- [Architecture](#-system-architecture)
- [Project Layout](#-project-layout)
- [Algorithm Modules (7 Phases)](#-algorithm-modules--7-phases)
- [43 Protocol Commands](#-43-protocol-commands)
- [Control Modes & State Machine](#-control-modes--state-machine)
- [Feishu Bot Integration](#-feishu-bot-integration)
- [Hardware Overview](#-hardware-overview)
- [Getting Started](#-getting-started)
- [API Reference](#-api-reference)
- [Credits](#-credits)

---

## ✨ What is DevClaw?

DevClaw Robot is a **desktop-scale 6-DOF robotic arm** built for learning, prototyping, and hands-on experimentation with modern robot control. It ships with a layered firmware stack covering trajectory generation, force-aware interaction, and remote chat-based operation — all on affordable STM32 hardware.

<table>
<tr>
<td width="50%">

**🧠 Multi-Layer Control Stack (7 Phases)**
- Recursive Newton-Euler dynamics with gravity compensation
- Extended Kalman state estimation at 20 kHz per joint
- Computed Torque Control with feedback linearisation
- LuGre friction modelling and online adaptation
- Three impedance paradigms: admittance / impedance / variable
- Current-based external force and torque estimation
- Trajectory learning via Dynamic Movement Primitives

</td>
<td width="50%">

**🛡️ Reliability & Diagnostics**
- Safety architecture following ISO 10218 / TS 15066 guidelines
- Finite state machine with 12 states and guarded transitions
- Per-joint I²t thermal monitoring with predictive shutdown
- Momentum-based collision detection, five configurable reactions
- Streaming telemetry over 10 channels with CRC-16 integrity
- Joint-level and Cartesian-space calibration routines
- Manipulability analysis and singularity proximity warnings

</td>
</tr>
<tr>
<td>

**🎯 Trajectory & Planning**
- Seven-segment S-Curve profiles, multi-axis synchronised
- Cartesian straight-line paths with SLERP rotation blending
- Minimum-jerk polynomials for human-like smoothness
- Natural cubic splines for multi-waypoint paths
- Time-optimal re-parameterisation
- Damped Least-Squares IK with automatic singularity damping

</td>
<td>

**🤖 Interaction & Remote Control**
- Zero-gravity lead-through teaching with auto-brake
- Hybrid force/position control with spiral insertion search
- One-shot DMP recording and goal-adaptive replay
- Feishu (飞书) bot for chat-based remote operation
- 43 text commands accessible over USB, UART, or CAN

</td>
</tr>
</table>

---

## 🏗️ System Architecture

```
┌─ Host PC / Feishu Bot ────────────────────────────────────┐
│  43 Protocol Commands (USB / UART / Feishu Webhook)        │
└───────────────────────────┬────────────────────────────────┘
                            ▼
┌─ Core Controller (STM32F4, 1kHz) ──────────────────────────┐
│                                                              │
│  MasterControlTick()                                         │
│  ├─ 1. RequestMotorFeedback()  ← CAN KF state ×6           │
│  ├─ 2. StateMachine.Tick()     ← 12-state FSM              │
│  ├─ 3. SafetyCheck()           ← ISO limits + thermal      │
│  ├─ 4. SpeedScale              ← Adaptive speed [0-1]      │
│  ├─ 5. Dispatch(mode)          ← 8 control modes           │
│  │     ├─ POSITION  → Motor DCE                            │
│  │     ├─ CTC       → ModelBasedControlTick                │
│  │     ├─ IMPEDANCE → SafeInteractionTick                  │
│  │     ├─ TEACH     → TeachAndForceTick                    │
│  │     ├─ DMP       → DMPTick                              │
│  │     ├─ HYBRID    → Force/Position Hybrid                │
│  │     ├─ MINJERK   → Trajectory Optimizer                 │
│  │     └─ STOPPING  → Controlled deceleration              │
│  ├─ 6. DOB compensation        ← Disturbance rejection     │
│  └─ 7. Telemetry.Update()      ← Stream to host            │
│                                                              │
│  ┌─ Algorithms ──────────────────────────────────────────┐  │
│  │ Kinematics: FK/IK/DLS/Jacobian/Workspace/Calibration  │  │
│  │ Dynamics:   RNEA/MassMatrix/Coriolis/Gravity          │  │
│  │ Trajectory: S-Curve/Cartesian/MinJerk/Spline          │  │
│  │ Control:    DOB/CTC/Impedance/Force/Friction/Teach    │  │
│  │ Safety:     Monitor/FSM/Telemetry                     │  │
│  │ Learning:   DMP                                       │  │
│  │ Ident:      ParamID/GravCalib/KinCalib                │  │
│  └───────────────────────────────────────────────────────┘  │
└───────────────────────────┬──────────────────────────────────┘
                            │ CAN Bus (4 wires)
                            ▼
┌─ Ctrl-Step Motor Driver (STM32F1, 20kHz) ×6 ───────────────┐
│  Encoder → KalmanFilter(pos/vel/acc) → DCE/PID → Stepper   │
│  CAN Feedback: estVelocity, estAcceleration, motorCurrent   │
└─────────────────────────────────────────────────────────────┘
```

---

## 📁 Project Layout

```
DevClaw-Robot/
│
├── 1.Hardware/                      # PCB & schematic design files
│   ├── REF-Core-Board/              # Main controller board
│   ├── REF-Base-Board/              # Base station board
│   ├── Ctrl-Step-Driver-20/         # NEMA17 motor driver
│   ├── Ctrl-Step-Driver-42/         # NEMA23 motor driver
│   └── LED-Ring/                    # Gripper LED ring
│
├── 2.Firmware/
│   ├── Core-STM32F4-fw/             # 🧠 Main controller firmware
│   │   └── Robot/
│   │       ├── instances/
│   │       │   ├── devclaw_robot.h       # Robot class — 23 includes, 43 commands
│   │       │   └── devclaw_robot.cpp     # ~1977 lines, 7-phase implementation
│   │       ├── algorithms/
│   │       │   ├── kinematic/            # FK, IK, DLS, Workspace, KinCalib
│   │       │   ├── dynamics/             # RNEA, Mass, Coriolis, Gravity, Jacobian
│   │       │   ├── trajectory/           # S-Curve, Cartesian, MinJerk, Spline
│   │       │   ├── control/              # DOB, CTC, Impedance, Force, Friction,
│   │       │   │                         # Teach, Hybrid, AutoTuner, Collision
│   │       │   ├── identification/       # ParamID, GravityCalib
│   │       │   ├── learning/             # DMP
│   │       │   └── safety/               # SafetyMonitor, StateMachine, Telemetry
│   │       └── actuators/
│   │           └── ctrl_step/            # CAN motor interface + KF feedback
│   │
│   └── Ctrl-Step-Driver-STM32F1-fw/ # Motor driver firmware
│       └── Ctrl/Motor/
│           ├── kalman_filter.h/.cpp      # 3-state EKF + RTS smoother
│           └── motor.h/.cpp              # 20kHz control loop
│
├── 3.Software/
│   ├── DevClawStudio/               # PC control software (Unity3D)
│   └── FeishuBot/                   # 🤖 Feishu webhook bridge
│       ├── feishu_config.json        # Bot credentials & limits
│       ├── feishu_bridge.py          # Flask ↔ Serial bridge server
│       └── feishu_protocol.md        # Command mapping reference
│
├── 4.Model/                         # 3D CAD model files (STEP/STL)
│
├── 5.Docs/
│   ├── 1.Images/                    # Documentation images
│   └── API_Reference.md             # Full 43-command API reference
│
└── SubModules/                      # Peak wireless controller
```

---

## 🧠 Algorithm Modules — 7 Phases

> 42 new files (21 headers + 21 implementations) across 7 algorithm categories.

<details>
<summary><b>Phase 1 — Core Algorithm Optimizations</b></summary>

| Module | File | Description |
|--------|------|-------------|
| **RNEA Dynamics** | `dynamics/6dof_dynamics.h/.cpp` | Newton-Euler recursive inverse dynamics, mass matrix, Coriolis, gravity, Jacobian |
| **S-Curve Planner** | `trajectory/s_curve_planner.h/.cpp` | 7-segment S-curve with multi-axis synchronization |
| **Cartesian Planner** | `trajectory/cartesian_planner.h/.cpp` | Linear + SLERP orientation + circular arc interpolation |
| **DLS-IK Solver** | `kinematic/dls_ik_solver.h/.cpp` | Damped least-squares IK with singularity robustness |

</details>

<details>
<summary><b>Phase 2 — Robust Control & Identification</b></summary>

| Module | File | Description |
|--------|------|-------------|
| **Kalman Filter** | `Motor/kalman_filter.h/.cpp` | 3-state EKF (pos/vel/acc) + Rauch-Tung-Striebel smoother, 20kHz |
| **Disturbance Observer** | `control/disturbance_observer.h/.cpp` | 2nd-order Butterworth DOB for each joint |
| **Auto-Tuner** | `control/auto_tuner.h/.cpp` | Relay feedback (Åström-Hägglund) + MRAC adaptive |
| **Param Identifier** | `identification/param_identifier.h/.cpp` | Fourier excitation + WLS regression for dynamic params |

</details>

<details>
<summary><b>Phase 3 — Safe Interaction & Learning</b></summary>

| Module | File | Description |
|--------|------|-------------|
| **Impedance Controller** | `control/impedance_controller.h/.cpp` | Admittance / Impedance / Variable stiffness + passivity enforcement |
| **Collision Detector** | `control/collision_detector.h/.cpp` | Generalized momentum observer + 5 reaction strategies |
| **DMP** | `learning/dmp.h/.cpp` | Dynamic Movement Primitives, multi-DOF, LWR weight learning |

</details>

<details>
<summary><b>Phase 4 — Model-Based Control & Force Sensing</b></summary>

| Module | File | Description |
|--------|------|-------------|
| **Force Estimator** | `control/force_estimator.h/.cpp` | Sensorless F/T from motor current + RNEA residual, Cartesian mapping |
| **Computed Torque** | `control/computed_torque.h/.cpp` | Full CTC with feedback linearization + integral action |
| **Friction Compensator** | `control/friction_compensator.h/.cpp` | LuGre dynamic friction model + online adaptation |

</details>

<details>
<summary><b>Phase 5 — Teaching, Assembly & Trajectory Optimization</b></summary>

| Module | File | Description |
|--------|------|-------------|
| **Teach Mode** | `control/teach_mode.h/.cpp` | Gravity comp + friction comp + virtual walls + auto-brake + recording |
| **Hybrid Force/Position** | `control/hybrid_force_position.h/.cpp` | Raibert-Craig selection matrix + PI force / PD position + spiral search |
| **Trajectory Optimizer** | `trajectory/trajectory_optimizer.h/.cpp` | Minimum-jerk (5th-order) + time-optimal + cubic spline (Thomas algo) |

</details>

<details>
<summary><b>Phase 6 — Safety, State Machine & Telemetry</b></summary>

| Module | File | Description |
|--------|------|-------------|
| **Safety Monitor** | `safety/safety_monitor.h/.cpp` | ISO 10218/15066, I²t thermal model, CAN watchdog, 5-level hierarchy |
| **State Machine** | `safety/robot_state_machine.h/.cpp` | 12 states, event-driven transitions, guard conditions |
| **Telemetry Streamer** | `safety/telemetry_streamer.h/.cpp` | 10 data channels, binary (CRC-16-CCITT) + ASCII CSV, configurable decimation |

</details>

<details>
<summary><b>Phase 7 — Calibration & Workspace Analysis</b></summary>

| Module | File | Description |
|--------|------|-------------|
| **Workspace Analyzer** | `kinematic/workspace_analyzer.h/.cpp` | Yoshikawa manipulability, condition number (power iteration), joint margin, singularity avoidance |
| **Gravity Calibrator** | `identification/gravity_calibrator.h/.cpp` | Multi-pose static calibration, WLS regression for gravity + payload ID |
| **Kinematic Calibrator** | `kinematic/kinematic_calibrator.h/.cpp` | DH parameter offset calibration via numerical Jacobian + regularized LS |

</details>

---

## 📡 43 Protocol Commands

Every command listed below can be issued over **USB**, **UART**, **CAN**, or through the **Feishu Bot** bridge.

<table>
<tr><td><b>Phase</b></td><td><b>#</b></td><td><b>Commands</b></td></tr>
<tr>
<td><b>P1</b> Motion</td><td>4</td>
<td><code>move_j_scurve</code> <code>move_l_cart</code> <code>set_dynamics_enabled</code> <code>get_manipulability</code></td>
</tr>
<tr>
<td><b>P2</b> Robust</td><td>5</td>
<td><code>set_dob_enabled</code> <code>set_dob_cutoff</code> <code>start_auto_tune</code> <code>start_identification</code> <code>run_identification</code></td>
</tr>
<tr>
<td><b>P3</b> Interact</td><td>7</td>
<td><code>set_impedance_mode</code> <code>set_stiffness</code> <code>set_collision_reaction</code> <code>reset_collision</code> <code>start_dmp_record</code> <code>stop_dmp_record</code> <code>execute_dmp</code></td>
</tr>
<tr>
<td><b>P4</b> Model</td><td>5</td>
<td><code>set_ctc_enabled</code> <code>set_ctc_frequency</code> <code>set_friction_comp</code> <code>calibrate_force</code> <code>get_ext_force</code></td>
</tr>
<tr>
<td><b>P5</b> Teach</td><td>6</td>
<td><code>set_teach_mode</code> <code>save_waypoint</code> <code>set_teach_recording</code> <code>move_j_minjerk</code> <code>set_hybrid_axis</code> <code>set_force_ref</code></td>
</tr>
<tr>
<td><b>P6</b> Safety</td><td>6</td>
<td><code>safety_check</code> <code>reset_estop</code> <code>request_mode</code> <code>set_tcp_speed_limit</code> <code>config_telemetry</code> <code>get_robot_state</code></td>
</tr>
<tr>
<td><b>P7</b> Calib</td><td>6</td>
<td><code>analyze_workspace</code> <code>record_grav_sample</code> <code>run_grav_calib</code> <code>record_kin_sample</code> <code>run_kin_calib</code> <code>is_near_singular</code></td>
</tr>
</table>

> **43 commands in total**, spanning 7 development phases. All commands double as Fibre RPC endpoints.

---

## 🔄 Control Modes & State Machine

```
  POWERED_OFF ──► INITIALIZING ──► IDLE
                                    │
        ┌──────────┬──────────┬─────┴────┬──────────┬──────────┐
        ▼          ▼          ▼          ▼          ▼          ▼
    POSITION   CTC_TORQUE  IMPEDANCE   TEACH    DMP_EXEC   HYBRID
        │          │          │          │          │          │
        └──────────┴──────────┴────┬─────┴──────────┴──────────┘
                                   ▼
                                STOPPING ──► IDLE
                                   │
                                   ▼
                                 ESTOP (latched, manual reset required)
```

| Mode | Description | Torque Control |
|------|-------------|:-:|
| **POSITION** | Standard DCE position control on motor driver | ✗ |
| **CTC_TORQUE** | Full model-based computed torque | ✔ |
| **IMPEDANCE** | Admittance/impedance/variable stiffness | ✔ |
| **TEACH** | Lead-through with gravity + friction compensation | ✔ |
| **DMP_EXEC** | Replay learned DMP trajectory | ✗ |
| **HYBRID** | Parallel force/position hybrid (Raibert-Craig) | ✔ |
| **MINJERK** | Minimum-jerk optimal trajectory execution | ✗ |
| **STOPPING** | Controlled deceleration, zero current | — |

---

## 🤖 Feishu Bot Integration

DevClaw can be operated remotely through a **Feishu (飞书) Bot**. Messages prefixed with `/claw` are translated into protocol commands by a lightweight Python bridge server.

<details>
<summary><b>Setup & Configuration</b></summary>

### 1. Create Feishu Bot

1. Go to [Feishu Open Platform](https://open.feishu.cn/) → Create Custom App
2. Enable **Bot** capability
3. Add **Event Subscription**: `im.message.receive_v1`
4. Point the event webhook to your bridge server: `https://your-server.com/devclaw/webhook`
5. Record `App ID` and `App Secret`

### 2. Configuration File

Create `3.Software/FeishuBot/feishu_config.json`:

```json
{
  "app_id": "cli_your_app_id",
  "app_secret": "your_app_secret",
  "verification_token": "your_verification_token",
  "encrypt_key": "",
  "robot_serial_port": "COM3",
  "robot_baud_rate": 115200,
  "allowed_user_ids": ["ou_xxxx"],
  "command_prefix": "/claw",
  "safety_mode": true,
  "max_speed_remote": 50,
  "feishu_api_base": "https://open.feishu.cn/open-apis"
}
```

### 3. Command Quick-Reference

| Chat Message | Mapped Command | Purpose |
|---|---|---|
| `/claw home` | `homing` | Drive every joint to its zero position |
| `/claw rest` | `resting` | Move to the predefined rest pose |
| `/claw movej 0 30 -60 0 0 0` | `move_j_scurve` | Joint-space S-Curve motion |
| `/claw movel 200 0 300 0 0 0` | `move_l_cart` | Cartesian straight-line motion |
| `/claw teach on` | `set_teach_mode 1` | Enable teach mode |
| `/claw teach off` | `set_teach_mode 0` | Disable teach mode |
| `/claw save` | `save_waypoint` | Save current position |
| `/claw record start` | `set_teach_recording 1` | Start recording |
| `/claw record stop` | `set_teach_recording 0` | Stop & encode DMP |
| `/claw replay 0 30 -60 0 0 0` | `execute_dmp` | Replay DMP to goal |
| `/claw mode 0-7` | `request_mode` | Switch control mode |
| `/claw status` | `get_robot_state` | Get current state |
| `/claw force` | `get_ext_force` | Get external force |
| `/claw estop` | — (E-Stop) | Emergency stop |
| `/claw reset` | `reset_estop` | Reset E-Stop |
| `/claw stiffness 500` | `set_stiffness` | Set impedance stiffness |
| `/claw speed 100` | `set_tcp_speed_limit` | Set TCP speed limit |
| `/claw calib gravity` | `record_grav_sample` | Record gravity sample |
| `/claw calib run` | `run_grav_calib` | Run gravity calibration |
| `/claw workspace` | `analyze_workspace` | Analyze workspace quality |
| `/claw telemetry 100` | `config_telemetry` | Set telemetry rate |

Responses come back as rich Feishu cards with joint angles, force readings, and safety status.

</details>

<details>
<summary><b>Feishu Bridge Server (Python)</b></summary>

The bridge converts incoming Feishu webhook payloads into serial protocol frames:

```
Feishu Cloud ──webhook──► Bridge Server (Flask) ──serial──► DevClaw (USB/UART)
                              │
                              ├─ Authenticate & parse /claw messages
                              ├─ Enforce remote-mode safety limits
                              ├─ Map to protocol command strings
                              ├─ Transmit over serial port
                              ├─ Collect response
                              └─ Reply with a Feishu card or text
```

**Dependencies**: Python 3.8+, `pyserial`, `flask`, `requests`

```bash
pip install -r 3.Software/FeishuBot/requirements.txt
python 3.Software/FeishuBot/feishu_bridge.py --config 3.Software/FeishuBot/feishu_config.json
```

</details>

---

## 🔧 Hardware Overview

<table>
<tr>
<td width="50%">

**Boards**
- **REF Core Board** — STM32F407 main controller, CAN / USB / UART
- **REF Base Board** — Power regulation, IMU, OLED, buzzer
- **Ctrl-Step Driver ×6** — STM32F103, 20 kHz closed-loop stepper
- **Peak Controller** — Wireless teaching pendant

</td>
<td width="50%">

**Specifications**
- **Degrees of freedom**: 6 (+ optional gripper)
- **Gear ratios**: J1 50:1, J2 30:1, J3 30:1, J4 24:1, J5 30:1, J6 50:1
- **DH lengths (m)**: L_BS 0.109, D_BS 0.035, L_AM 0.146, L_FA 0.115, D_EW 0.052, L_WT 0.072
- **Loop rates**: 1 kHz core · 20 kHz motor
- **Bus**: CAN 2.0 (VCC, GND, CANH, CANL)

</td>
</tr>
</table>

![DevClaw assembly](5.Docs/1.Images/case.png)

---

## 🚀 Getting Started

<details>
<summary><b>Development Environment</b></summary>

### Prerequisites

- **STM32CubeIDE** or **Keil MDK-ARM** v5.x
- **STM32CubeMX** (HAL code generation)
- **ST-Link V2** programmer / debugger
- **Python 3.8+** (only needed for the Feishu bridge)
- **CAN analyser** (handy but optional)

### Build & Flash

```bash
# 1. Clone the repository
git clone https://github.com/your-username/DevClaw-Robot.git
cd DevClaw-Robot

# 2. Core controller firmware (STM32F4)
#    Open 2.Firmware/Core-STM32F4-fw/ in STM32CubeIDE → Build → Flash

# 3. Motor driver firmware (STM32F1) — repeat for each of the 6 drivers
#    Open 2.Firmware/Ctrl-Step-Driver-STM32F1-fw/ → Build → Flash

# 4. (Optional) Launch the Feishu bridge
pip install -r 3.Software/FeishuBot/requirements.txt
python 3.Software/FeishuBot/feishu_bridge.py --config 3.Software/FeishuBot/feishu_config.json
```

### First-Run Calibration

```bash
# Over USB serial or through the Feishu bot:
1. Power on — encoders auto-calibrate on boot
2. /claw home                    # Run the homing sequence
3. /claw calib gravity           # Record 10+ static poses for gravity ID
4. /claw calib run               # Solve the WLS regression
5. /claw status                  # Confirm the arm reports IDLE
```

</details>

---

## 📋 API Reference

> Full reference: [`5.Docs/API_Reference.md`](5.Docs/API_Reference.md)

<details>
<summary><b>Command Format (Fibre Protocol)</b></summary>

```
# USB/UART text protocol:
command_name arg1 arg2 ...

# Examples:
move_j_scurve 0 30 -60 0 0 0 1.0 1.0
set_impedance_mode 1
set_teach_mode 1
safety_check
get_robot_state
```

</details>

<details>
<summary><b>Command Modes</b></summary>

| Mode | Freq | Execution | Interruptible | Use Case |
|------|------|-----------|:---:|------|
| **SEQ** (Sequential) | Low (<5Hz) | FIFO queue | ✗ | Pick-and-place, palletizing |
| **INT** (Immediate) | Any | Override, instant | ✔ | Real-time sync, teleoperation |
| **TRJ** (Trajectory) | High (200Hz) | Auto-interpolate | ✗ | 3D printing, drawing |

</details>

---

<a name="cn"></a>

## 🇨🇳 中文说明

<div align="center">

### DevClaw — 桌面级六轴智能机械臂平台

*在 [稚晖君](https://github.com/peng-zhihui) 原始项目基础上进行七轮迭代开发*

</div>

### 项目简介

DevClaw Robot 旨在提供一个**可动手实验的六自由度机械臂**——从轨迹规划、力控交互到远程飞书操控，全部运行在低成本 STM32 硬件上。项目历经七个阶段的增量开发，逐步搭建起完整的机器人控制软件栈。

### 七轮迭代一览

| 阶段 | 主题 | 新增模块 |
|------|------|--------|
| **Phase 1** | 核心运动与动力学 | RNEA 逆动力学、S-Curve 规划器、笛卡尔插补、DLS-IK |
| **Phase 2** | 鲁棒控制与参数辨识 | 卡尔曼滤波、扰动观测器、自整定、WLS 参数辨识 |
| **Phase 3** | 安全交互与运动学习 | 阻抗控制、碰撞检测（5 种反应策略）、DMP |
| **Phase 4** | 基于模型的力控 | 计算力矩控制、LuGre 摩擦补偿、无传感器力估计 |
| **Phase 5** | 示教装配与轨迹优化 | 拖动示教、力位混合、最小 Jerk + 样条 |
| **Phase 6** | 安全状态机与遥测 | ISO 安全监控、12 态 FSM、10 通道遥测流 |
| **Phase 7** | 标定与工作空间分析 | 操作度分析、重力/运动学标定 |

### 飞书远程控制

在飞书群聊或私聊中输入以 `/claw` 开头的指令，即可通过 Bridge Server 透传给 DevClaw：

```
/claw home          # 全关节回零
/claw movej 0 30 -60 0 0 0    # S-Curve 关节运动
/claw teach on      # 开启零重力拖动示教
/claw save          # 记录当前关节位置
/claw replay ...    # DMP 轨迹回放
/claw estop         # 紧急停止（电机失能）
/claw status        # 查询状态机与安全等级
```

### DH参数

| 参数 | L_BS | D_BS | L_AM | L_FA | D_EW | L_WT |
|------|------|------|------|------|------|------|
| **值 (m)** | 0.109 | 0.035 | 0.146 | 0.115 | 0.052 | 0.072 |

### 减速比

| 关节 | J1 | J2 | J3 | J4 | J5 | J6 |
|------|-----|-----|-----|-----|-----|-----|
| **减速比** | 50 | 30 | 30 | 24 | 30 | 50 |

### 机械结构说明

原始设计采用 `步进电机` + Harmonic `谐波减速模组`。若想降低成本，可改用 `自制摆线针轮减速器` + `3D 打印` 方案，整机物料费可压到 2000 元以内。

摆线减速器参考: [peng-zhihui/CycloidAcuratorNano](https://github.com/peng-zhihui/CycloidAcuratorNano)

### 电路板组成

- **REF 核心板** — STM32F407 主控，CAN / USB / UART
- **REF 底板** — 电源、IMU、OLED、蜂鸣器
- **Ctrl-Step 驱动器 ×6** — STM32F103，20 kHz 闭环步进控制
- **Peak 示教器** — 无线遥控手柄

### 指令模式

| 模式 | 发送频率 | 执行方式 | 可打断 | 适用场景 |
|------|---------|---------|:------:|---------|
| **SEQ** 顺序 | 低 (<5Hz) | FIFO队列 | ✗ | 视觉抓取、码垛 |
| **INT** 实时 | 不限 | 覆盖执行 | ✔ | 动作同步、遥操作 |
| **TRJ** 轨迹 | 高 (200Hz) | 自动插值 | ✗ | 3D打印、绘画 |

---

## 🙏 Credits

<div align="center">

<table>
<tr>
<td align="center" width="50%">
<a href="https://github.com/peng-zhihui/Dummy-Robot">
<img src="https://img.shields.io/badge/Original_Project-peng--zhihui-blue?style=for-the-badge&logo=github" alt="Original Project"/>
</a>
<br/><br/>
<b>稚晖君 (peng-zhihui)</b><br/>
<sub>Created the original open-source robotic arm that serves as the mechanical and electrical foundation for DevClaw’s seven-phase firmware expansion.</sub>
</td>
<td align="center" width="50%">
<a href="https://github.com/unlir/XDrive">
<img src="https://img.shields.io/badge/Motor_Driver-XDrive-green?style=for-the-badge&logo=github" alt="XDrive"/>
</a>
<br/><br/>
<b>unlir</b><br/>
<sub>Stepper motor closed-loop driver whose CAN bus architecture and encoder calibration flow informed the Ctrl-Step driver design.</sub>
</td>
</tr>
<tr>
<td align="center">
<a href="https://github.com/odriverobotics/ODrive">
<img src="https://img.shields.io/badge/Protocol-ODrive-orange?style=for-the-badge&logo=github" alt="ODrive"/>
</a>
<br/><br/>
<b>ODrive Robotics</b><br/>
<sub>Motor control platform whose Fibre RPC layer underpins DevClaw’s text-based command protocol.</sub>
</td>
<td align="center">
<a href="https://github.com/olikraus/u8g2">
<img src="https://img.shields.io/badge/Display-U8G2-lightgrey?style=for-the-badge&logo=github" alt="U8G2"/>
</a>
&nbsp;
<a href="https://github.com/samuelsadok/fibre">
<img src="https://img.shields.io/badge/RPC-Fibre-yellow?style=for-the-badge&logo=github" alt="Fibre"/>
</a>
<br/><br/>
<b>U8G2 & Fibre</b><br/>
<sub>OLED rendering library and lightweight RPC serialisation framework used across the firmware.</sub>
</td>
</tr>
</table>

</div>

---

> **License**: Released under the MIT License. The upstream project by [peng-zhihui](https://github.com/peng-zhihui) retains its own license terms.

---

<div align="center">

<sub>
<b>DevClaw Robot</b> — Extending <a href="https://github.com/peng-zhihui/Dummy-Robot">peng-zhihui’s original project</a> with 7 phases of firmware development<br/>
42 algorithm source files · 43 protocol commands · ~2 000 lines of core control logic<br/>
Kinematics → Dynamics → Robust Control → Force Sensing → Teaching → Safety → Calibration<br/>
<br/>
Made with ❤️ for the open-source robotics community
</sub>

</div>

