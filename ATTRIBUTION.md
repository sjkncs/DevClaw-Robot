# Attribution & Intellectual Property Notice

## Original Project

**DevClaw Robot** is a derivative work based on the **Dummy Robot** project by **稚晖君 (peng-zhihui)**.

| | |
|---|---|
| **Original Author** | [稚晖君 (peng-zhihui)](https://github.com/peng-zhihui) |
| **Original Repository** | [github.com/peng-zhihui/Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot) |
| **Original License** | MIT License |
| **Original Project Name** | Dummy Robot |

We express our sincere gratitude to 稚晖君 for creating and open-sourcing the Dummy Robot project. His outstanding work on hardware design, kinematic firmware, motor driver, CAN communication, and the Peak wireless controller forms the foundation upon which DevClaw Robot is built.

---

## What Belongs to the Original Project

The following components are created by 稚晖君 and the original Dummy Robot project. All intellectual property rights for these components remain with the original author:

### Hardware Design (entirely original)
- `1.Hardware/` — All PCB schematics and board designs
  - REF-Core-Board (STM32F407 main controller)
  - REF-Base-Board (power, IMU, OLED, buzzer)
  - Ctrl-Step-Driver-20 / Ctrl-Step-Driver-42 (stepper motor drivers)
  - LED-Ring (gripper LED ring)

### Mechanical Design (entirely original)
- `4.Model/` — All 3D CAD model files (STEP/STL)

### Core Firmware Framework (original, with modifications noted below)
- `2.Firmware/Core-STM32F4-fw/` — HAL configuration, RTOS setup, communication framework
  - `UserApp/main.cpp` — Original structure by 稚晖君, **modified** to use DevClawRobot class
  - `UserApp/protocols/` — Original Fibre protocol framework, **modified** extern declarations
  - `Robot/actuators/ctrl_step/` — Original CAN motor interface
  - `Robot/frame/` — Original kinematic solver (6dof_kinematic.h/.cpp)

### Motor Driver Firmware (original, with additions)
- `2.Firmware/Ctrl-Step-Driver-STM32F1-fw/` — Original motor driver firmware
  - `Ctrl/Motor/motor.h/.cpp` — Original 20 kHz control loop
  - `Ctrl/Motor/kalman_filter.h/.cpp` — **Added** by DevClaw project (3-state EKF)

### PC Software (original)
- `3.Software/` — Original Unity3D control software (renamed folder only)

### SubModules (original)
- `0.SubModules/` — Peak wireless controller by 稚晖君

---

## What DevClaw Robot Added

The following components are new additions created by the DevClaw Robot project, built on top of the original Dummy Robot framework:

### New Algorithm Modules — 42 source files (21 .h + 21 .cpp)
All files under `2.Firmware/Core-STM32F4-fw/Robot/algorithms/` are **new**:

| Directory | New Files | Description |
|-----------|-----------|-------------|
| `algorithms/dynamics/` | `6dof_dynamics.h/.cpp` | RNEA inverse dynamics, mass matrix, Coriolis, gravity, Jacobian |
| `algorithms/trajectory/` | `s_curve_planner.h/.cpp` | 7-segment S-Curve trajectory planner |
| `algorithms/trajectory/` | `cartesian_planner.h/.cpp` | Cartesian linear + SLERP + circular arc |
| `algorithms/trajectory/` | `trajectory_optimizer.h/.cpp` | Minimum-jerk, time-optimal, cubic spline |
| `algorithms/kinematic/` | `dls_ik_solver.h/.cpp` | Damped least-squares IK |
| `algorithms/kinematic/` | `workspace_analyzer.h/.cpp` | Manipulability, condition number, joint margin |
| `algorithms/kinematic/` | `kinematic_calibrator.h/.cpp` | DH offset calibration |
| `algorithms/control/` | `disturbance_observer.h/.cpp` | Multi-joint DOB |
| `algorithms/control/` | `auto_tuner.h/.cpp` | Relay feedback + MRAC auto-tuning |
| `algorithms/control/` | `impedance_controller.h/.cpp` | Admittance / impedance / variable stiffness |
| `algorithms/control/` | `collision_detector.h/.cpp` | Momentum-based collision detection |
| `algorithms/control/` | `force_estimator.h/.cpp` | Sensorless wrench estimation |
| `algorithms/control/` | `computed_torque.h/.cpp` | Feedback-linearisation CTC |
| `algorithms/control/` | `friction_compensator.h/.cpp` | LuGre friction model |
| `algorithms/control/` | `teach_mode.h/.cpp` | Lead-through teaching |
| `algorithms/control/` | `hybrid_force_position.h/.cpp` | Raibert-Craig hybrid control |
| `algorithms/identification/` | `param_identifier.h/.cpp` | WLS parameter identification |
| `algorithms/identification/` | `gravity_calibrator.h/.cpp` | Multi-pose gravity calibration |
| `algorithms/learning/` | `dmp.h/.cpp` | Dynamic Movement Primitives |
| `algorithms/safety/` | `safety_monitor.h/.cpp` | ISO 10218/15066 safety monitor |
| `algorithms/safety/` | `robot_state_machine.h/.cpp` | 12-state FSM |
| `algorithms/safety/` | `telemetry_streamer.h/.cpp` | 10-channel telemetry |

### Modified Firmware Files
- `Robot/instances/devclaw_robot.h` — **Extensively rewritten** from original `dummy_robot.h`: added 7 phases of control, 43 protocol commands (original had basic motion only)
- `Robot/instances/devclaw_robot.cpp` — **Extensively rewritten** from original `dummy_robot.cpp`: ~1977 lines of new control logic
- `UserApp/common_inc.h` — Modified include path
- `UserApp/main.cpp` — Modified class name references
- `UserApp/protocols/*.cpp` — Modified extern declarations

### New Documentation (entirely DevClaw)
- `README.md` — Completely rewritten (original README not included; see original repo)
- `5.Docs/API_Reference.md` — New: full 43-command API reference
- `ATTRIBUTION.md` — This file
- `LICENSE` — MIT license with dual attribution

### New Feishu Bot Integration (entirely DevClaw)
- `3.Software/FeishuBot/feishu_bridge.py` — New: Flask ↔ Serial bridge server
- `3.Software/FeishuBot/feishu_protocol.md` — New: command mapping reference
- `3.Software/FeishuBot/feishu_config.json` — New: bot configuration
- `3.Software/FeishuBot/requirements.txt` — New: Python dependencies

### New Test Suite (entirely DevClaw)
- `2.Firmware/Core-STM32F4-fw/Robot/tests/test_algorithms.cpp` — New: algorithm validation tests

---

## License

Both the original Dummy Robot project and the DevClaw Robot additions are released under the **MIT License**. See `LICENSE` for details.

When using or redistributing this project, please:
1. Retain the original copyright notice for 稚晖君's work
2. Include this `ATTRIBUTION.md` file
3. Do not misrepresent the origin of the original hardware/firmware design

---

## Acknowledgments

We deeply appreciate the open-source robotics community and especially:

- **稚晖君 (peng-zhihui)** — For the visionary Dummy Robot project that inspired and enabled DevClaw
- **unlir** — For the XDrive stepper motor driver reference design
- **ODrive Robotics** — For the Fibre communication protocol framework
- **U8G2** — For the OLED graphics library
