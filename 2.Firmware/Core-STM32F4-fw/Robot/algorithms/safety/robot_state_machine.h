#ifndef ROBOT_STATE_MACHINE_H
#define ROBOT_STATE_MACHINE_H

#include <cstdint>

/**
 * @brief Hierarchical Robot State Machine
 *
 * Manages all operating modes of the robot, ensuring safe transitions
 * and preventing conflicting control modes from running simultaneously.
 *
 * === State Hierarchy ===
 *
 *   POWERED_OFF ──► INITIALIZING ──► IDLE
 *                                     │
 *         ┌───────────┬───────────┬───┴───┬──────────┬──────────┐
 *         ▼           ▼           ▼       ▼          ▼          ▼
 *     POSITION    CTC_TORQUE   IMPEDANCE TEACH    DMP_EXEC   HYBRID_FORCE
 *         │           │           │       │          │          │
 *         └───────────┴───────────┴───┬───┴──────────┴──────────┘
 *                                     ▼
 *                                  STOPPING ──► IDLE
 *                                     │
 *                                     ▼
 *                                  ESTOP (latched)
 *
 * === Transition Rules ===
 *   - Only one active control mode at a time
 *   - Transitions go through STOPPING state (controlled deceleration)
 *   - ESTOP can be entered from any state
 *   - ESTOP requires explicit reset → returns to IDLE
 *   - TEACH mode disables position tracking
 *   - CTC/IMPEDANCE/HYBRID require dynamics solver
 *
 * === Guard Conditions ===
 *   - Motors must be enabled before leaving IDLE
 *   - Safety monitor must report OK for mode transitions
 *   - Homing must be complete for absolute position modes
 */
class RobotStateMachine
{
public:
    enum RobotState_t
    {
        STATE_POWERED_OFF = 0,
        STATE_INITIALIZING,
        STATE_IDLE,            // Motors enabled, no active control
        STATE_POSITION,        // Standard position control (DCE)
        STATE_CTC_TORQUE,      // Computed torque control
        STATE_IMPEDANCE,       // Impedance/admittance control
        STATE_TEACH,           // Lead-through teaching
        STATE_DMP_EXEC,        // DMP trajectory execution
        STATE_HYBRID_FORCE,    // Force/position hybrid
        STATE_MINJERK_EXEC,    // Minimum-jerk trajectory execution
        STATE_STOPPING,        // Controlled deceleration
        STATE_ESTOP,           // Emergency stop (latched)
        STATE_FAULT            // Non-recoverable fault
    };

    enum TransitionEvent_t
    {
        EVT_INIT_COMPLETE,
        EVT_ENABLE_MOTORS,
        EVT_DISABLE_MOTORS,
        EVT_REQUEST_POSITION,
        EVT_REQUEST_CTC,
        EVT_REQUEST_IMPEDANCE,
        EVT_REQUEST_TEACH,
        EVT_REQUEST_DMP,
        EVT_REQUEST_HYBRID,
        EVT_REQUEST_MINJERK,
        EVT_MOTION_COMPLETE,
        EVT_STOP_REQUEST,
        EVT_ESTOP,
        EVT_ESTOP_RESET,
        EVT_FAULT,
        EVT_STOP_COMPLETE
    };

    struct StateInfo_t
    {
        RobotState_t current;
        RobotState_t previous;
        RobotState_t requested;       // Pending mode after STOPPING
        uint32_t stateEntryTime;      // ms since entering current state
        uint32_t totalTime;           // Total uptime (ms)
        bool motorsEnabled;
        bool homingComplete;
        bool dynamicsAvailable;
        uint32_t transitionCount;
    };

    RobotStateMachine();

    /**
     * @brief Initialize state machine
     */
    void Init();

    /**
     * @brief Process a transition event
     * @return true if transition was accepted
     */
    bool ProcessEvent(TransitionEvent_t _event);

    /**
     * @brief Tick the state machine (call every control cycle)
     * @param _timeMs Current time in ms
     */
    void Tick(uint32_t _timeMs);

    /**
     * @brief Get current state
     */
    RobotState_t GetState() const { return info.current; }

    /**
     * @brief Get full state info
     */
    const StateInfo_t &GetInfo() const { return info; }

    /**
     * @brief Check if a specific control mode is active
     */
    bool IsActive(RobotState_t _state) const { return info.current == _state; }

    /**
     * @brief Check if robot is in any motion state
     */
    bool IsInMotion() const;

    /**
     * @brief Check if torque control is active (CTC/impedance/teach/hybrid)
     */
    bool IsTorqueMode() const;

    /**
     * @brief Set flags
     */
    void SetHomingComplete(bool _done) { info.homingComplete = _done; }
    void SetDynamicsAvailable(bool _avail) { info.dynamicsAvailable = _avail; }

    /**
     * @brief Get state name string (for telemetry/debug)
     */
    static const char* StateName(RobotState_t _state);

private:
    StateInfo_t info;
    uint32_t stoppingStartTime;
    static constexpr uint32_t STOPPING_TIMEOUT_MS = 2000; // 2s max stopping time

    /**
     * @brief Check guard conditions for a transition
     */
    bool CheckGuard(RobotState_t _from, RobotState_t _to) const;

    /**
     * @brief Enter a new state (execute entry actions)
     */
    void EnterState(RobotState_t _newState, uint32_t _timeMs);
};

#endif // ROBOT_STATE_MACHINE_H
