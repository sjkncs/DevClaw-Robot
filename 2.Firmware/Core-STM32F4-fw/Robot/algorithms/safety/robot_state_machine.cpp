#include "robot_state_machine.h"
#include <cstring>


RobotStateMachine::RobotStateMachine()
{
    memset(&info, 0, sizeof(StateInfo_t));
    info.current = STATE_POWERED_OFF;
    info.previous = STATE_POWERED_OFF;
    stoppingStartTime = 0;
}


void RobotStateMachine::Init()
{
    info.current = STATE_INITIALIZING;
    info.previous = STATE_POWERED_OFF;
    info.totalTime = 0;
    info.stateEntryTime = 0;
    info.transitionCount = 0;
}


const char* RobotStateMachine::StateName(RobotState_t _state)
{
    switch (_state)
    {
    case STATE_POWERED_OFF:   return "POWERED_OFF";
    case STATE_INITIALIZING:  return "INITIALIZING";
    case STATE_IDLE:          return "IDLE";
    case STATE_POSITION:      return "POSITION";
    case STATE_CTC_TORQUE:    return "CTC_TORQUE";
    case STATE_IMPEDANCE:     return "IMPEDANCE";
    case STATE_TEACH:         return "TEACH";
    case STATE_DMP_EXEC:      return "DMP_EXEC";
    case STATE_HYBRID_FORCE:  return "HYBRID_FORCE";
    case STATE_MINJERK_EXEC:  return "MINJERK_EXEC";
    case STATE_STOPPING:      return "STOPPING";
    case STATE_ESTOP:         return "ESTOP";
    case STATE_FAULT:         return "FAULT";
    default:                  return "UNKNOWN";
    }
}


bool RobotStateMachine::IsInMotion() const
{
    switch (info.current)
    {
    case STATE_POSITION:
    case STATE_CTC_TORQUE:
    case STATE_IMPEDANCE:
    case STATE_TEACH:
    case STATE_DMP_EXEC:
    case STATE_HYBRID_FORCE:
    case STATE_MINJERK_EXEC:
        return true;
    default:
        return false;
    }
}


bool RobotStateMachine::IsTorqueMode() const
{
    switch (info.current)
    {
    case STATE_CTC_TORQUE:
    case STATE_IMPEDANCE:
    case STATE_TEACH:
    case STATE_HYBRID_FORCE:
        return true;
    default:
        return false;
    }
}


bool RobotStateMachine::CheckGuard(RobotState_t _from, RobotState_t _to) const
{
    // E-stop can always be entered
    if (_to == STATE_ESTOP) return true;

    // Cannot leave FAULT
    if (_from == STATE_FAULT) return false;

    // Cannot leave ESTOP except via explicit reset → IDLE
    if (_from == STATE_ESTOP && _to != STATE_IDLE) return false;

    // Motors must be enabled for any motion state
    if (_to >= STATE_POSITION && _to <= STATE_MINJERK_EXEC)
    {
        if (!info.motorsEnabled) return false;
    }

    // Torque modes require dynamics solver
    if (_to == STATE_CTC_TORQUE || _to == STATE_IMPEDANCE ||
        _to == STATE_TEACH || _to == STATE_HYBRID_FORCE)
    {
        if (!info.dynamicsAvailable) return false;
    }

    // Can only enter motion states from IDLE
    if (_to >= STATE_POSITION && _to <= STATE_MINJERK_EXEC)
    {
        if (_from != STATE_IDLE) return false;
    }

    return true;
}


void RobotStateMachine::EnterState(RobotState_t _newState, uint32_t _timeMs)
{
    info.previous = info.current;
    info.current = _newState;
    info.stateEntryTime = _timeMs;
    info.transitionCount++;

    if (_newState == STATE_STOPPING)
        stoppingStartTime = _timeMs;
}


bool RobotStateMachine::ProcessEvent(TransitionEvent_t _event)
{
    RobotState_t cur = info.current;
    RobotState_t next = cur;

    switch (_event)
    {
    case EVT_INIT_COMPLETE:
        if (cur == STATE_INITIALIZING) next = STATE_IDLE;
        break;

    case EVT_ENABLE_MOTORS:
        info.motorsEnabled = true;
        return true;

    case EVT_DISABLE_MOTORS:
        info.motorsEnabled = false;
        if (IsInMotion()) next = STATE_STOPPING;
        break;

    case EVT_REQUEST_POSITION:
        if (cur == STATE_IDLE) next = STATE_POSITION;
        else if (IsInMotion()) { info.requested = STATE_POSITION; next = STATE_STOPPING; }
        break;

    case EVT_REQUEST_CTC:
        if (cur == STATE_IDLE) next = STATE_CTC_TORQUE;
        else if (IsInMotion()) { info.requested = STATE_CTC_TORQUE; next = STATE_STOPPING; }
        break;

    case EVT_REQUEST_IMPEDANCE:
        if (cur == STATE_IDLE) next = STATE_IMPEDANCE;
        else if (IsInMotion()) { info.requested = STATE_IMPEDANCE; next = STATE_STOPPING; }
        break;

    case EVT_REQUEST_TEACH:
        if (cur == STATE_IDLE) next = STATE_TEACH;
        else if (IsInMotion()) { info.requested = STATE_TEACH; next = STATE_STOPPING; }
        break;

    case EVT_REQUEST_DMP:
        if (cur == STATE_IDLE) next = STATE_DMP_EXEC;
        else if (IsInMotion()) { info.requested = STATE_DMP_EXEC; next = STATE_STOPPING; }
        break;

    case EVT_REQUEST_HYBRID:
        if (cur == STATE_IDLE) next = STATE_HYBRID_FORCE;
        else if (IsInMotion()) { info.requested = STATE_HYBRID_FORCE; next = STATE_STOPPING; }
        break;

    case EVT_REQUEST_MINJERK:
        if (cur == STATE_IDLE) next = STATE_MINJERK_EXEC;
        else if (IsInMotion()) { info.requested = STATE_MINJERK_EXEC; next = STATE_STOPPING; }
        break;

    case EVT_MOTION_COMPLETE:
        if (IsInMotion()) next = STATE_IDLE;
        break;

    case EVT_STOP_REQUEST:
        if (IsInMotion()) next = STATE_STOPPING;
        break;

    case EVT_ESTOP:
        next = STATE_ESTOP;
        break;

    case EVT_ESTOP_RESET:
        if (cur == STATE_ESTOP) next = STATE_IDLE;
        break;

    case EVT_FAULT:
        next = STATE_FAULT;
        break;

    case EVT_STOP_COMPLETE:
        if (cur == STATE_STOPPING)
        {
            if (info.requested != STATE_POWERED_OFF && info.requested != cur)
                next = info.requested;
            else
                next = STATE_IDLE;
            info.requested = STATE_POWERED_OFF;
        }
        break;
    }

    if (next != cur)
    {
        if (!CheckGuard(cur, next))
            return false;

        EnterState(next, info.totalTime);
        return true;
    }

    return false;
}


void RobotStateMachine::Tick(uint32_t _timeMs)
{
    info.totalTime = _timeMs;

    // Auto-transition from STOPPING after timeout
    if (info.current == STATE_STOPPING)
    {
        uint32_t elapsed = _timeMs - stoppingStartTime;
        if (elapsed >= STOPPING_TIMEOUT_MS)
        {
            ProcessEvent(EVT_STOP_COMPLETE);
        }
    }
}
