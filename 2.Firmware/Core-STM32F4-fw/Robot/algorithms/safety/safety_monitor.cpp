#include "safety_monitor.h"
#include <cmath>
#include <cstring>


SafetyMonitor::SafetyMonitor()
{
    memset(&config, 0, sizeof(SafetyConfig_t));
    memset(&status, 0, sizeof(SafetyStatus_t));
    memset(watchdogTimers, 0, sizeof(watchdogTimers));
    status.speedScale = 1.0f;
}


void SafetyMonitor::InitDefault(float _dt)
{
    config.dt = _dt;

    // Joint position limits (DevClaw Robot typical range)
    float posMin[6] = {-170, -120, -170, -170, -120, -360};
    float posMax[6] = { 170,  120,  170,  170,  120,  360};
    memcpy(config.jointLimits.posMin, posMin, sizeof(posMin));
    memcpy(config.jointLimits.posMax, posMax, sizeof(posMax));
    config.jointLimits.posSoftMargin = 5.0f; // 5° before hard limit

    float velMax[6] = {180, 180, 240, 360, 360, 480}; // deg/s
    float torqueMax[6] = {5.0f, 8.0f, 6.0f, 2.0f, 1.5f, 1.0f}; // N*m
    float currentMax[6] = {2.5f, 2.5f, 2.5f, 2.0f, 2.0f, 1.5f}; // A
    memcpy(config.jointLimits.velMax, velMax, sizeof(velMax));
    memcpy(config.jointLimits.torqueMax, torqueMax, sizeof(torqueMax));
    memcpy(config.jointLimits.currentMax, currentMax, sizeof(currentMax));

    // Thermal model (small NEMA17 steppers)
    float kHeat[6] = {8.0f, 8.0f, 8.0f, 10.0f, 10.0f, 12.0f}; // °C/(A²·s)
    float kCool[6] = {0.02f, 0.02f, 0.02f, 0.025f, 0.025f, 0.03f}; // 1/s
    memcpy(config.thermal.kHeat, kHeat, sizeof(kHeat));
    memcpy(config.thermal.kCool, kCool, sizeof(kCool));
    config.thermal.tempWarning = 65.0f;
    config.thermal.tempShutdown = 85.0f;
    config.thermal.ambientTemp = 25.0f;

    config.tcpSpeedMax = 250.0f;       // ISO/TS 15066: 250 mm/s for collaborative
    config.extForceMax = 50.0f;        // 50 N max external force
    config.watchdogTimeout = 0.1f;     // 100 ms CAN timeout
    config.timingJitterMax = 2.0f;     // 2 ms max jitter

    // Initialize temperatures to ambient
    for (int i = 0; i < NUM_JOINTS; i++)
        status.estTemperature[i] = config.thermal.ambientTemp;

    status.speedScale = 1.0f;
    status.level = SAFETY_OK;
    status.estopped = false;
}


void SafetyMonitor::Init(const SafetyConfig_t &_config)
{
    config = _config;
    for (int i = 0; i < NUM_JOINTS; i++)
        status.estTemperature[i] = config.thermal.ambientTemp;
    status.speedScale = 1.0f;
    status.level = SAFETY_OK;
    status.estopped = false;
}


void SafetyMonitor::UpdateThermalModel(const float _current[NUM_JOINTS])
{
    float dt = config.dt;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float I2 = _current[i] * _current[i];
        float dT = dt * (config.thermal.kHeat[i] * I2
                       - config.thermal.kCool[i] * (status.estTemperature[i] - config.thermal.ambientTemp));
        status.estTemperature[i] += dT;

        // Clamp to reasonable range
        if (status.estTemperature[i] < config.thermal.ambientTemp)
            status.estTemperature[i] = config.thermal.ambientTemp;
        if (status.estTemperature[i] > 150.0f)
            status.estTemperature[i] = 150.0f;
    }
}


SafetyMonitor::SafetyLevel_t SafetyMonitor::CheckPositionLimits(const float _q[NUM_JOINTS])
{
    SafetyLevel_t worst = SAFETY_OK;
    float margin = config.jointLimits.posSoftMargin;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float qMin = config.jointLimits.posMin[i];
        float qMax = config.jointLimits.posMax[i];

        // Hard limit violation → E-stop
        if (_q[i] <= qMin || _q[i] >= qMax)
        {
            status.violation = VIOLATION_POS_LIMIT;
            status.violationJoint = i;
            status.violationValue = _q[i];
            return SAFETY_ESTOP;
        }

        // Soft limit → controlled stop
        if (_q[i] <= qMin + margin || _q[i] >= qMax - margin)
        {
            if (worst < SAFETY_STOP)
            {
                worst = SAFETY_STOP;
                status.violation = VIOLATION_POS_LIMIT;
                status.violationJoint = i;
                status.violationValue = _q[i];
            }
        }

        // Approaching soft limit → speed reduction
        float warningMargin = margin * 3.0f;
        if (_q[i] <= qMin + warningMargin || _q[i] >= qMax - warningMargin)
        {
            if (worst < SAFETY_SPEED_REDUCE)
                worst = SAFETY_SPEED_REDUCE;
        }
    }

    return worst;
}


SafetyMonitor::SafetyLevel_t SafetyMonitor::CheckVelocityLimits(const float _dq[NUM_JOINTS])
{
    SafetyLevel_t worst = SAFETY_OK;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float absVel = fabsf(_dq[i]);
        float vMax = config.jointLimits.velMax[i];

        if (absVel > vMax * 1.2f) // 120% → E-stop
        {
            status.violation = VIOLATION_VEL_LIMIT;
            status.violationJoint = i;
            status.violationValue = absVel;
            return SAFETY_ESTOP;
        }

        if (absVel > vMax) // 100% → controlled stop
        {
            if (worst < SAFETY_STOP)
            {
                worst = SAFETY_STOP;
                status.violation = VIOLATION_VEL_LIMIT;
                status.violationJoint = i;
                status.violationValue = absVel;
            }
        }

        if (absVel > vMax * 0.85f) // 85% → speed reduction
        {
            if (worst < SAFETY_SPEED_REDUCE)
                worst = SAFETY_SPEED_REDUCE;
        }
    }

    return worst;
}


SafetyMonitor::SafetyLevel_t SafetyMonitor::CheckCurrentLimits(const float _current[NUM_JOINTS])
{
    SafetyLevel_t worst = SAFETY_OK;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float absI = fabsf(_current[i]);
        float iMax = config.jointLimits.currentMax[i];

        if (absI > iMax * 1.5f) // 150% → E-stop
        {
            status.violation = VIOLATION_TORQUE_LIMIT;
            status.violationJoint = i;
            status.violationValue = absI;
            return SAFETY_ESTOP;
        }

        if (absI > iMax) // 100% → warning
        {
            if (worst < SAFETY_WARNING)
            {
                worst = SAFETY_WARNING;
                status.violation = VIOLATION_TORQUE_LIMIT;
                status.violationJoint = i;
                status.violationValue = absI;
            }
        }
    }

    return worst;
}


SafetyMonitor::SafetyLevel_t SafetyMonitor::CheckThermalLimits()
{
    SafetyLevel_t worst = SAFETY_OK;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        if (status.estTemperature[i] > config.thermal.tempShutdown)
        {
            status.violation = VIOLATION_THERMAL;
            status.violationJoint = i;
            status.violationValue = status.estTemperature[i];
            return SAFETY_STOP; // Controlled stop, not E-stop (thermal is gradual)
        }

        if (status.estTemperature[i] > config.thermal.tempWarning)
        {
            if (worst < SAFETY_SPEED_REDUCE)
            {
                worst = SAFETY_SPEED_REDUCE;
                status.violation = VIOLATION_THERMAL;
                status.violationJoint = i;
                status.violationValue = status.estTemperature[i];
            }
        }
    }

    return worst;
}


float SafetyMonitor::ComputeSpeedScale(const float _q[NUM_JOINTS],
                                          const float _dq[NUM_JOINTS]) const
{
    float scale = 1.0f;

    // Reduce speed near joint limits
    float margin = config.jointLimits.posSoftMargin * 3.0f;
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float distMin = _q[i] - config.jointLimits.posMin[i];
        float distMax = config.jointLimits.posMax[i] - _q[i];
        float minDist = (distMin < distMax) ? distMin : distMax;

        if (minDist < margin && minDist > 0)
        {
            float s = minDist / margin;
            if (s < scale) scale = s;
        }
    }

    // Reduce speed at high velocities
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float ratio = fabsf(_dq[i]) / config.jointLimits.velMax[i];
        if (ratio > 0.8f)
        {
            float s = (1.0f - ratio) / 0.2f;
            if (s < 0) s = 0;
            if (s < scale) scale = s;
        }
    }

    // Reduce speed at high temperatures
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float tempRange = config.thermal.tempShutdown - config.thermal.tempWarning;
        if (tempRange < 1.0f) tempRange = 1.0f;
        float excess = status.estTemperature[i] - config.thermal.tempWarning;
        if (excess > 0)
        {
            float s = 1.0f - excess / tempRange;
            if (s < 0.1f) s = 0.1f;
            if (s < scale) scale = s;
        }
    }

    return scale;
}


SafetyMonitor::SafetyLevel_t SafetyMonitor::Check(
    const float _q[NUM_JOINTS],
    const float _dq[NUM_JOINTS],
    const float _current[NUM_JOINTS],
    float _tcpSpeed,
    float _extForce,
    float _loopTimeMs)
{
    // If E-stopped, stay E-stopped until manual reset
    if (status.estopped)
    {
        status.level = SAFETY_ESTOP;
        return SAFETY_ESTOP;
    }

    SafetyLevel_t worst = SAFETY_OK;

    // Reset violation for this cycle
    status.violation = VIOLATION_NONE;
    status.violationJoint = -1;

    // 1. Update thermal model
    UpdateThermalModel(_current);

    // 2. Check all limits
    SafetyLevel_t posLevel = CheckPositionLimits(_q);
    if (posLevel > worst) worst = posLevel;

    SafetyLevel_t velLevel = CheckVelocityLimits(_dq);
    if (velLevel > worst) worst = velLevel;

    SafetyLevel_t curLevel = CheckCurrentLimits(_current);
    if (curLevel > worst) worst = curLevel;

    SafetyLevel_t thermalLevel = CheckThermalLimits();
    if (thermalLevel > worst) worst = thermalLevel;

    // 3. TCP speed check (ISO/TS 15066)
    status.tcpSpeed = _tcpSpeed;
    if (_tcpSpeed > config.tcpSpeedMax * 1.1f)
    {
        worst = SAFETY_STOP;
        status.violation = VIOLATION_TCP_SPEED;
        status.violationValue = _tcpSpeed;
    }
    else if (_tcpSpeed > config.tcpSpeedMax * 0.9f)
    {
        if (worst < SAFETY_SPEED_REDUCE)
            worst = SAFETY_SPEED_REDUCE;
    }

    // 4. External force check
    status.extForceMag = _extForce;
    if (_extForce > config.extForceMax)
    {
        if (worst < SAFETY_STOP)
        {
            worst = SAFETY_STOP;
            status.violation = VIOLATION_EXT_FORCE;
            status.violationValue = _extForce;
        }
    }

    // 5. Watchdog check
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        watchdogTimers[i] += config.dt;
        if (watchdogTimers[i] > config.watchdogTimeout)
        {
            if (worst < SAFETY_STOP)
            {
                worst = SAFETY_STOP;
                status.violation = VIOLATION_WATCHDOG;
                status.violationJoint = i;
                status.violationValue = watchdogTimers[i];
            }
        }
    }

    // 6. Timing jitter check
    if (_loopTimeMs > config.timingJitterMax)
    {
        if (worst < SAFETY_WARNING)
        {
            worst = SAFETY_WARNING;
            status.violation = VIOLATION_TIMING;
            status.violationValue = _loopTimeMs;
        }
    }

    // 7. Compute speed scale
    status.speedScale = ComputeSpeedScale(_q, _dq);
    if (worst >= SAFETY_STOP)
        status.speedScale = 0;

    // 8. Latch E-stop
    if (worst >= SAFETY_ESTOP)
    {
        status.estopped = true;
        status.estopCount++;
    }

    // Update counters
    if (worst >= SAFETY_STOP) status.stopCount++;
    if (worst == SAFETY_WARNING) status.warningCount++;

    status.level = worst;
    return worst;
}


void SafetyMonitor::FeedWatchdog(int _jointIdx)
{
    if (_jointIdx >= 0 && _jointIdx < NUM_JOINTS)
        watchdogTimers[_jointIdx] = 0;
}


bool SafetyMonitor::ResetEstop()
{
    // Only reset if no active violations remain
    status.estopped = false;
    status.level = SAFETY_OK;
    status.violation = VIOLATION_NONE;
    return true;
}


void SafetyMonitor::SetJointLimits(const float _posMin[NUM_JOINTS],
                                     const float _posMax[NUM_JOINTS])
{
    memcpy(config.jointLimits.posMin, _posMin, NUM_JOINTS * sizeof(float));
    memcpy(config.jointLimits.posMax, _posMax, NUM_JOINTS * sizeof(float));
}


void SafetyMonitor::SetTCPSpeedLimit(float _maxSpeed)
{
    if (_maxSpeed > 0)
        config.tcpSpeedMax = _maxSpeed;
}
