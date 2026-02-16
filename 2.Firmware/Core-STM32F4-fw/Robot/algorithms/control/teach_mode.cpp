#include "teach_mode.h"
#include <cmath>
#include <cstring>


TeachMode::TeachMode()
{
    memset(&config, 0, sizeof(Config_t));
    state = TEACH_IDLE;
    teachTimer = 0;
    recordTimer = 0;
    waypointCount = 0;
    trajCount = 0;
    prevVelMag = 0;
    releaseTimer = 0;
    isReleased = false;
}


void TeachMode::InitDefault(float _dt)
{
    config.dt = _dt;

    // Virtual damping: just enough to prevent drift, not too much to feel heavy
    float defaultDamping[NUM_JOINTS] = {0.5f, 0.8f, 0.6f, 0.3f, 0.2f, 0.1f};
    memcpy(config.virtualDamping, defaultDamping, sizeof(defaultDamping));

    // Velocity limits in teach mode (safety)
    float defaultVelLim[NUM_JOINTS] = {45.0f, 45.0f, 60.0f, 90.0f, 90.0f, 120.0f};
    memcpy(config.velocityLimit, defaultVelLim, sizeof(defaultVelLim));

    config.wallStiffness = 5.0f;     // N*m/deg at virtual wall
    config.wallMargin = 10.0f;       // 10 degrees from limit
    config.brakeDamping = 3.0f;      // High damping when released
    config.releaseThreshold = 0.15f; // N*m threshold for "released" detection
    config.recordRate = 100.0f;      // 100 Hz recording rate

    ClearRecording();
}


void TeachMode::Init(const Config_t &_config)
{
    config = _config;
    ClearRecording();
}


void TeachMode::Start()
{
    state = TEACH_ACTIVE;
    teachTimer = 0;
    isReleased = false;
    releaseTimer = 0;
}


void TeachMode::Stop()
{
    state = TEACH_IDLE;
}


void TeachMode::Pause()
{
    if (state == TEACH_ACTIVE || state == TEACH_RECORDING)
        state = TEACH_PAUSED;
}


void TeachMode::Resume()
{
    if (state == TEACH_PAUSED)
        state = TEACH_ACTIVE;
}


void TeachMode::StartRecording()
{
    if (state == TEACH_ACTIVE)
    {
        state = TEACH_RECORDING;
        trajCount = 0;
        recordTimer = 0;
    }
}


int TeachMode::StopRecording()
{
    if (state == TEACH_RECORDING)
        state = TEACH_ACTIVE;
    return trajCount;
}


bool TeachMode::SaveWaypoint(const float _joints[NUM_JOINTS])
{
    if (waypointCount >= MAX_WAYPOINTS) return false;

    Waypoint_t &wp = waypoints[waypointCount];
    memcpy(wp.joints, _joints, NUM_JOINTS * sizeof(float));
    wp.timestamp = teachTimer;
    waypointCount++;
    return true;
}


void TeachMode::ClearRecording()
{
    waypointCount = 0;
    trajCount = 0;
}


float TeachMode::ComputeWallTorque(float _q, float _qMin, float _qMax) const
{
    float tau = 0;

    // Repulsive force near lower limit
    float distMin = _q - _qMin;
    if (distMin < config.wallMargin && distMin > 0)
    {
        float penetration = config.wallMargin - distMin;
        tau += config.wallStiffness * penetration;
    }
    else if (distMin <= 0)
    {
        tau += config.wallStiffness * config.wallMargin * 2.0f; // Hard stop
    }

    // Repulsive force near upper limit
    float distMax = _qMax - _q;
    if (distMax < config.wallMargin && distMax > 0)
    {
        float penetration = config.wallMargin - distMax;
        tau -= config.wallStiffness * penetration;
    }
    else if (distMax <= 0)
    {
        tau -= config.wallStiffness * config.wallMargin * 2.0f;
    }

    return tau;
}


void TeachMode::Tick(const float _q[NUM_JOINTS],
                      const float _dq[NUM_JOINTS],
                      const float _gravTorque[NUM_JOINTS],
                      const float _frictionComp[NUM_JOINTS],
                      float _tauCmd[NUM_JOINTS])
{
    float dt = config.dt;
    teachTimer += dt;

    if (state == TEACH_IDLE)
    {
        memset(_tauCmd, 0, NUM_JOINTS * sizeof(float));
        return;
    }

    if (state == TEACH_PAUSED)
    {
        // Hold position with high stiffness (like a brake)
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            _tauCmd[i] = _gravTorque[i] - config.brakeDamping * 5.0f * _dq[i] * 0.01745f;
        }
        return;
    }

    // --- TEACH_ACTIVE or TEACH_RECORDING ---

    // Detect if operator has released the arm
    float velMag = 0;
    for (int i = 0; i < NUM_JOINTS; i++)
        velMag += _dq[i] * _dq[i];
    velMag = sqrtf(velMag);

    if (velMag < 1.0f && prevVelMag < 1.0f) // Nearly stationary
    {
        releaseTimer += dt;
        if (releaseTimer > 0.3f) // Stationary for 300ms
            isReleased = true;
    }
    else
    {
        releaseTimer = 0;
        isReleased = false;
    }
    prevVelMag = velMag;

    // Compute control torque
    float DEG2RAD = 0.01745329251994f;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        // 1. Gravity compensation (core of teach mode)
        _tauCmd[i] = _gravTorque[i];

        // 2. Friction compensation (makes arm feel lighter)
        _tauCmd[i] += _frictionComp[i];

        // 3. Virtual damping (stability + auto-brake)
        float damping = config.virtualDamping[i];
        if (isReleased)
            damping = config.brakeDamping; // Increase damping when released

        _tauCmd[i] -= damping * _dq[i] * DEG2RAD;

        // 4. Virtual walls near joint limits
        // _q is in degrees, limits in degrees
        _tauCmd[i] += ComputeWallTorque(_q[i], -170.0f, 170.0f); // Generic limits

        // 5. Velocity limiting (safety clamp)
        if (fabsf(_dq[i]) > config.velocityLimit[i])
        {
            float sign = (_dq[i] > 0) ? -1.0f : 1.0f;
            _tauCmd[i] += sign * config.brakeDamping * 2.0f; // Strong braking
        }
    }

    // Record trajectory if in recording mode
    if (state == TEACH_RECORDING)
    {
        recordTimer += dt;
        float recordInterval = 1.0f / config.recordRate;

        if (recordTimer >= recordInterval && trajCount < MAX_TRAJ_POINTS)
        {
            memcpy(trajBuffer[trajCount], _q, NUM_JOINTS * sizeof(float));
            trajCount++;
            recordTimer -= recordInterval;
        }
    }
}
