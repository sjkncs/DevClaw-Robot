#include "hybrid_force_position.h"
#include <cmath>
#include <cstring>


HybridForcePositionController::HybridForcePositionController()
{
    memset(&config, 0, sizeof(Config_t));
    memset(&ref, 0, sizeof(Reference_t));
    memset(&state, 0, sizeof(State_t));
    spiralAngle = 0;
    spiralRadius = 0;
    prevForceZ = 0;
}


void HybridForcePositionController::InitDefault(float _dt)
{
    config.dt = _dt;

    // Default: position control on all axes
    for (int i = 0; i < DOF; i++)
        config.selectionMatrix[i] = 0;

    // Force control gains
    float defaultKfp[DOF] = {0.005f, 0.005f, 0.005f, 0.01f, 0.01f, 0.01f};
    float defaultKfi[DOF] = {0.02f, 0.02f, 0.02f, 0.05f, 0.05f, 0.05f};
    memcpy(config.Kf_p, defaultKfp, sizeof(defaultKfp));
    memcpy(config.Kf_i, defaultKfi, sizeof(defaultKfi));
    config.forceIntegralLimit = 20.0f;

    // Position control gains
    float defaultKxp[DOF] = {500.0f, 500.0f, 500.0f, 50.0f, 50.0f, 50.0f};
    float defaultKxd[DOF] = {30.0f, 30.0f, 30.0f, 5.0f, 5.0f, 5.0f};
    memcpy(config.Kx_p, defaultKxp, sizeof(defaultKxp));
    memcpy(config.Kx_d, defaultKxd, sizeof(defaultKxd));

    config.maxForce = 30.0f;    // 30 N max
    config.maxTorque = 5.0f;    // 5 N*m max

    Reset();
}


void HybridForcePositionController::Init(const Config_t &_config)
{
    config = _config;
    Reset();
}


void HybridForcePositionController::SetSelectionMatrix(const float _sel[DOF])
{
    memcpy(config.selectionMatrix, _sel, DOF * sizeof(float));
}


void HybridForcePositionController::SetForceReference(const float _forceRef[DOF])
{
    memcpy(ref.forceRef, _forceRef, DOF * sizeof(float));
}


void HybridForcePositionController::SetPositionReference(const float _posRef[DOF])
{
    memcpy(ref.posRef, _posRef, DOF * sizeof(float));
}


void HybridForcePositionController::Reset()
{
    memset(&state, 0, sizeof(State_t));
    spiralAngle = 0;
    spiralRadius = 0;
    prevForceZ = 0;
}


void HybridForcePositionController::Compute(
    const float _posCurrent[DOF],
    const float _velCurrent[DOF],
    const float _forceMeas[DOF],
    const float _jacobian[36],
    const float _gravTorque[DOF],
    float _tauOut[DOF])
{
    float dt = config.dt;
    float F_cmd[DOF];

    for (int i = 0; i < DOF; i++)
    {
        float s = config.selectionMatrix[i];

        // Force-controlled component
        state.forceError[i] = ref.forceRef[i] - _forceMeas[i];
        state.forceIntegral[i] += state.forceError[i] * dt;

        // Anti-windup
        if (state.forceIntegral[i] > config.forceIntegralLimit)
            state.forceIntegral[i] = config.forceIntegralLimit;
        else if (state.forceIntegral[i] < -config.forceIntegralLimit)
            state.forceIntegral[i] = -config.forceIntegralLimit;

        float forceCtrl = config.Kf_p[i] * state.forceError[i]
                        + config.Kf_i[i] * state.forceIntegral[i];

        // Position-controlled component
        state.posError[i] = ref.posRef[i] - _posCurrent[i];
        float velError = ref.velRef[i] - _velCurrent[i];

        float posCtrl = config.Kx_p[i] * state.posError[i]
                      + config.Kx_d[i] * velError;

        // Hybrid: blend force and position based on selection matrix
        F_cmd[i] = s * forceCtrl + (1.0f - s) * posCtrl;

        // Safety clamp
        float limit = (i < 3) ? config.maxForce : config.maxTorque;
        if (F_cmd[i] > limit) F_cmd[i] = limit;
        else if (F_cmd[i] < -limit) F_cmd[i] = -limit;

        state.cmdForce[i] = F_cmd[i];
    }

    // Map to joint space: tau = J^T * F_cmd + g(q)
    for (int j = 0; j < DOF; j++)
    {
        _tauOut[j] = _gravTorque[j];
        for (int i = 0; i < DOF; i++)
            _tauOut[j] += _jacobian[i * DOF + j] * F_cmd[i]; // J^T * F

        state.cmdTorque[j] = _tauOut[j];
    }
}


bool HybridForcePositionController::SpiralSearchTick(
    const SpiralConfig_t &_spiralCfg,
    const float _forceMeas[DOF],
    float _xyOffset[2])
{
    float dt = config.dt;

    // Expand spiral
    spiralAngle += _spiralCfg.angularSpeed * dt;
    spiralRadius += _spiralCfg.radiusRate * dt / (2.0f * 3.14159265f);

    if (spiralRadius > _spiralCfg.maxRadius)
        spiralRadius = _spiralCfg.maxRadius;

    // XY offset = spiral pattern
    _xyOffset[0] = spiralRadius * cosf(spiralAngle);
    _xyOffset[1] = spiralRadius * sinf(spiralAngle);

    // Detect hole: sudden drop in Z force while maintaining insertion pressure
    float forceZ = _forceMeas[2];
    float forceDrop = prevForceZ - forceZ;
    prevForceZ = forceZ;

    // Hole found when:
    // 1. We were pressing against surface (prevForceZ > threshold)
    // 2. Force suddenly dropped (peg entered hole)
    if (forceDrop > _spiralCfg.contactThreshold * 0.5f &&
        forceZ < _spiralCfg.contactThreshold * 0.3f)
    {
        // Reset spiral for next use
        spiralAngle = 0;
        spiralRadius = 0;
        return true;
    }

    return false;
}
