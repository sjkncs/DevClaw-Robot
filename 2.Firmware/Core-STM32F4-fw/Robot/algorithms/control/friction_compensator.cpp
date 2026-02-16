#include "friction_compensator.h"
#include <cmath>
#include <cstring>


FrictionCompensator::FrictionCompensator()
{
    memset(&config, 0, sizeof(Config_t));
    memset(&state, 0, sizeof(State_t));
}


float FrictionCompensator::StribeckFunction(float _v, const LuGreParams_t &_p)
{
    // g(v) = Fc + (Fs - Fc) * exp(-(v/vs)^2)
    float ratio = _v / (_p.vs + 1e-6f);
    return _p.Fc + (_p.Fs - _p.Fc) * expf(-ratio * ratio);
}


void FrictionCompensator::InitDefault(float _dt)
{
    config.dt = _dt;
    config.adaptiveEnabled = false;
    config.adaptationRate = 0.001f;

    // Default LuGre parameters per joint (tuned for small stepper + harmonic drive)
    // Joints 1-3 (large): higher friction
    // Joints 4-6 (small): lower friction
    LuGreParams_t defaultParams[6] = {
        {1000.0f, 2.0f, 0.010f, 0.08f, 0.12f, 0.5f},  // J1
        {1000.0f, 2.5f, 0.015f, 0.10f, 0.15f, 0.4f},  // J2
        {1000.0f, 2.0f, 0.012f, 0.08f, 0.12f, 0.5f},  // J3
        { 800.0f, 1.5f, 0.008f, 0.05f, 0.08f, 0.6f},  // J4
        { 800.0f, 1.5f, 0.006f, 0.04f, 0.06f, 0.6f},  // J5
        { 600.0f, 1.0f, 0.005f, 0.03f, 0.05f, 0.8f},  // J6
    };
    memcpy(config.params, defaultParams, sizeof(defaultParams));

    Reset();
}


void FrictionCompensator::Init(const Config_t &_config)
{
    config = _config;
    Reset();
}


void FrictionCompensator::SetJointParams(int _joint, const LuGreParams_t &_params)
{
    if (_joint < 0 || _joint >= NUM_JOINTS) return;
    config.params[_joint] = _params;
}


void FrictionCompensator::SetFromIdentification(const float _fv[NUM_JOINTS],
                                                  const float _fc[NUM_JOINTS])
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        config.params[i].sigma2 = _fv[i];
        config.params[i].Fc = fabsf(_fc[i]);
        config.params[i].Fs = fabsf(_fc[i]) * 1.5f; // Static = 1.5x Coulomb
    }
}


void FrictionCompensator::Reset()
{
    memset(&state, 0, sizeof(State_t));
}


void FrictionCompensator::Compensate(const float _dq[NUM_JOINTS],
                                       float _tauComp[NUM_JOINTS])
{
    float dt = config.dt;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float v = _dq[i];
        const LuGreParams_t &p = config.params[i];

        // Stribeck function
        float gv = StribeckFunction(v, p);
        state.stribeckFn[i] = gv;

        // Bristle dynamics: dz/dt = v - sigma0 * |v| / g(v) * z
        float absV = fabsf(v);
        float dzdt = v - p.sigma0 * absV / (gv + 1e-10f) * state.z[i];

        // Integrate bristle state (semi-implicit for stability)
        state.z[i] += dzdt * dt;

        // Clamp bristle deflection to prevent divergence
        float zMax = gv / (p.sigma0 + 1e-6f) * 1.5f;
        if (state.z[i] > zMax) state.z[i] = zMax;
        else if (state.z[i] < -zMax) state.z[i] = -zMax;

        state.dz[i] = dzdt;

        // Total friction: F = sigma0*z + sigma1*dz/dt + sigma2*v
        state.tauFriction[i] = p.sigma0 * state.z[i]
                              + p.sigma1 * dzdt
                              + p.sigma2 * v;

        // Compensation = negative friction
        state.tauComp[i] = -state.tauFriction[i];
        _tauComp[i] = state.tauComp[i];
    }
}


void FrictionCompensator::Update(const float _dq[NUM_JOINTS],
                                   const float _tauResidual[NUM_JOINTS],
                                   float _tauComp[NUM_JOINTS])
{
    // First compute compensation
    Compensate(_dq, _tauComp);

    // Then optionally adapt parameters
    if (config.adaptiveEnabled)
    {
        for (int i = 0; i < NUM_JOINTS; i++)
            AdaptParameters(i, _dq[i], _tauResidual[i]);
    }
}


void FrictionCompensator::AdaptParameters(int _joint, float _velocity,
                                            float _residual)
{
    // Gradient descent on friction parameters to minimize residual
    // residual = tau_actual - tau_predicted (including friction)
    // If residual > 0: we're under-compensating friction
    // If residual < 0: we're over-compensating

    float lr = config.adaptationRate;
    LuGreParams_t &p = config.params[_joint];
    float absV = fabsf(_velocity);

    // Only adapt when moving (avoid stiction adaptation issues)
    if (absV < 0.1f) return;

    // Adapt Coulomb friction
    float signV = (_velocity > 0) ? 1.0f : -1.0f;
    p.Fc += lr * _residual * signV;
    if (p.Fc < 0.001f) p.Fc = 0.001f;
    if (p.Fc > 2.0f) p.Fc = 2.0f;

    // Adapt viscous friction
    p.sigma2 += lr * _residual * _velocity * 0.1f;
    if (p.sigma2 < 0.0001f) p.sigma2 = 0.0001f;
    if (p.sigma2 > 1.0f) p.sigma2 = 1.0f;

    // Keep Fs >= Fc
    if (p.Fs < p.Fc * 1.1f) p.Fs = p.Fc * 1.1f;
}


void FrictionCompensator::GetFrictionTorque(float _tauFriction[NUM_JOINTS]) const
{
    memcpy(_tauFriction, state.tauFriction, NUM_JOINTS * sizeof(float));
}
