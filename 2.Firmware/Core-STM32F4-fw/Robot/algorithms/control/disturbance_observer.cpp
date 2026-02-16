#include "disturbance_observer.h"
#include <cmath>
#include <cstring>


// ========================== DisturbanceObserver ==========================

DisturbanceObserver::DisturbanceObserver()
{
    memset(&config, 0, sizeof(Config_t));
    disturbanceEst = 0;
    b0 = b1 = b2 = 0;
    a1 = a2 = 0;
    x1_prev = x2_prev = y1_prev = y2_prev = 0;
    x1b_prev = x2b_prev = y1b_prev = y2b_prev = 0;
}


void DisturbanceObserver::Init(const Config_t &_config)
{
    config = _config;
    ComputeFilterCoeffs(config.cutoffFreq, config.sampleFreq);
    Reset();
}


void DisturbanceObserver::Reset()
{
    disturbanceEst = 0;
    x1_prev = x2_prev = y1_prev = y2_prev = 0;
    x1b_prev = x2b_prev = y1b_prev = y2b_prev = 0;
}


void DisturbanceObserver::ComputeFilterCoeffs(float _cutoffHz, float _sampleHz)
{
    // 2nd-order Butterworth low-pass filter design via bilinear transform
    // Analog prototype: H(s) = 1 / (s^2 + sqrt(2)*s + 1)
    // Bilinear: s = 2*fs * (1-z^-1)/(1+z^-1)

    float wc = 2.0f * (float)M_PI * _cutoffHz;
    float T = 1.0f / _sampleHz;

    // Pre-warp
    float wc_d = 2.0f / T * tanf(wc * T / 2.0f);

    float k = wc_d * T / 2.0f;
    float k2 = k * k;
    float sqrt2_k = 1.41421356f * k; // sqrt(2) * k

    float denom = 1.0f + sqrt2_k + k2;
    float inv_denom = 1.0f / denom;

    b0 = k2 * inv_denom;
    b1 = 2.0f * k2 * inv_denom;
    b2 = k2 * inv_denom;
    a1 = 2.0f * (k2 - 1.0f) * inv_denom;
    a2 = (1.0f - sqrt2_k + k2) * inv_denom;
}


void DisturbanceObserver::SetCutoffFrequency(float _freq)
{
    config.cutoffFreq = _freq;
    ComputeFilterCoeffs(_freq, config.sampleFreq);
}


float DisturbanceObserver::ApplyFilter(float _input,
                                        float &_x1, float &_x2,
                                        float &_y1, float &_y2)
{
    // Direct Form II implementation
    float y = b0 * _input + b1 * _x1 + b2 * _x2 - a1 * _y1 - a2 * _y2;

    _x2 = _x1;
    _x1 = _input;
    _y2 = _y1;
    _y1 = y;

    return y;
}


float DisturbanceObserver::Update(float _tauCmd, float _velocity, float _acceleration)
{
    if (!config.enabled)
    {
        disturbanceEst = 0;
        return 0;
    }

    // DOB structure (Ohnishi, 1987):
    //
    // Disturbance = tau_cmd - M_n * ddq - B_n * dq
    //
    // But we can't directly differentiate for ddq (noise amplification).
    // Instead, use the inverse-model + low-pass filter approach:
    //
    // Path A: Q(s) * tau_cmd
    // Path B: Q(s) * M_n * ddq  (using KF-estimated acceleration)
    //
    // d_hat = Q(s) * [tau_cmd - M_n * ddq - B_n * dq]
    //
    // With KF providing clean ddq, we can directly compute:

    float tau_model = config.nominalInertia * _acceleration
                    + config.nominalFriction * _velocity;

    float disturbance_raw = _tauCmd - tau_model;

    // Apply low-pass filter Q(s) to disturbance estimate
    disturbanceEst = ApplyFilter(disturbance_raw,
                                  x1_prev, x2_prev, y1_prev, y2_prev);

    // Safety clamp
    if (disturbanceEst > config.disturbanceLimit)
        disturbanceEst = config.disturbanceLimit;
    else if (disturbanceEst < -config.disturbanceLimit)
        disturbanceEst = -config.disturbanceLimit;

    return disturbanceEst;
}


// ========================== MultiJointDOB ==========================

void MultiJointDOB::Init(float _cutoffFreq, float _sampleFreq,
                          const float _inertias[NUM_JOINTS])
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        DisturbanceObserver::Config_t cfg;
        cfg.cutoffFreq = _cutoffFreq;
        cfg.sampleFreq = _sampleFreq;
        cfg.nominalInertia = _inertias[i];
        cfg.nominalFriction = 0.01f; // Default, should be identified
        cfg.disturbanceLimit = 5.0f; // N*m safety limit
        cfg.enabled = true;
        jointDOB[i].Init(cfg);
    }
}


void MultiJointDOB::Reset()
{
    for (int i = 0; i < NUM_JOINTS; i++)
        jointDOB[i].Reset();
}


void MultiJointDOB::Update(const float _tauCmd[NUM_JOINTS],
                             const float _velocity[NUM_JOINTS],
                             const float _acceleration[NUM_JOINTS],
                             float _tauComp[NUM_JOINTS])
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        jointDOB[i].Update(_tauCmd[i], _velocity[i], _acceleration[i]);
        _tauComp[i] = jointDOB[i].GetCompensation();
    }
}


void MultiJointDOB::GetDisturbances(float _dist[NUM_JOINTS]) const
{
    for (int i = 0; i < NUM_JOINTS; i++)
        _dist[i] = jointDOB[i].GetDisturbanceEstimate();
}


void MultiJointDOB::SetEnabled(bool _enable)
{
    for (int i = 0; i < NUM_JOINTS; i++)
        jointDOB[i].SetEnabled(_enable);
}
