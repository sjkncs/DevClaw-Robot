#include "computed_torque.h"
#include <cmath>
#include <cstring>


ComputedTorqueController::ComputedTorqueController()
{
    memset(&gains, 0, sizeof(Gains_t));
    memset(&state, 0, sizeof(State_t));
    dt = 0.001f;
}


void ComputedTorqueController::Init(float _dt, float _naturalFreq)
{
    dt = _dt;
    SetNaturalFrequency(_naturalFreq);
    gains.integralLimit = 10.0f; // rad*s max integral
    Reset();
}


void ComputedTorqueController::SetGains(const Gains_t &_gains)
{
    memcpy(&gains, &_gains, sizeof(Gains_t));
}


void ComputedTorqueController::SetNaturalFrequency(float _wn)
{
    // Critically damped: Kp = wn^2, Kv = 2*wn
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        gains.Kp[i] = _wn * _wn;
        gains.Kv[i] = 2.0f * _wn;
        gains.Ki[i] = _wn * 0.1f; // Small integral for steady-state
    }
    gains.integralLimit = 10.0f;
}


void ComputedTorqueController::SetPerJointFrequency(const float _wn[NUM_JOINTS])
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        gains.Kp[i] = _wn[i] * _wn[i];
        gains.Kv[i] = 2.0f * _wn[i];
        gains.Ki[i] = _wn[i] * 0.1f;
    }
    gains.integralLimit = 10.0f;
}


void ComputedTorqueController::Reset()
{
    memset(&state, 0, sizeof(State_t));
}


void ComputedTorqueController::MatVec6(const float *A, const float *x, float *y)
{
    for (int i = 0; i < 6; i++)
    {
        y[i] = 0;
        for (int j = 0; j < 6; j++)
            y[i] += A[i * 6 + j] * x[j];
    }
}


void ComputedTorqueController::Compute(const Reference_t &_ref,
                                         const float _q[NUM_JOINTS],
                                         const float _dq[NUM_JOINTS],
                                         const float _massMatrix[36],
                                         const float _hTorque[NUM_JOINTS],
                                         const float *_tauDOB,
                                         float _tauOut[NUM_JOINTS])
{
    // Outer loop: u = ddq_d + Kv*(dq_d - dq) + Kp*(q_d - q) + Ki*integral(e)
    float u[NUM_JOINTS];

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        state.posError[i] = _ref.q_d[i] - _q[i];
        state.velError[i] = _ref.dq_d[i] - _dq[i];

        // Integral with anti-windup
        state.integralError[i] += state.posError[i] * dt;
        if (state.integralError[i] > gains.integralLimit)
            state.integralError[i] = gains.integralLimit;
        else if (state.integralError[i] < -gains.integralLimit)
            state.integralError[i] = -gains.integralLimit;

        u[i] = _ref.ddq_d[i]
             + gains.Kv[i] * state.velError[i]
             + gains.Kp[i] * state.posError[i]
             + gains.Ki[i] * state.integralError[i];

        state.uCommand[i] = u[i];
    }

    // Inner loop: tau = M(q)*u + h(q,dq)
    float Mu[NUM_JOINTS];
    MatVec6(_massMatrix, u, Mu);

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        _tauOut[i] = Mu[i] + _hTorque[i];

        // Add DOB compensation if available
        if (_tauDOB)
            _tauOut[i] += _tauDOB[i];

        state.tauCommand[i] = _tauOut[i];
    }
}


void ComputedTorqueController::ComputePDGravity(const Reference_t &_ref,
                                                  const float _q[NUM_JOINTS],
                                                  const float _dq[NUM_JOINTS],
                                                  const float _gravTorque[NUM_JOINTS],
                                                  float _tauOut[NUM_JOINTS])
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        state.posError[i] = _ref.q_d[i] - _q[i];
        state.velError[i] = _ref.dq_d[i] - _dq[i];

        state.integralError[i] += state.posError[i] * dt;
        if (state.integralError[i] > gains.integralLimit)
            state.integralError[i] = gains.integralLimit;
        else if (state.integralError[i] < -gains.integralLimit)
            state.integralError[i] = -gains.integralLimit;

        // PD + gravity compensation (no mass matrix decoupling)
        _tauOut[i] = gains.Kp[i] * state.posError[i]
                   + gains.Kv[i] * state.velError[i]
                   + gains.Ki[i] * state.integralError[i]
                   + _gravTorque[i];

        state.tauCommand[i] = _tauOut[i];
    }
}
