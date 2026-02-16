#include "param_identifier.h"
#include <cmath>
#include <cstring>


// ========================== Constructor ==========================

ParamIdentifier::ParamIdentifier()
{
    sampleCount = 0;

    // Default DH parameters (DevClaw Robot, same as kinematic solver)
    float tmpDH[6][4] = {
        {0.0f,            0.109f,  0.035f,  -(float)M_PI_2},
        {-(float)M_PI_2,  0.0f,    0.146f,  0.0f},
        {(float)M_PI_2,   0.052f,  0.0f,    (float)M_PI_2},
        {0.0f,            0.115f,  0.0f,    -(float)M_PI_2},
        {0.0f,            0.0f,    0.0f,    (float)M_PI_2},
        {0.0f,            0.072f,  0.0f,    0.0f}
    };
    memcpy(DH, tmpDH, sizeof(tmpDH));

    // Default reduction ratios
    float defaultRatios[6] = {50, 30, 30, 24, 30, 50};
    memcpy(reductions, defaultRatios, sizeof(defaultRatios));
}


void ParamIdentifier::SetDHParams(const float _dh[NUM_JOINTS][4])
{
    memcpy(DH, _dh, NUM_JOINTS * 4 * sizeof(float));
}


void ParamIdentifier::SetReductionRatios(const float _ratios[NUM_JOINTS])
{
    memcpy(reductions, _ratios, NUM_JOINTS * sizeof(float));
}


// ========================== Excitation Trajectory ==========================

int ParamIdentifier::GenerateExcitationTrajectory(const FourierTrajectoryConfig_t &_config,
                                                    int _numPoints,
                                                    float _qOut[][NUM_JOINTS],
                                                    float _dqOut[][NUM_JOINTS],
                                                    float _ddqOut[][NUM_JOINTS])
{
    if (_numPoints <= 0) return 0;

    float dt = _config.duration / (float)_numPoints;
    float w0 = 2.0f * (float)M_PI * _config.fundamentalFreq;
    int nH = _config.numHarmonics;
    if (nH > 10) nH = 10;
    if (nH < 1) nH = 1;

    int validPoints = 0;

    for (int n = 0; n < _numPoints; n++)
    {
        float t = (float)n * dt;
        bool valid = true;

        for (int j = 0; j < NUM_JOINTS; j++)
        {
            // q_j(t) = q0_j + sum_{k=1}^{nH} [a_{jk}/(k*w0) * sin(k*w0*t)
            //                                  - b_{jk}/(k*w0) * cos(k*w0*t)]
            // dq_j(t) = sum_{k=1}^{nH} [a_{jk} * cos(k*w0*t) + b_{jk} * sin(k*w0*t)]
            // ddq_j(t) = sum_{k=1}^{nH} [-a_{jk}*k*w0 * sin(k*w0*t)
            //                             + b_{jk}*k*w0 * cos(k*w0*t)]

            float q_center = (_config.jointLimitsMax[j] + _config.jointLimitsMin[j]) * 0.5f;
            float q = q_center;
            float dq = 0;
            float ddq = 0;

            for (int k = 0; k < nH; k++)
            {
                float wk = (float)(k + 1) * w0;
                float ak = _config.amplitudes[j][k];
                // Use alternating sin/cos pattern as coefficients
                float bk = ak * 0.7f; // Phase offset for richness

                float swt = sinf(wk * t);
                float cwt = cosf(wk * t);

                q   += ak / wk * swt - bk / wk * cwt;
                dq  += ak * cwt + bk * swt;
                ddq += -ak * wk * swt + bk * wk * cwt;
            }

            // Check joint limits
            if (q < _config.jointLimitsMin[j] || q > _config.jointLimitsMax[j])
                valid = false;
            if (fabsf(dq) > _config.velLimits[j])
                valid = false;
            if (fabsf(ddq) > _config.accLimits[j])
                valid = false;

            // Convert deg to rad for output
            float DEG2RAD = 0.01745329251994f;
            _qOut[n][j] = q * DEG2RAD;
            _dqOut[n][j] = dq * DEG2RAD;
            _ddqOut[n][j] = ddq * DEG2RAD;
        }

        if (valid) validPoints++;
    }

    return validPoints;
}


// ========================== Data Recording ==========================

void ParamIdentifier::RecordSample(const float _q[NUM_JOINTS],
                                    const float _dq[NUM_JOINTS],
                                    const float _ddq[NUM_JOINTS],
                                    const float _tau[NUM_JOINTS])
{
    if (sampleCount >= MAX_SAMPLES) return;

    Sample_t &s = samples[sampleCount];
    memcpy(s.q, _q, NUM_JOINTS * sizeof(float));
    memcpy(s.dq, _dq, NUM_JOINTS * sizeof(float));
    memcpy(s.ddq, _ddq, NUM_JOINTS * sizeof(float));
    memcpy(s.tau, _tau, NUM_JOINTS * sizeof(float));

    // Weight based on velocity magnitude (higher velocity = better SNR for friction)
    float velNorm = 0;
    for (int i = 0; i < NUM_JOINTS; i++)
        velNorm += _dq[i] * _dq[i];
    s.weight = 1.0f + sqrtf(velNorm) * 0.1f;

    sampleCount++;
}


// ========================== Regressor Construction ==========================

void ParamIdentifier::ComputeGravityRegressor(const float _q[NUM_JOINTS],
                                                float _grav[NUM_JOINTS][NUM_JOINTS])
{
    // Numerical differentiation of gravity torque w.r.t. gravity parameter
    // For the reduced model, gravity torque on joint i depends on
    // all downstream link gravity parameters.
    //
    // Simplified model: g_i(q) ≈ f(q_1..q_i) for each joint
    // Using DH parameters to compute the gravity moment arm

    float g = 9.81f;
    memset(_grav, 0, NUM_JOINTS * NUM_JOINTS * sizeof(float));

    // Cumulative rotation from base to each joint
    // For a standard 6-DOF arm, gravity affects joints 1-3 primarily
    // Joint 1: base rotation -> gravity torque from offset mass
    // Joint 2: shoulder -> full arm gravity
    // Joint 3: elbow -> forearm + wrist gravity

    // Compute sin/cos of joint angles
    float sq[NUM_JOINTS], cq[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        sq[i] = sinf(_q[i] + DH[i][0]);
        cq[i] = cosf(_q[i] + DH[i][0]);
    }

    // Joint 2 gravity regressor (shoulder - primary gravity load)
    // tau_g2 = -m2*lc2*g*cos(q2) - (m3*l3 + m4*lc4 + ...)*g*cos(q2+q3) - ...
    _grav[1][1] = -g * cq[1];  // Joint 2 own gravity param
    _grav[1][2] = -g * cosf(_q[1] + DH[1][0] + _q[2] + DH[2][0]); // J3 gravity on J2

    // Joint 3 gravity regressor (elbow)
    _grav[2][2] = -g * cosf(_q[1] + DH[1][0] + _q[2] + DH[2][0]);

    // Joint 4-6: wrist joints have small gravity contribution
    float s23 = sinf(_q[1] + DH[1][0] + _q[2] + DH[2][0]);
    float c23 = cosf(_q[1] + DH[1][0] + _q[2] + DH[2][0]);
    _grav[1][3] = -g * c23 * cq[3]; // Approximate
    _grav[3][3] = -g * s23 * sq[3];

    // Joints 5,6 have negligible gravity torque for this arm geometry
    _grav[4][4] = 0;
    _grav[5][5] = 0;
}


void ParamIdentifier::BuildReducedRegressor(const float _q[NUM_JOINTS],
                                              const float _dq[NUM_JOINTS],
                                              const float _ddq[NUM_JOINTS],
                                              float _Y[NUM_JOINTS * REDUCED_PARAMS])
{
    // Reduced parameter vector per joint i (4 params each):
    //   pi = [m_i*lc_i, I_reflected_i, fv_i, fc_i]
    //
    // Regressor row for joint i:
    //   tau_i = g_i(q)*[m_i*lc_i] + ddq_i*[I_reflected_i] + dq_i*[fv_i] + sign(dq_i)*[fc_i]
    //         + coupling terms from other joints (included in gravity regressor)

    memset(_Y, 0, NUM_JOINTS * REDUCED_PARAMS * sizeof(float));

    // Gravity regressor
    float gravReg[NUM_JOINTS][NUM_JOINTS];
    ComputeGravityRegressor(_q, gravReg);

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        int rowOffset = i * REDUCED_PARAMS;

        // For each joint j's gravity parameter affecting joint i
        for (int j = 0; j < NUM_JOINTS; j++)
        {
            int paramIdx = j * 4 + 0; // m*lc is the 0th param of joint j
            _Y[rowOffset + paramIdx] = gravReg[i][j];
        }

        // Reflected inertia: tau_i += I_i * ddq_i
        int inertiaIdx = i * 4 + 1;
        _Y[rowOffset + inertiaIdx] = _ddq[i];

        // Viscous friction: tau_i += fv_i * dq_i
        int fvIdx = i * 4 + 2;
        _Y[rowOffset + fvIdx] = _dq[i];

        // Coulomb friction: tau_i += fc_i * sign(dq_i)
        int fcIdx = i * 4 + 3;
        if (_dq[i] > 0.01f)
            _Y[rowOffset + fcIdx] = 1.0f;
        else if (_dq[i] < -0.01f)
            _Y[rowOffset + fcIdx] = -1.0f;
        else
            _Y[rowOffset + fcIdx] = 0.0f; // Dead zone to avoid chattering
    }
}


// ========================== Solver ==========================

bool ParamIdentifier::CholeskySolve(const float *A, const float *b, float *x, int N)
{
    // Cholesky decomposition: A = L * L^T
    float L[REDUCED_PARAMS * REDUCED_PARAMS];
    memset(L, 0, N * N * sizeof(float));

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            float sum = 0;
            for (int k = 0; k < j; k++)
                sum += L[i * N + k] * L[j * N + k];

            if (i == j)
            {
                float val = A[i * N + i] - sum;
                if (val <= 1e-10f) return false;
                L[i * N + j] = sqrtf(val);
            }
            else
            {
                if (fabsf(L[j * N + j]) < 1e-10f) return false;
                L[i * N + j] = (A[i * N + j] - sum) / L[j * N + j];
            }
        }
    }

    // Forward substitution: L * y = b
    float y[REDUCED_PARAMS];
    for (int i = 0; i < N; i++)
    {
        float sum = 0;
        for (int k = 0; k < i; k++)
            sum += L[i * N + k] * y[k];
        y[i] = (b[i] - sum) / L[i * N + i];
    }

    // Backward substitution: L^T * x = y
    for (int i = N - 1; i >= 0; i--)
    {
        float sum = 0;
        for (int k = i + 1; k < N; k++)
            sum += L[k * N + i] * x[k];
        x[i] = (y[i] - sum) / L[i * N + i];
    }

    return true;
}


bool ParamIdentifier::SolveNormalEquations(const float *_YtWY, const float *_YtWtau, float *_pi)
{
    // Add Tikhonov regularization for numerical stability
    // (YtWY + lambda*I) * pi = YtWtau
    float A[REDUCED_PARAMS * REDUCED_PARAMS];
    memcpy(A, _YtWY, REDUCED_PARAMS * REDUCED_PARAMS * sizeof(float));

    float lambda = 0.001f; // Regularization
    for (int i = 0; i < REDUCED_PARAMS; i++)
        A[i * REDUCED_PARAMS + i] += lambda;

    return CholeskySolve(A, _YtWtau, _pi, REDUCED_PARAMS);
}


// ========================== Identification ==========================

ParamIdentifier::IdentResult_t ParamIdentifier::Identify()
{
    IdentResult_t result;
    memset(&result, 0, sizeof(IdentResult_t));
    result.valid = false;
    result.numSamples = sampleCount;

    if (sampleCount < REDUCED_PARAMS * 2)
    {
        // Not enough samples
        return result;
    }

    // Build normal equations: (Y^T W Y) * pi = Y^T W tau
    float YtWY[REDUCED_PARAMS * REDUCED_PARAMS];
    float YtWtau[REDUCED_PARAMS];
    memset(YtWY, 0, sizeof(YtWY));
    memset(YtWtau, 0, sizeof(YtWtau));

    float Y_row[NUM_JOINTS * REDUCED_PARAMS];

    for (int s = 0; s < sampleCount; s++)
    {
        BuildReducedRegressor(samples[s].q, samples[s].dq, samples[s].ddq, Y_row);

        float w = samples[s].weight;

        // Accumulate normal equations for each joint equation
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            float *Yi = &Y_row[i * REDUCED_PARAMS]; // Row i of regressor
            float taui = samples[s].tau[i];

            // YtWY += w * Yi^T * Yi
            for (int p = 0; p < REDUCED_PARAMS; p++)
            {
                for (int q = 0; q < REDUCED_PARAMS; q++)
                {
                    YtWY[p * REDUCED_PARAMS + q] += w * Yi[p] * Yi[q];
                }
                // YtWtau += w * Yi^T * taui
                YtWtau[p] += w * Yi[p] * taui;
            }
        }
    }

    // Solve normal equations
    float pi[REDUCED_PARAMS];
    if (!SolveNormalEquations(YtWY, YtWtau, pi))
    {
        return result;
    }

    memcpy(result.params, pi, REDUCED_PARAMS * sizeof(float));

    // Compute residual
    float residual = 0;
    for (int s = 0; s < sampleCount; s++)
    {
        BuildReducedRegressor(samples[s].q, samples[s].dq, samples[s].ddq, Y_row);

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            float *Yi = &Y_row[i * REDUCED_PARAMS];
            float predicted = 0;
            for (int p = 0; p < REDUCED_PARAMS; p++)
                predicted += Yi[p] * pi[p];

            float err = samples[s].tau[i] - predicted;
            residual += samples[s].weight * err * err;
        }
    }
    result.residual = residual / (float)(sampleCount * NUM_JOINTS);

    // Decode per-joint parameters
    for (int j = 0; j < NUM_JOINTS; j++)
    {
        result.gravityParam[j]     = pi[j * 4 + 0];
        result.reflectedInertia[j] = pi[j * 4 + 1];
        result.viscousFriction[j]  = pi[j * 4 + 2];
        result.coulombFriction[j]  = pi[j * 4 + 3];
    }

    // Compute condition number estimate (ratio of largest to smallest diagonal of YtWY)
    float maxDiag = 0, minDiag = 1e30f;
    for (int i = 0; i < REDUCED_PARAMS; i++)
    {
        float d = fabsf(YtWY[i * REDUCED_PARAMS + i]);
        if (d > maxDiag) maxDiag = d;
        if (d < minDiag && d > 1e-10f) minDiag = d;
    }
    result.conditionNumber = (minDiag > 1e-10f) ? sqrtf(maxDiag / minDiag) : 1e6f;

    result.valid = true;
    return result;
}
