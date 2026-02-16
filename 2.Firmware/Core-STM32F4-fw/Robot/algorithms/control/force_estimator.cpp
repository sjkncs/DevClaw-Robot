#include "force_estimator.h"
#include <cmath>
#include <cstring>


ForceEstimator::ForceEstimator()
{
    memset(&config, 0, sizeof(Config_t));
    memset(&jointForce, 0, sizeof(JointForce_t));
    memset(&cartForce, 0, sizeof(CartesianForce_t));
    memset(filters, 0, sizeof(filters));
    memset(bias, 0, sizeof(bias));
    biasAlpha = 0.0001f;
}


void ForceEstimator::InitDefault(float _dt)
{
    config.dt = _dt;
    config.filterCutoff = 20.0f; // 20Hz LP for force estimation

    // DevClaw Robot motor parameters
    // Kt: torque constant (N*m/A) for typical NEMA17 steppers
    float defaultKt[6] = {0.4f, 0.4f, 0.4f, 0.3f, 0.3f, 0.2f};
    memcpy(config.motorKt, defaultKt, sizeof(defaultKt));

    float defaultGear[6] = {50.0f, 30.0f, 30.0f, 24.0f, 30.0f, 50.0f};
    memcpy(config.gearRatio, defaultGear, sizeof(defaultGear));

    // Deadzone: minimum detectable torque (below this = noise)
    float defaultDZ[6] = {0.1f, 0.15f, 0.1f, 0.05f, 0.04f, 0.03f};
    memcpy(config.deadzone, defaultDZ, sizeof(defaultDZ));

    // Friction (initial estimates, updated by ParamIdentifier)
    float defaultFv[6] = {0.01f, 0.015f, 0.012f, 0.008f, 0.006f, 0.005f};
    float defaultFc[6] = {0.05f, 0.08f, 0.06f, 0.04f, 0.03f, 0.02f};
    memcpy(config.frictionV, defaultFv, sizeof(defaultFv));
    memcpy(config.frictionC, defaultFc, sizeof(defaultFc));

    ComputeFilterCoeffs(config.filterCutoff, config.dt);
    Reset();
}


void ForceEstimator::Init(const Config_t &_config)
{
    config = _config;
    ComputeFilterCoeffs(config.filterCutoff, config.dt);
    Reset();
}


void ForceEstimator::SetFrictionParams(const float _fv[NUM_JOINTS],
                                        const float _fc[NUM_JOINTS])
{
    memcpy(config.frictionV, _fv, NUM_JOINTS * sizeof(float));
    memcpy(config.frictionC, _fc, NUM_JOINTS * sizeof(float));
}


void ForceEstimator::Reset()
{
    memset(&jointForce, 0, sizeof(JointForce_t));
    memset(&cartForce, 0, sizeof(CartesianForce_t));
    memset(filters, 0, sizeof(filters));
    memset(bias, 0, sizeof(bias));
}


void ForceEstimator::ComputeFilterCoeffs(float _cutoffHz, float _dt)
{
    // 2nd-order Butterworth low-pass filter (bilinear transform)
    float wc = 2.0f * (float)M_PI * _cutoffHz;
    float wca = 2.0f / _dt * tanf(wc * _dt / 2.0f); // Pre-warped freq

    float k = wca * _dt / 2.0f;
    float k2 = k * k;
    float sqrt2_k = 1.41421356f * k; // sqrt(2) * k
    float denom = 1.0f + sqrt2_k + k2;

    filterB[0] = k2 / denom;
    filterB[1] = 2.0f * k2 / denom;
    filterB[2] = k2 / denom;

    filterA[0] = 1.0f;
    filterA[1] = (2.0f * k2 - 2.0f) / denom;
    filterA[2] = (1.0f - sqrt2_k + k2) / denom;
}


float ForceEstimator::ApplyFilter(float _input, int _jointIdx)
{
    FilterState_t &f = filters[_jointIdx];

    float output = filterB[0] * _input + filterB[1] * f.x1 + filterB[2] * f.x2
                 - filterA[1] * f.y1 - filterA[2] * f.y2;

    f.x2 = f.x1;
    f.x1 = _input;
    f.y2 = f.y1;
    f.y1 = output;

    return output;
}


float ForceEstimator::ComputeFriction(float _dq, int _jointIdx) const
{
    float fv = config.frictionV[_jointIdx];
    float fc = config.frictionC[_jointIdx];

    // Smooth Coulomb approximation to avoid discontinuity
    // fc * tanh(dq / epsilon) instead of fc * sign(dq)
    float epsilon = 0.1f; // Smoothing region (rad/s)
    float coulomb = fc * tanhf(_dq / epsilon);

    return fv * _dq + coulomb;
}


ForceEstimator::JointForce_t ForceEstimator::Update(
    const float _motorCurrent[NUM_JOINTS],
    const float _q[NUM_JOINTS],
    const float _dq[NUM_JOINTS],
    const float _ddq[NUM_JOINTS],
    const float _tauRNEA[NUM_JOINTS])
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        // Motor-side torque: tau_motor = Kt * I * gear_ratio
        jointForce.tauMotor[i] = config.motorKt[i] * _motorCurrent[i] * config.gearRatio[i];

        // Model-predicted torque (RNEA gives: M*ddq + C*dq + g)
        jointForce.tauModel[i] = _tauRNEA[i];

        // Friction torque
        jointForce.tauFriction[i] = ComputeFriction(_dq[i], i);

        // Raw external torque estimate
        float tauExtRaw = jointForce.tauMotor[i] - jointForce.tauModel[i]
                        - jointForce.tauFriction[i];

        // Subtract slow-adapting bias
        tauExtRaw -= bias[i];

        // Low-pass filter
        float tauExtFiltered = ApplyFilter(tauExtRaw, i);

        // Apply deadzone
        if (fabsf(tauExtFiltered) < config.deadzone[i])
            tauExtFiltered = 0;

        jointForce.tauExt[i] = tauExtFiltered;

        // Update bias estimate (very slow adaptation)
        // Only update when no external force is likely (low velocity, low torque)
        if (fabsf(_dq[i]) < 0.5f && fabsf(tauExtFiltered) < config.deadzone[i] * 2.0f)
        {
            bias[i] += biasAlpha * (tauExtRaw + bias[i]);
        }

        // Confidence based on velocity (higher velocity = better estimation)
        // and acceleration noise (lower accel = more confident)
        float velConf = 1.0f - expf(-fabsf(_dq[i]) * 2.0f);
        float accNoise = fabsf(_ddq[i]) * 0.01f;
        jointForce.confidence[i] = velConf * (1.0f / (1.0f + accNoise));
        if (jointForce.confidence[i] > 1.0f) jointForce.confidence[i] = 1.0f;
        if (jointForce.confidence[i] < 0.0f) jointForce.confidence[i] = 0.0f;
    }

    return jointForce;
}


bool ForceEstimator::Solve6x6(const float *A, const float *b, float *x)
{
    // Gaussian elimination with partial pivoting for 6x6 system
    float Aug[6][7];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
            Aug[i][j] = A[i * 6 + j];
        Aug[i][6] = b[i];
    }

    for (int col = 0; col < 6; col++)
    {
        // Partial pivoting
        int maxRow = col;
        float maxVal = fabsf(Aug[col][col]);
        for (int row = col + 1; row < 6; row++)
        {
            if (fabsf(Aug[row][col]) > maxVal)
            {
                maxVal = fabsf(Aug[row][col]);
                maxRow = row;
            }
        }
        if (maxVal < 1e-10f) return false;

        if (maxRow != col)
        {
            for (int j = col; j <= 6; j++)
            {
                float tmp = Aug[col][j];
                Aug[col][j] = Aug[maxRow][j];
                Aug[maxRow][j] = tmp;
            }
        }

        // Eliminate below
        for (int row = col + 1; row < 6; row++)
        {
            float factor = Aug[row][col] / Aug[col][col];
            for (int j = col; j <= 6; j++)
                Aug[row][j] -= factor * Aug[col][j];
        }
    }

    // Back substitution
    for (int i = 5; i >= 0; i--)
    {
        x[i] = Aug[i][6];
        for (int j = i + 1; j < 6; j++)
            x[i] -= Aug[i][j] * x[j];
        if (fabsf(Aug[i][i]) < 1e-10f) return false;
        x[i] /= Aug[i][i];
    }

    return true;
}


ForceEstimator::CartesianForce_t ForceEstimator::MapToCartesian(
    const float _jacobian[36])
{
    // F_ext = J^{-T} * tau_ext
    // Equivalent: J^T * F_ext = tau_ext
    // Solve: (J*J^T) * F = J * tau_ext  (for square J, just J^T * F = tau)

    // For 6x6 Jacobian (square, non-redundant robot):
    // Solve J^T * F = tau  ->  F = J^{-T} * tau

    // Transpose J
    float JT[36];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            JT[i * 6 + j] = _jacobian[j * 6 + i];

    float F[6];
    bool ok = Solve6x6(JT, jointForce.tauExt, F);

    if (ok)
    {
        cartForce.Fx = F[0];
        cartForce.Fy = F[1];
        cartForce.Fz = F[2];
        cartForce.Tx = F[3];
        cartForce.Ty = F[4];
        cartForce.Tz = F[5];
        cartForce.magnitude = sqrtf(F[0]*F[0] + F[1]*F[1] + F[2]*F[2]);
    }
    else
    {
        // Near singularity: use pseudo-inverse approach
        // F ≈ J * (J*J^T + lambda*I)^{-1} * tau
        float JJT[36];
        memset(JJT, 0, sizeof(JJT));
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    JJT[i * 6 + j] += _jacobian[i * 6 + k] * JT[j * 6 + k];

        // Damped: JJT + lambda*I
        float lambda = 0.01f;
        for (int i = 0; i < 6; i++)
            JJT[i * 6 + i] += lambda;

        // rhs = J * tau_ext
        float rhs[6];
        memset(rhs, 0, sizeof(rhs));
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 6; j++)
                rhs[i] += _jacobian[i * 6 + j] * jointForce.tauExt[j];

        if (Solve6x6(JJT, rhs, F))
        {
            cartForce.Fx = F[0]; cartForce.Fy = F[1]; cartForce.Fz = F[2];
            cartForce.Tx = F[3]; cartForce.Ty = F[4]; cartForce.Tz = F[5];
            cartForce.magnitude = sqrtf(F[0]*F[0] + F[1]*F[1] + F[2]*F[2]);
        }
    }

    return cartForce;
}


void ForceEstimator::CalibrateDeadzone(const float _motorCurrent[NUM_JOINTS],
                                        const float _tauRNEA[NUM_JOINTS])
{
    // Called when robot is stationary: residual = noise floor
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float tauMotor = config.motorKt[i] * _motorCurrent[i] * config.gearRatio[i];
        float residual = fabsf(tauMotor - _tauRNEA[i]);

        // Set deadzone to 1.5x the measured residual
        config.deadzone[i] = residual * 1.5f;
        if (config.deadzone[i] < 0.02f) config.deadzone[i] = 0.02f;

        // Reset bias
        bias[i] = tauMotor - _tauRNEA[i];
    }
}
