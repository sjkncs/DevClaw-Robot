#include "gravity_calibrator.h"
#include <cmath>
#include <cstring>


GravityCalibrator::GravityCalibrator()
{
    memset(motorKt, 0, sizeof(motorKt));
    memset(gearRatio, 0, sizeof(gearRatio));
    sampleCount = 0;
    memset(&result, 0, sizeof(CalibResult_t));
}


void GravityCalibrator::Init(const float _motorKt[NUM_JOINTS],
                               const float _gearRatio[NUM_JOINTS])
{
    memcpy(motorKt, _motorKt, sizeof(motorKt));
    memcpy(gearRatio, _gearRatio, sizeof(gearRatio));
    sampleCount = 0;
    memset(&result, 0, sizeof(CalibResult_t));
}


int GravityCalibrator::RecordSample(const float _q[NUM_JOINTS],
                                      const float _current[NUM_JOINTS])
{
    if (sampleCount >= MAX_POSES) return -1;

    Sample_t &s = samples[sampleCount];
    memcpy(s.q, _q, NUM_JOINTS * sizeof(float));

    // Convert current to torque: tau = Kt * I * gear_ratio
    for (int i = 0; i < NUM_JOINTS; i++)
        s.tauMeas[i] = motorKt[i] * _current[i] * gearRatio[i];

    sampleCount++;
    return sampleCount - 1;
}


void GravityCalibrator::BuildGravityRegressor(const float _q[NUM_JOINTS],
                                                 float _Y[NUM_JOINTS * NUM_PARAMS]) const
{
    // Simplified gravity regressor for 6-DOF robot
    // Parameters: pi = [m1*cx1, m1*cz1, m2*cx2, m2*cz2, ..., m6*cx6, m6*cz6, m_payload]
    // 13 parameters total
    //
    // tau_g_i = sum over links j>=i of: m_j * g * (partial derivative of COM_j height w.r.t. q_i)
    //
    // For a serial chain, the gravity regressor depends on cumulative sin/cos of joint angles.

    memset(_Y, 0, NUM_JOINTS * NUM_PARAMS * sizeof(float));

    float DEG2RAD = 0.01745329251994f;
    float g = 9.81f;

    // Cumulative angles for planar gravity projection
    float c1 = cosf(_q[0] * DEG2RAD);
    float s1 = sinf(_q[0] * DEG2RAD);
    float c12 = cosf((_q[0] + _q[1]) * DEG2RAD);
    float s12 = sinf((_q[0] + _q[1]) * DEG2RAD);
    float c123 = cosf((_q[0] + _q[1] + _q[2]) * DEG2RAD);
    float s123 = sinf((_q[0] + _q[1] + _q[2]) * DEG2RAD);

    // Joint 1 (base rotation - affects all links in gravity)
    // Link 1: params [0]=m1*cx1, [1]=m1*cz1
    _Y[0 * NUM_PARAMS + 0] = g * c1;   // m1*cx1 contribution
    _Y[0 * NUM_PARAMS + 1] = g * s1;   // m1*cz1 contribution

    // Link 2: params [2]=m2*cx2, [3]=m2*cz2
    _Y[0 * NUM_PARAMS + 2] = g * c12;
    _Y[0 * NUM_PARAMS + 3] = g * s12;
    _Y[1 * NUM_PARAMS + 2] = g * c12;
    _Y[1 * NUM_PARAMS + 3] = g * s12;

    // Link 3: params [4]=m3*cx3, [5]=m3*cz3
    _Y[0 * NUM_PARAMS + 4] = g * c123;
    _Y[0 * NUM_PARAMS + 5] = g * s123;
    _Y[1 * NUM_PARAMS + 4] = g * c123;
    _Y[1 * NUM_PARAMS + 5] = g * s123;
    _Y[2 * NUM_PARAMS + 4] = g * c123;
    _Y[2 * NUM_PARAMS + 5] = g * s123;

    // Links 4-6 (wrist): simplified - primarily affected by joints 1-3
    // params [6..11] for links 4-6
    float c_wrist = c123; // Approximate wrist as extension
    float s_wrist = s123;
    for (int link = 3; link < 6; link++)
    {
        int paramIdx = link * 2;
        for (int joint = 0; joint <= link && joint < NUM_JOINTS; joint++)
        {
            _Y[joint * NUM_PARAMS + paramIdx] = g * c_wrist * 0.5f;
            _Y[joint * NUM_PARAMS + paramIdx + 1] = g * s_wrist * 0.5f;
        }
    }

    // Payload: param [12] = m_payload
    // Payload at end-effector, affected by all joints
    for (int joint = 0; joint < NUM_JOINTS; joint++)
        _Y[joint * NUM_PARAMS + 12] = g * c123 * 0.3f; // Approx lever arm
}


bool GravityCalibrator::SolveWLS(const float *Y, const float *b, const float *W,
                                    int nRows, int nCols, float lambda, float *x) const
{
    // Solve (Y^T W Y + lambda*I) * x = Y^T W b
    // nRows = total measurements, nCols = NUM_PARAMS

    // Compute A = Y^T * W * Y + lambda*I  (nCols x nCols)
    float A[NUM_PARAMS * NUM_PARAMS];
    float rhs[NUM_PARAMS];

    memset(A, 0, sizeof(A));
    memset(rhs, 0, sizeof(rhs));

    for (int i = 0; i < nCols; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            float sum = 0;
            for (int k = 0; k < nRows; k++)
            {
                float w = W ? W[k] : 1.0f;
                sum += Y[k * nCols + i] * w * Y[k * nCols + j];
            }
            A[i * nCols + j] = sum;
        }
        A[i * nCols + i] += lambda; // Regularization

        float rhsSum = 0;
        for (int k = 0; k < nRows; k++)
        {
            float w = W ? W[k] : 1.0f;
            rhsSum += Y[k * nCols + i] * w * b[k];
        }
        rhs[i] = rhsSum;
    }

    // Solve A * x = rhs via Gaussian elimination with partial pivoting
    float Aug[NUM_PARAMS][NUM_PARAMS + 1];
    for (int i = 0; i < nCols; i++)
    {
        for (int j = 0; j < nCols; j++)
            Aug[i][j] = A[i * nCols + j];
        Aug[i][nCols] = rhs[i];
    }

    for (int col = 0; col < nCols; col++)
    {
        // Pivot
        int maxRow = col;
        float maxVal = fabsf(Aug[col][col]);
        for (int row = col + 1; row < nCols; row++)
            if (fabsf(Aug[row][col]) > maxVal)
            { maxVal = fabsf(Aug[row][col]); maxRow = row; }

        if (maxVal < 1e-10f) return false;

        if (maxRow != col)
            for (int j = col; j <= nCols; j++)
            { float t = Aug[col][j]; Aug[col][j] = Aug[maxRow][j]; Aug[maxRow][j] = t; }

        // Eliminate
        for (int row = col + 1; row < nCols; row++)
        {
            float f = Aug[row][col] / Aug[col][col];
            for (int j = col; j <= nCols; j++)
                Aug[row][j] -= f * Aug[col][j];
        }
    }

    // Back substitution
    for (int i = nCols - 1; i >= 0; i--)
    {
        x[i] = Aug[i][nCols];
        for (int j = i + 1; j < nCols; j++)
            x[i] -= Aug[i][j] * x[j];
        x[i] /= Aug[i][i];
    }

    return true;
}


GravityCalibrator::CalibResult_t GravityCalibrator::Calibrate()
{
    memset(&result, 0, sizeof(CalibResult_t));
    result.valid = false;

    if (sampleCount < 3)
    {
        result.numPoses = sampleCount;
        return result;
    }

    int nRows = sampleCount * NUM_JOINTS;
    int nCols = NUM_PARAMS;

    // Build full regressor and measurement vector
    // Using stack allocation for embedded (max 50*6*13 = 3900 floats)
    float Y[MAX_POSES * NUM_JOINTS * NUM_PARAMS];
    float b[MAX_POSES * NUM_JOINTS];

    float DEG2RAD = 0.01745329251994f;

    for (int k = 0; k < sampleCount; k++)
    {
        float qRad[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++)
            qRad[i] = samples[k].q[i]; // BuildGravityRegressor uses deg internally

        float Yk[NUM_JOINTS * NUM_PARAMS];
        BuildGravityRegressor(samples[k].q, Yk);

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            b[k * NUM_JOINTS + i] = samples[k].tauMeas[i];
            for (int j = 0; j < NUM_PARAMS; j++)
                Y[(k * NUM_JOINTS + i) * nCols + j] = Yk[i * NUM_PARAMS + j];
        }
    }

    // Solve with Tikhonov regularization
    float lambda = 0.001f;
    bool ok = SolveWLS(Y, b, nullptr, nRows, nCols, lambda, result.params);

    if (!ok) return result;

    // Compute residual
    float residualSum = 0;
    for (int k = 0; k < nRows; k++)
    {
        float pred = 0;
        for (int j = 0; j < nCols; j++)
            pred += Y[k * nCols + j] * result.params[j];
        float err = pred - b[k];
        residualSum += err * err;
    }
    result.residualNorm = sqrtf(residualSum / (float)nRows);

    result.numPoses = sampleCount;
    result.valid = true;
    result.payloadMass = result.params[12]; // Last parameter
    result.payloadCOM[0] = 0;
    result.payloadCOM[1] = 0;
    result.payloadCOM[2] = 0;

    return result;
}


void GravityCalibrator::ComputeGravityTorque(const float _q[NUM_JOINTS],
                                                float _tauG[NUM_JOINTS]) const
{
    if (!result.valid)
    {
        memset(_tauG, 0, NUM_JOINTS * sizeof(float));
        return;
    }

    float Y[NUM_JOINTS * NUM_PARAMS];
    BuildGravityRegressor(_q, Y);

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float tau = 0;
        for (int j = 0; j < NUM_PARAMS; j++)
            tau += Y[i * NUM_PARAMS + j] * result.params[j];
        _tauG[i] = tau;
    }
}
