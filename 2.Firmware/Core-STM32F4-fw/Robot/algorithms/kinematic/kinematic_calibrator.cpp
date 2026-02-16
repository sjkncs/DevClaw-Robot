#include "kinematic_calibrator.h"
#include <cmath>
#include <cstring>


KinematicCalibrator::KinematicCalibrator()
{
    memset(&nominalDH, 0, sizeof(DHParams_t));
    memset(&correctedDH, 0, sizeof(DHParams_t));
    memset(&result, 0, sizeof(CalibResult_t));
    sampleCount = 0;
}


void KinematicCalibrator::Init(const DHParams_t &_nominalDH)
{
    memcpy(&nominalDH, &_nominalDH, sizeof(DHParams_t));
    memcpy(&correctedDH, &_nominalDH, sizeof(DHParams_t));
    sampleCount = 0;
    memset(&result, 0, sizeof(CalibResult_t));
}


int KinematicCalibrator::RecordSample(const float _q[NUM_JOINTS],
                                        const float _measPos[3],
                                        const float _measRot[3])
{
    if (sampleCount >= MAX_SAMPLES) return -1;

    PoseSample_t &s = samples[sampleCount];
    memcpy(s.q, _q, NUM_JOINTS * sizeof(float));
    memcpy(s.measPos, _measPos, 3 * sizeof(float));

    if (_measRot)
    {
        memcpy(s.measRot, _measRot, 3 * sizeof(float));
        s.hasOrientation = true;
    }
    else
    {
        memset(s.measRot, 0, 3 * sizeof(float));
        s.hasOrientation = false;
    }

    sampleCount++;
    return sampleCount - 1;
}


void KinematicCalibrator::ComputeFK(const float _q[NUM_JOINTS],
                                       const DHParams_t &_dh,
                                       const float _offsets[NUM_JOINTS],
                                       float _pos[3]) const
{
    // Simplified FK using DH convention: T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    // Chain multiplication T_0^6 = T_0^1 * T_1^2 * ... * T_5^6
    float DEG2RAD = 0.01745329251994f;

    // Homogeneous transform accumulator (4x4 as flat array)
    float T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float theta = (_q[i] + _offsets[i]) * DEG2RAD + _dh.theta0[i];
        float d = _dh.d[i];
        float a = _dh.a[i];
        float alpha = _dh.alpha[i];

        float ct = cosf(theta), st = sinf(theta);
        float ca = cosf(alpha), sa = sinf(alpha);

        // DH transform for this joint
        float Ti[16] = {
            ct,  -st*ca,  st*sa,  a*ct,
            st,   ct*ca, -ct*sa,  a*st,
             0,      sa,     ca,     d,
             0,       0,      0,     1
        };

        // T = T * Ti
        float Tnew[16];
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++)
            {
                float sum = 0;
                for (int k = 0; k < 4; k++)
                    sum += T[r * 4 + k] * Ti[k * 4 + c];
                Tnew[r * 4 + c] = sum;
            }
        memcpy(T, Tnew, sizeof(T));
    }

    // Extract position (mm)
    _pos[0] = T[0 * 4 + 3] * 1000.0f;
    _pos[1] = T[1 * 4 + 3] * 1000.0f;
    _pos[2] = T[2 * 4 + 3] * 1000.0f;
}


void KinematicCalibrator::BuildOffsetJacobian(const float _q[NUM_JOINTS],
                                                 const DHParams_t &_dh,
                                                 float _J[3 * NUM_JOINTS]) const
{
    // Numerical differentiation: J_ij = (pos_i(q + eps*e_j) - pos_i(q - eps*e_j)) / (2*eps)
    float eps = 0.01f; // 0.01 degree perturbation

    float offsets0[NUM_JOINTS] = {0};
    float posPlus[3], posMinus[3];

    for (int j = 0; j < NUM_JOINTS; j++)
    {
        float offsetsP[NUM_JOINTS], offsetsM[NUM_JOINTS];
        memset(offsetsP, 0, sizeof(offsetsP));
        memset(offsetsM, 0, sizeof(offsetsM));
        offsetsP[j] = eps;
        offsetsM[j] = -eps;

        ComputeFK(_q, _dh, offsetsP, posPlus);
        ComputeFK(_q, _dh, offsetsM, posMinus);

        for (int i = 0; i < 3; i++)
            _J[i * NUM_JOINTS + j] = (posPlus[i] - posMinus[i]) / (2.0f * eps);
    }
}


bool KinematicCalibrator::SolveRegLS(const float *J, const float *b,
                                        int nRows, int nCols, float lambda, float *x) const
{
    // (J^T J + lambda*I) x = J^T b
    float A[NUM_JOINTS * NUM_JOINTS];
    float rhs[NUM_JOINTS];

    memset(A, 0, nCols * nCols * sizeof(float));
    memset(rhs, 0, nCols * sizeof(float));

    for (int i = 0; i < nCols; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            float sum = 0;
            for (int k = 0; k < nRows; k++)
                sum += J[k * nCols + i] * J[k * nCols + j];
            A[i * nCols + j] = sum;
        }
        A[i * nCols + i] += lambda;

        float rs = 0;
        for (int k = 0; k < nRows; k++)
            rs += J[k * nCols + i] * b[k];
        rhs[i] = rs;
    }

    // Gaussian elimination
    float Aug[NUM_JOINTS][NUM_JOINTS + 1];
    for (int i = 0; i < nCols; i++)
    {
        for (int j = 0; j < nCols; j++)
            Aug[i][j] = A[i * nCols + j];
        Aug[i][nCols] = rhs[i];
    }

    for (int col = 0; col < nCols; col++)
    {
        int maxRow = col;
        float maxVal = fabsf(Aug[col][col]);
        for (int row = col + 1; row < nCols; row++)
            if (fabsf(Aug[row][col]) > maxVal)
            { maxVal = fabsf(Aug[row][col]); maxRow = row; }

        if (maxVal < 1e-10f) return false;

        if (maxRow != col)
            for (int j = col; j <= nCols; j++)
            { float t = Aug[col][j]; Aug[col][j] = Aug[maxRow][j]; Aug[maxRow][j] = t; }

        for (int row = col + 1; row < nCols; row++)
        {
            float f = Aug[row][col] / Aug[col][col];
            for (int j = col; j <= nCols; j++)
                Aug[row][j] -= f * Aug[col][j];
        }
    }

    for (int i = nCols - 1; i >= 0; i--)
    {
        x[i] = Aug[i][nCols];
        for (int j = i + 1; j < nCols; j++)
            x[i] -= Aug[i][j] * x[j];
        x[i] /= Aug[i][i];
    }

    return true;
}


KinematicCalibrator::CalibResult_t KinematicCalibrator::CalibrateOffsets()
{
    memset(&result, 0, sizeof(CalibResult_t));
    result.valid = false;
    result.fullCalib = false;
    result.numSamples = sampleCount;

    if (sampleCount < 3) return result;

    int nRows = sampleCount * 3; // 3 position components per sample
    int nCols = NUM_JOINTS;      // 6 offset parameters

    // Build Jacobian and error vector
    float J[MAX_SAMPLES * 3 * NUM_JOINTS];
    float b[MAX_SAMPLES * 3];

    float zeroOffsets[NUM_JOINTS] = {0};
    float initialResidual = 0;

    for (int k = 0; k < sampleCount; k++)
    {
        // Compute predicted position with nominal DH
        float predPos[3];
        ComputeFK(samples[k].q, nominalDH, zeroOffsets, predPos);

        // Error = measured - predicted
        for (int i = 0; i < 3; i++)
        {
            b[k * 3 + i] = samples[k].measPos[i] - predPos[i];
            float e = b[k * 3 + i];
            initialResidual += e * e;
        }

        // Calibration Jacobian at this pose
        float Jk[3 * NUM_JOINTS];
        BuildOffsetJacobian(samples[k].q, nominalDH, Jk);

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < NUM_JOINTS; j++)
                J[(k * 3 + i) * nCols + j] = Jk[i * NUM_JOINTS + j];
    }

    result.initialRMS = sqrtf(initialResidual / (float)(sampleCount * 3));

    // Solve for joint offsets
    float offsets[NUM_JOINTS];
    float lambda = 0.01f;
    bool ok = SolveRegLS(J, b, nRows, nCols, lambda, offsets);

    if (!ok) return result;

    memcpy(result.jointOffsets, offsets, NUM_JOINTS * sizeof(float));

    // Compute post-calibration residual
    float postResidual = 0;
    float maxErr = 0;
    for (int k = 0; k < sampleCount; k++)
    {
        float predPos[3];
        ComputeFK(samples[k].q, nominalDH, offsets, predPos);

        float err2 = 0;
        for (int i = 0; i < 3; i++)
        {
            float e = samples[k].measPos[i] - predPos[i];
            err2 += e * e;
            postResidual += e * e;
        }
        float errNorm = sqrtf(err2);
        if (errNorm > maxErr) maxErr = errNorm;
    }

    result.residualRMS = sqrtf(postResidual / (float)(sampleCount * 3));
    result.residualMax = maxErr;
    result.valid = true;

    // Update corrected DH
    memcpy(&correctedDH, &nominalDH, sizeof(DHParams_t));
    float DEG2RAD = 0.01745329251994f;
    for (int i = 0; i < NUM_JOINTS; i++)
        correctedDH.theta0[i] += offsets[i] * DEG2RAD;

    return result;
}


KinematicCalibrator::CalibResult_t KinematicCalibrator::CalibrateFull()
{
    // For full DH calibration, we need more samples and solve for more parameters
    // Simplified: just do offset calibration for embedded use
    CalibResult_t r = CalibrateOffsets();
    r.fullCalib = false; // Full DH not implemented in embedded version
    return r;
}


void KinematicCalibrator::ApplyCorrection(const float _qRaw[NUM_JOINTS],
                                             float _qCorrected[NUM_JOINTS]) const
{
    if (!result.valid)
    {
        memcpy(_qCorrected, _qRaw, NUM_JOINTS * sizeof(float));
        return;
    }

    for (int i = 0; i < NUM_JOINTS; i++)
        _qCorrected[i] = _qRaw[i] + result.jointOffsets[i];
}


void KinematicCalibrator::GetCorrectedDH(DHParams_t &_correctedDH) const
{
    memcpy(&_correctedDH, &correctedDH, sizeof(DHParams_t));
}
