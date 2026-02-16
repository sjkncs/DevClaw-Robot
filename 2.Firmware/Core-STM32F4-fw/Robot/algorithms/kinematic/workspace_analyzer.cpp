#include "workspace_analyzer.h"
#include <cmath>
#include <cstring>


WorkspaceAnalyzer::WorkspaceAnalyzer()
{
    memset(&config, 0, sizeof(Config_t));
    memset(&result, 0, sizeof(AnalysisResult_t));
}


void WorkspaceAnalyzer::InitDefault()
{
    float jMin[6] = {-170, -120, -170, -170, -120, -360};
    float jMax[6] = { 170,  120,  170,  170,  120,  360};
    memcpy(config.jointMin, jMin, sizeof(jMin));
    memcpy(config.jointMax, jMax, sizeof(jMax));
    config.singularityThreshold = 50.0f;
    config.jointMarginThreshold = 0.05f;
    config.manipulabilityMin = 0.001f;
}


void WorkspaceAnalyzer::Init(const Config_t &_config)
{
    config = _config;
}


void WorkspaceAnalyzer::JJT(const float *J, float *out)
{
    // out = J * J^T (6x6)
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            float sum = 0;
            for (int k = 0; k < 6; k++)
                sum += J[i * 6 + k] * J[j * 6 + k];
            out[i * 6 + j] = sum;
        }
}


float WorkspaceAnalyzer::Det6x6(const float *A)
{
    // LU decomposition for 6x6 determinant
    float L[36], U[36];
    memset(L, 0, sizeof(L));
    memcpy(U, A, sizeof(U));

    for (int i = 0; i < 6; i++) L[i * 6 + i] = 1.0f;

    for (int col = 0; col < 6; col++)
    {
        for (int row = col + 1; row < 6; row++)
        {
            if (fabsf(U[col * 6 + col]) < 1e-12f) return 0;
            float factor = U[row * 6 + col] / U[col * 6 + col];
            L[row * 6 + col] = factor;
            for (int k = col; k < 6; k++)
                U[row * 6 + k] -= factor * U[col * 6 + k];
        }
    }

    float det = 1.0f;
    for (int i = 0; i < 6; i++)
        det *= U[i * 6 + i];

    return det;
}


float WorkspaceAnalyzer::ComputeManipulability(const float _jacobian[36]) const
{
    float jjt[36];
    JJT(_jacobian, jjt);
    float det = Det6x6(jjt);
    return (det > 0) ? sqrtf(det) : 0;
}


float WorkspaceAnalyzer::LargestEigenvalue(const float *A, int N, int maxIter)
{
    // Power iteration for largest eigenvalue of NxN symmetric matrix
    float v[6] = {1, 0, 0, 0, 0, 0};
    float lambda = 0;

    for (int iter = 0; iter < maxIter; iter++)
    {
        float Av[6] = {0};
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                Av[i] += A[i * N + j] * v[j];

        // Normalize
        float norm = 0;
        for (int i = 0; i < N; i++) norm += Av[i] * Av[i];
        norm = sqrtf(norm);
        if (norm < 1e-12f) return 0;

        lambda = norm;
        for (int i = 0; i < N; i++) v[i] = Av[i] / norm;
    }

    return lambda;
}


float WorkspaceAnalyzer::SmallestEigenvalue(const float *A, int N, int maxIter)
{
    // Inverse power iteration: find smallest eigenvalue
    // Solve A*v_new = v_old each iteration (using Gaussian elimination)
    float v[6] = {0, 1, 0, 0, 0, 0};
    float lambda = 1.0f;

    for (int iter = 0; iter < maxIter; iter++)
    {
        // Solve A * w = v via Gaussian elimination
        float Aug[6][7];
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
                Aug[i][j] = A[i * N + j];
            Aug[i][N] = v[i];
        }

        // Forward elimination with partial pivoting
        for (int col = 0; col < N; col++)
        {
            int maxRow = col;
            float maxVal = fabsf(Aug[col][col]);
            for (int row = col + 1; row < N; row++)
                if (fabsf(Aug[row][col]) > maxVal) { maxVal = fabsf(Aug[row][col]); maxRow = row; }

            if (maxVal < 1e-10f) return 0; // Singular

            if (maxRow != col)
                for (int j = col; j <= N; j++)
                { float t = Aug[col][j]; Aug[col][j] = Aug[maxRow][j]; Aug[maxRow][j] = t; }

            for (int row = col + 1; row < N; row++)
            {
                float f = Aug[row][col] / Aug[col][col];
                for (int j = col; j <= N; j++)
                    Aug[row][j] -= f * Aug[col][j];
            }
        }

        // Back substitution
        float w[6];
        for (int i = N - 1; i >= 0; i--)
        {
            w[i] = Aug[i][N];
            for (int j = i + 1; j < N; j++)
                w[i] -= Aug[i][j] * w[j];
            if (fabsf(Aug[i][i]) < 1e-10f) return 0;
            w[i] /= Aug[i][i];
        }

        // Normalize
        float norm = 0;
        for (int i = 0; i < N; i++) norm += w[i] * w[i];
        norm = sqrtf(norm);
        if (norm < 1e-12f) return 0;

        lambda = 1.0f / norm;
        for (int i = 0; i < N; i++) v[i] = w[i] / norm;
    }

    return lambda;
}


float WorkspaceAnalyzer::ComputeConditionNumber(const float _jacobian[36]) const
{
    float jjt[36];
    JJT(_jacobian, jjt);

    float sigMax2 = LargestEigenvalue(jjt, 6, 15);
    float sigMin2 = SmallestEigenvalue(jjt, 6, 15);

    if (sigMin2 < 1e-10f) return 1e6f; // Near-singular
    return sqrtf(sigMax2 / sigMin2);
}


bool WorkspaceAnalyzer::CheckSingularity(const float _jacobian[36]) const
{
    return ComputeConditionNumber(_jacobian) > config.singularityThreshold;
}


bool WorkspaceAnalyzer::CheckJointLimits(const float _q[NUM_JOINTS]) const
{
    for (int i = 0; i < NUM_JOINTS; i++)
        if (_q[i] < config.jointMin[i] || _q[i] > config.jointMax[i])
            return false;
    return true;
}


void WorkspaceAnalyzer::GetSingularityAvoidanceGradient(const float _jacobian[36],
                                                           float _gradient[NUM_JOINTS]) const
{
    // Gradient of manipulability w.r.t. joint angles (numerical approximation)
    // dw/dq_i ≈ (w(q+eps*e_i) - w(q-eps*e_i)) / (2*eps)
    // Approximated using Jacobian perturbation

    float w0 = ComputeManipulability(_jacobian);

    // Simple approach: move joints toward configuration center
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float center = (config.jointMax[i] + config.jointMin[i]) / 2.0f;
        float range = config.jointMax[i] - config.jointMin[i];
        // Gradient points toward center, scaled by proximity to limits
        _gradient[i] = (center - 0) / (range + 1e-6f); // Normalized
    }

    // Scale by inverse of condition number (stronger near singularity)
    float kappa = ComputeConditionNumber(_jacobian);
    float scale = (kappa > 10.0f) ? 1.0f / kappa : 0;
    for (int i = 0; i < NUM_JOINTS; i++)
        _gradient[i] *= scale;
}


WorkspaceAnalyzer::AnalysisResult_t WorkspaceAnalyzer::Analyze(
    const float _q[NUM_JOINTS], const float _jacobian[36])
{
    // Manipulability
    result.manipulability = ComputeManipulability(_jacobian);

    // Condition number
    result.conditionNumber = ComputeConditionNumber(_jacobian);
    result.nearSingularity = (result.conditionNumber > config.singularityThreshold);
    result.singularityDistance = 1.0f / (result.conditionNumber + 1e-6f);

    // Joint limit proximity
    result.minJointMargin = 1.0f;
    result.worstJoint = 0;
    result.nearJointLimit = false;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float range = config.jointMax[i] - config.jointMin[i];
        if (range < 1e-6f) range = 1e-6f;

        float distMin = _q[i] - config.jointMin[i];
        float distMax = config.jointMax[i] - _q[i];
        float minDist = (distMin < distMax) ? distMin : distMax;

        result.jointLimitProximity[i] = minDist / range;
        if (result.jointLimitProximity[i] < 0) result.jointLimitProximity[i] = 0;

        if (result.jointLimitProximity[i] < result.minJointMargin)
        {
            result.minJointMargin = result.jointLimitProximity[i];
            result.worstJoint = i;
        }

        if (result.jointLimitProximity[i] < config.jointMarginThreshold)
            result.nearJointLimit = true;
    }

    // Reachability
    result.isReachable = CheckJointLimits(_q) && !result.nearSingularity;

    return result;
}
