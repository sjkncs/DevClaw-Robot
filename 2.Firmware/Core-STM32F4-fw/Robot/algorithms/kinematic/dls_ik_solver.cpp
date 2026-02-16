#include "dls_ik_solver.h"
#include <cmath>
#include <cstring>


// ========================== Constructor & Setup ==========================

DLS_IK_Solver::DLS_IK_Solver()
{
    // Default configuration
    config.lambda_max = 0.5f;
    config.lambda_min = 0.001f;
    config.manip_threshold = 0.01f;
    config.pos_tolerance = 0.5f;      // mm
    config.rot_tolerance = 0.01f;     // rad (~0.57 deg)
    config.max_iterations = 50;
    config.step_size = 1.0f;
    config.joint_limit_avoidance = true;
    config.joint_limit_gain = 0.1f;

    // Default joint limits (DevClaw Robot)
    jointLimits.min[0] = -170; jointLimits.max[0] = 170;
    jointLimits.min[1] = -73;  jointLimits.max[1] = 90;
    jointLimits.min[2] = 35;   jointLimits.max[2] = 180;
    jointLimits.min[3] = -180; jointLimits.max[3] = 180;
    jointLimits.min[4] = -120; jointLimits.max[4] = 120;
    jointLimits.min[5] = -720; jointLimits.max[5] = 720;
    limitsSet = true;
}


void DLS_IK_Solver::AttachSolvers(DOF6Kinematic *_kinSolver, DOF6Dynamics *_dynSolver)
{
    kinSolver = _kinSolver;
    dynSolver = _dynSolver;
}


void DLS_IK_Solver::SetConfig(const Config_t &_config)
{
    config = _config;
}


void DLS_IK_Solver::SetJointLimits(const JointLimits_t &_limits)
{
    jointLimits = _limits;
    limitsSet = true;
}


// ========================== Orientation Error ==========================

void DLS_IK_Solver::OrientationError(const float R_c[9], const float R_t[9], float dw[3])
{
    // Orientation error using: dw = 0.5 * (n_c x n_t + s_c x s_t + a_c x a_t)
    // where n, s, a are columns of the rotation matrices
    // This gives the angular velocity vector to rotate from R_c to R_t

    // R_error = R_t * R_c^T
    // Extract axis-angle from R_error -> angular velocity direction * angle
    // Simplified: use the skew-symmetric part of R_error

    float Re[9]; // R_t * R_c^T
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            Re[3 * i + j] = 0;
            for (int k = 0; k < 3; k++)
                Re[3 * i + j] += R_t[3 * i + k] * R_c[3 * j + k]; // R_c^T: swap k,j
        }

    // Extract rotation vector from rotation matrix using Rodrigues' formula
    // theta = acos((trace(R)-1)/2)
    // axis = [R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)] / (2*sin(theta))
    float trace = Re[0] + Re[4] + Re[8];
    float cos_theta = (trace - 1.0f) * 0.5f;

    if (cos_theta > 1.0f) cos_theta = 1.0f;
    if (cos_theta < -1.0f) cos_theta = -1.0f;

    float theta = acosf(cos_theta);

    if (fabsf(theta) < 1e-6f)
    {
        // No rotation needed
        dw[0] = 0; dw[1] = 0; dw[2] = 0;
        return;
    }

    float sin_theta = sinf(theta);
    if (fabsf(sin_theta) < 1e-6f)
    {
        // theta ≈ pi, use eigenvalue method
        // Find the column of (Re + I) with largest norm
        float col0[3] = {Re[0] + 1, Re[3], Re[6]};
        float col1[3] = {Re[1], Re[4] + 1, Re[7]};
        float col2[3] = {Re[2], Re[5], Re[8] + 1};

        float n0 = col0[0] * col0[0] + col0[1] * col0[1] + col0[2] * col0[2];
        float n1 = col1[0] * col1[0] + col1[1] * col1[1] + col1[2] * col1[2];
        float n2 = col2[0] * col2[0] + col2[1] * col2[1] + col2[2] * col2[2];

        float *best = col0;
        float bestN = n0;
        if (n1 > bestN) { best = col1; bestN = n1; }
        if (n2 > bestN) { best = col2; bestN = n2; }

        float invN = 1.0f / sqrtf(bestN);
        dw[0] = best[0] * invN * theta;
        dw[1] = best[1] * invN * theta;
        dw[2] = best[2] * invN * theta;
        return;
    }

    float k = theta / (2.0f * sin_theta);
    dw[0] = k * (Re[7] - Re[5]); // R(2,1) - R(1,2)
    dw[1] = k * (Re[2] - Re[6]); // R(0,2) - R(2,0)
    dw[2] = k * (Re[3] - Re[1]); // R(1,0) - R(0,1)
}


// ========================== Pose Error ==========================

void DLS_IK_Solver::ComputePoseError(const DOF6Kinematic::Pose6D_t &_currentPose,
                                      const DOF6Kinematic::Pose6D_t &_targetPose,
                                      float _dx[N])
{
    // Position error (mm) - FK returns meters internally, but poses are in mm
    _dx[0] = _targetPose.X - _currentPose.X;
    _dx[1] = _targetPose.Y - _currentPose.Y;
    _dx[2] = _targetPose.Z - _currentPose.Z;

    // Orientation error
    float R_target[9];
    if (_targetPose.hasR)
    {
        memcpy(R_target, _targetPose.R, 9 * sizeof(float));
    }
    else
    {
        // Convert Euler angles to rotation matrix
        float DEG2RAD = 0.01745329251994f;
        float ea[3] = {_targetPose.A * DEG2RAD, _targetPose.B * DEG2RAD, _targetPose.C * DEG2RAD};
        float ca = cosf(ea[2]), sa = sinf(ea[2]);
        float cb = cosf(ea[1]), sb = sinf(ea[1]);
        float cc = cosf(ea[0]), sc = sinf(ea[0]);
        R_target[0] = ca * cb; R_target[1] = ca * sb * sc - sa * cc; R_target[2] = ca * sb * cc + sa * sc;
        R_target[3] = sa * cb; R_target[4] = sa * sb * sc + ca * cc; R_target[5] = sa * sb * cc - ca * sc;
        R_target[6] = -sb;     R_target[7] = cb * sc;                R_target[8] = cb * cc;
    }

    OrientationError(_currentPose.R, R_target, &_dx[3]);
}


// ========================== Adaptive Damping ==========================

float DLS_IK_Solver::ComputeAdaptiveDamping(float _manipulability)
{
    if (_manipulability >= config.manip_threshold)
        return config.lambda_min;

    // Smooth transition: Nakamura & Hanafusa damping
    float ratio = _manipulability / config.manip_threshold;
    float lambda_sq = config.lambda_max * config.lambda_max * (1.0f - ratio * ratio);
    float result;
    arm_sqrt_f32(lambda_sq, &result);
    return result;
}


// ========================== 6x6 Solver (Cholesky) ==========================

bool DLS_IK_Solver::Solve6x6(const float A[36], const float b[6], float x[6])
{
    // Cholesky decomposition: A = L * L^T (A must be symmetric positive definite)
    float L[36];
    memset(L, 0, sizeof(L));

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            float sum = 0;
            for (int k = 0; k < j; k++)
                sum += L[i * 6 + k] * L[j * 6 + k];

            if (i == j)
            {
                float val = A[i * 6 + i] - sum;
                if (val <= 0) return false; // Not positive definite
                L[i * 6 + j] = sqrtf(val);
            }
            else
            {
                L[i * 6 + j] = (A[i * 6 + j] - sum) / L[j * 6 + j];
            }
        }
    }

    // Forward substitution: L * y = b
    float y[6];
    for (int i = 0; i < 6; i++)
    {
        float sum = 0;
        for (int k = 0; k < i; k++)
            sum += L[i * 6 + k] * y[k];
        y[i] = (b[i] - sum) / L[i * 6 + i];
    }

    // Backward substitution: L^T * x = y
    for (int i = 5; i >= 0; i--)
    {
        float sum = 0;
        for (int k = i + 1; k < 6; k++)
            sum += L[k * 6 + i] * x[k]; // L^T[i][k] = L[k][i]
        x[i] = (y[i] - sum) / L[i * 6 + i];
    }

    return true;
}


// ========================== Joint Limit Avoidance ==========================

void DLS_IK_Solver::ComputeJointLimitGradient(const float _q[N], float _grad[N])
{
    if (!limitsSet)
    {
        memset(_grad, 0, N * sizeof(float));
        return;
    }

    // Gradient of joint limit cost function:
    // H(q) = sum_i (1/2n) * ((q_i - q_mid_i) / (q_max_i - q_min_i))^(2n)
    // dH/dq_i pushes joints toward center of range
    for (int i = 0; i < N; i++)
    {
        float range = jointLimits.max[i] - jointLimits.min[i];
        if (range < 1e-3f)
        {
            _grad[i] = 0;
            continue;
        }

        float mid = (jointLimits.max[i] + jointLimits.min[i]) * 0.5f;
        float normalized = (_q[i] - mid) / range; // [-0.5, 0.5]

        // Stronger repulsion near limits (cubic gradient)
        float margin_high = (jointLimits.max[i] - _q[i]) / range;
        float margin_low = (_q[i] - jointLimits.min[i]) / range;

        _grad[i] = 0;
        if (margin_high < 0.15f)
            _grad[i] -= config.joint_limit_gain / (margin_high + 0.01f);
        if (margin_low < 0.15f)
            _grad[i] += config.joint_limit_gain / (margin_low + 0.01f);
    }
}


void DLS_IK_Solver::ClampJoints(float _q[N])
{
    if (!limitsSet) return;
    for (int i = 0; i < N; i++)
    {
        if (_q[i] < jointLimits.min[i]) _q[i] = jointLimits.min[i];
        if (_q[i] > jointLimits.max[i]) _q[i] = jointLimits.max[i];
    }
}


// ========================== Main DLS-IK Solver ==========================

bool DLS_IK_Solver::Solve(const DOF6Kinematic::Pose6D_t &_targetPose,
                           const float _initialJoints[N],
                           Result_t &_result)
{
    if (!kinSolver || !dynSolver) return false;

    float q[N];
    memcpy(q, _initialJoints, N * sizeof(float));

    DOF6Kinematic::Pose6D_t currentPose{};
    DOF6Kinematic::Joint6D_t jointStruct{};

    for (int iter = 0; iter < config.max_iterations; iter++)
    {
        // 1. Compute current FK pose
        for (int i = 0; i < N; i++) jointStruct.a[i] = q[i];
        kinSolver->SolveFK(jointStruct, currentPose);
        // Convert FK output (meters) to mm for comparison
        currentPose.X *= 1000.0f;
        currentPose.Y *= 1000.0f;
        currentPose.Z *= 1000.0f;

        // 2. Compute pose error
        float dx[6];
        ComputePoseError(currentPose, _targetPose, dx);

        // Scale position error from mm to m for Jacobian compatibility
        dx[0] *= 0.001f;
        dx[1] *= 0.001f;
        dx[2] *= 0.001f;

        // 3. Check convergence
        float posErr = sqrtf(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]) * 1000.0f; // back to mm
        float rotErr = sqrtf(dx[3] * dx[3] + dx[4] * dx[4] + dx[5] * dx[5]);

        if (posErr < config.pos_tolerance && rotErr < config.rot_tolerance)
        {
            memcpy(_result.joints, q, N * sizeof(float));
            _result.posError = posErr;
            _result.rotError = rotErr;
            _result.iterations = iter;
            _result.converged = true;
            _result.manipulability = dynSolver->ComputeManipulability(q);
            return true;
        }

        // 4. Compute Jacobian (6x6)
        float J[36];
        dynSolver->SolveJacobian(q, J);

        // 5. Compute manipulability and adaptive damping
        float manip = dynSolver->ComputeManipulability(q);
        float lambda = ComputeAdaptiveDamping(manip);

        // 6. Compute JJt = J * J^T + lambda^2 * I (6x6)
        float JJt[36];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                JJt[i * 6 + j] = 0;
                for (int k = 0; k < 6; k++)
                    JJt[i * 6 + j] += J[i * 6 + k] * J[j * 6 + k];
            }
            JJt[i * 6 + i] += lambda * lambda; // Damping
        }

        // 7. Solve (J*J^T + lambda^2*I) * y = dx
        float y[6];
        if (!Solve6x6(JJt, dx, y))
        {
            // Fallback: increase damping
            lambda = config.lambda_max;
            for (int i = 0; i < 6; i++)
                JJt[i * 6 + i] += lambda * lambda;
            if (!Solve6x6(JJt, dx, y))
                break; // Cannot solve
        }

        // 8. dq = J^T * y (joint angle update in rad)
        float dq_rad[N];
        for (int i = 0; i < N; i++)
        {
            dq_rad[i] = 0;
            for (int k = 0; k < 6; k++)
                dq_rad[i] += J[k * 6 + i] * y[k];
        }

        // Convert to degrees
        float dq_deg[N];
        float RAD2DEG = 57.295777754771045f;
        for (int i = 0; i < N; i++)
            dq_deg[i] = dq_rad[i] * RAD2DEG;

        // 9. Joint limit avoidance (null-space projection)
        if (config.joint_limit_avoidance && limitsSet)
        {
            float grad[N];
            ComputeJointLimitGradient(q, grad);

            // Null-space projector: N = I - J^+ * J
            // Simplified: just add gradient directly (approximate)
            for (int i = 0; i < N; i++)
                dq_deg[i] += grad[i];
        }

        // 10. Apply step with line search / step size control
        float alpha = config.step_size;

        // Limit maximum step size to 15 degrees per joint per iteration
        float maxStep = 0;
        for (int i = 0; i < N; i++)
        {
            if (fabsf(dq_deg[i]) > maxStep)
                maxStep = fabsf(dq_deg[i]);
        }
        if (maxStep > 15.0f)
            alpha *= 15.0f / maxStep;

        for (int i = 0; i < N; i++)
            q[i] += alpha * dq_deg[i];

        // 11. Clamp to joint limits
        ClampJoints(q);
    }

    // Did not converge, but return best result
    for (int i = 0; i < N; i++) jointStruct.a[i] = q[i];
    kinSolver->SolveFK(jointStruct, currentPose);
    currentPose.X *= 1000.0f;
    currentPose.Y *= 1000.0f;
    currentPose.Z *= 1000.0f;

    float dx[6];
    ComputePoseError(currentPose, _targetPose, dx);

    memcpy(_result.joints, q, N * sizeof(float));
    _result.posError = sqrtf(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]);
    _result.rotError = sqrtf(dx[3] * dx[3] + dx[4] * dx[4] + dx[5] * dx[5]);
    _result.iterations = config.max_iterations;
    _result.converged = false;
    _result.manipulability = dynSolver->ComputeManipulability(q);

    return false;
}


// ========================== Hybrid Solver ==========================

bool DLS_IK_Solver::SolveHybrid(const DOF6Kinematic::Pose6D_t &_targetPose,
                                  const float _currentJoints[N],
                                  Result_t &_result)
{
    if (!kinSolver || !dynSolver) return false;

    // Step 1: Try analytical IK first (fast)
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint{};
    for (int i = 0; i < N; i++) lastJoint.a[i] = _currentJoints[i];

    kinSolver->SolveIK(_targetPose, lastJoint, ikSolves);

    // Find best analytical solution
    float bestAnalyticalJoints[N];
    float minMaxAngle = 1e6f;
    int bestConfig = -1;

    for (int c = 0; c < 8; c++)
    {
        bool valid = true;
        if (limitsSet)
        {
            for (int j = 0; j < N; j++)
            {
                if (ikSolves.config[c].a[j] < jointLimits.min[j] ||
                    ikSolves.config[c].a[j] > jointLimits.max[j])
                {
                    valid = false;
                    break;
                }
            }
        }
        if (!valid) continue;

        float maxAngle = 0;
        for (int j = 0; j < N; j++)
        {
            float diff = fabsf(ikSolves.config[c].a[j] - _currentJoints[j]);
            if (diff > maxAngle) maxAngle = diff;
        }
        if (maxAngle < minMaxAngle)
        {
            minMaxAngle = maxAngle;
            bestConfig = c;
        }
    }

    if (bestConfig >= 0)
    {
        for (int i = 0; i < N; i++)
            bestAnalyticalJoints[i] = ikSolves.config[bestConfig].a[i];

        // Check manipulability at analytical solution
        float manip = dynSolver->ComputeManipulability(bestAnalyticalJoints);

        if (manip > config.manip_threshold)
        {
            // Good solution, use analytical result directly
            memcpy(_result.joints, bestAnalyticalJoints, N * sizeof(float));
            _result.manipulability = manip;
            _result.converged = true;
            _result.iterations = 0;

            // Compute actual error for reporting
            DOF6Kinematic::Joint6D_t js{};
            DOF6Kinematic::Pose6D_t p{};
            for (int i = 0; i < N; i++) js.a[i] = bestAnalyticalJoints[i];
            kinSolver->SolveFK(js, p);
            p.X *= 1000; p.Y *= 1000; p.Z *= 1000;
            float dx[6];
            ComputePoseError(p, _targetPose, dx);
            _result.posError = sqrtf(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]);
            _result.rotError = sqrtf(dx[3] * dx[3] + dx[4] * dx[4] + dx[5] * dx[5]);

            return true;
        }

        // Near singularity: use analytical as initial guess for DLS refinement
        return Solve(_targetPose, bestAnalyticalJoints, _result);
    }

    // No valid analytical solution: use DLS from current joints
    return Solve(_targetPose, _currentJoints, _result);
}
