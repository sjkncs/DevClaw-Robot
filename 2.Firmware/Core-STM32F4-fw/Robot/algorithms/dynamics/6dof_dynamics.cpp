#include "6dof_dynamics.h"
#include <cmath>


// ========================== Helper Math Functions ==========================

inline float cosf_fast(float x) { return arm_cos_f32(x); }
inline float sinf_fast(float x) { return arm_sin_f32(x); }

void DOF6Dynamics::Vec3Cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

void DOF6Dynamics::Vec3Add(const float a[3], const float b[3], float out[3])
{
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

void DOF6Dynamics::Vec3Sub(const float a[3], const float b[3], float out[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

void DOF6Dynamics::Vec3Scale(const float a[3], float s, float out[3])
{
    out[0] = a[0] * s;
    out[1] = a[1] * s;
    out[2] = a[2] * s;
}

float DOF6Dynamics::Vec3Dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void DOF6Dynamics::Mat3Vec3Mul(const float M[9], const float v[3], float out[3])
{
    out[0] = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
    out[1] = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
    out[2] = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
}

void DOF6Dynamics::Mat3Transpose(const float M[9], float Mt[9])
{
    Mt[0] = M[0]; Mt[1] = M[3]; Mt[2] = M[6];
    Mt[3] = M[1]; Mt[4] = M[4]; Mt[5] = M[7];
    Mt[6] = M[2]; Mt[7] = M[5]; Mt[8] = M[8];
}

void DOF6Dynamics::Mat3Mul(const float A[9], const float B[9], float C[9])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C[3 * i + j] = 0;
            for (int k = 0; k < 3; k++)
                C[3 * i + j] += A[3 * i + k] * B[3 * k + j];
        }
    }
}


// ========================== Constructor ==========================

DOF6Dynamics::DOF6Dynamics(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT)
    : armConfig({L_BS, D_BS, L_AM, L_FA, D_EW, L_WT})
{
    // DH parameters: [theta_home, d, a, alpha] - same as 6dof_kinematic.cpp
    float tmp_DH[6][4] = {
        {0.0f,            armConfig.L_BASE,    armConfig.D_BASE,  -(float) M_PI_2},
        {-(float) M_PI_2, 0.0f,                armConfig.L_ARM,   0.0f},
        {(float) M_PI_2,  armConfig.D_ELBOW,   0.0f,              (float) M_PI_2},
        {0.0f,            armConfig.L_FOREARM,  0.0f,              -(float) M_PI_2},
        {0.0f,            0.0f,                 0.0f,              (float) M_PI_2},
        {0.0f,            armConfig.L_WRIST,    0.0f,              0.0f}
    };
    memcpy(DH_matrix, tmp_DH, sizeof(tmp_DH));

    // Default gravity vector (base frame Z-up)
    dynConfig.gravity_vec[0] = 0.0f;
    dynConfig.gravity_vec[1] = 0.0f;
    dynConfig.gravity_vec[2] = -GRAVITY;

    // Default link parameters (reasonable estimates for DevClaw Robot)
    // These should be calibrated via system identification for best results
    // Link 1: Base rotation joint
    dynConfig.links[0] = {
        .mass = 0.5f,
        .com = {0.0f, 0.0f, -0.03f},
        .inertia = {5e-4f, 0, 0,  0, 5e-4f, 0,  0, 0, 3e-4f},
        .friction_v = 0.01f,
        .friction_c = 0.05f
    };
    // Link 2: Shoulder joint (heaviest arm link)
    dynConfig.links[1] = {
        .mass = 0.8f,
        .com = {0.073f, 0.0f, 0.0f},
        .inertia = {3e-4f, 0, 0,  0, 2e-3f, 0,  0, 0, 2e-3f},
        .friction_v = 0.02f,
        .friction_c = 0.08f
    };
    // Link 3: Elbow joint
    dynConfig.links[2] = {
        .mass = 0.6f,
        .com = {0.0f, 0.0f, 0.05f},
        .inertia = {8e-4f, 0, 0,  0, 8e-4f, 0,  0, 0, 2e-4f},
        .friction_v = 0.015f,
        .friction_c = 0.06f
    };
    // Link 4: Wrist 1
    dynConfig.links[3] = {
        .mass = 0.3f,
        .com = {0.0f, 0.0f, 0.0f},
        .inertia = {1e-4f, 0, 0,  0, 1e-4f, 0,  0, 0, 8e-5f},
        .friction_v = 0.008f,
        .friction_c = 0.03f
    };
    // Link 5: Wrist 2
    dynConfig.links[4] = {
        .mass = 0.2f,
        .com = {0.0f, 0.0f, 0.0f},
        .inertia = {6e-5f, 0, 0,  0, 6e-5f, 0,  0, 0, 5e-5f},
        .friction_v = 0.005f,
        .friction_c = 0.02f
    };
    // Link 6: Wrist 3 (end-effector)
    dynConfig.links[5] = {
        .mass = 0.15f,
        .com = {0.0f, 0.0f, 0.02f},
        .inertia = {3e-5f, 0, 0,  0, 3e-5f, 0,  0, 0, 2e-5f},
        .friction_v = 0.003f,
        .friction_c = 0.01f
    };

    // Zero out RNEA intermediate buffers
    memset(omega, 0, sizeof(omega));
    memset(alpha_ang, 0, sizeof(alpha_ang));
    memset(a_linear, 0, sizeof(a_linear));
    memset(a_com, 0, sizeof(a_com));
    memset(f_link, 0, sizeof(f_link));
    memset(n_link, 0, sizeof(n_link));
    memset(f_joint, 0, sizeof(f_joint));
    memset(n_joint, 0, sizeof(n_joint));

    paramsSet = true;
}


// ========================== Parameter Setting ==========================

void DOF6Dynamics::SetLinkParams(const DynConfig_t &_config)
{
    memcpy(&dynConfig, &_config, sizeof(DynConfig_t));
    paramsSet = true;
}

void DOF6Dynamics::SetSingleLinkParam(int _jointIdx, const LinkParam_t &_param)
{
    if (_jointIdx >= 0 && _jointIdx < NUM_JOINTS)
        memcpy(&dynConfig.links[_jointIdx], &_param, sizeof(LinkParam_t));
}


// ========================== Transform Computation ==========================

void DOF6Dynamics::ComputeTransform(int i, float q_rad, float R_i_to_prev[9], float p_i_in_prev[3])
{
    // DH transform from frame i to frame i-1:
    // R_{i-1,i} and p_{i}^{i-1}
    // Using standard DH convention:
    //   T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    //   where theta = q + theta_home

    float theta = q_rad + DH_matrix[i][0];
    float d     = DH_matrix[i][1];
    float a     = DH_matrix[i][2];
    float al    = DH_matrix[i][3];

    float ct = cosf_fast(theta);
    float st = sinf_fast(theta);
    float ca = cosf_fast(al);
    float sa = sinf_fast(al);

    // R_{i-1, i} (rotation from frame i expressed in frame i-1)
    R_i_to_prev[0] = ct;   R_i_to_prev[1] = -ca * st;  R_i_to_prev[2] = sa * st;
    R_i_to_prev[3] = st;   R_i_to_prev[4] = ca * ct;    R_i_to_prev[5] = -sa * ct;
    R_i_to_prev[6] = 0.0f; R_i_to_prev[7] = sa;         R_i_to_prev[8] = ca;

    // Position of origin of frame i in frame i-1
    p_i_in_prev[0] = a * ct;
    p_i_in_prev[1] = a * st;
    p_i_in_prev[2] = d;
}


// ========================== Core RNEA ==========================

void DOF6Dynamics::RNEA_Core(const float q_rad[NUM_JOINTS],
                              const float dq_rad[NUM_JOINTS],
                              const float ddq_rad[NUM_JOINTS],
                              const float grav[3],
                              float tau[NUM_JOINTS])
{
    // Pre-compute all transforms
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        ComputeTransform(i, q_rad[i], R_prev[i], p_org[i]);
        Mat3Transpose(R_prev[i], R_next[i]); // R_{i, i-1} = (R_{i-1, i})^T
    }

    // Initialize base frame (frame 0's "previous" = base)
    // omega_0 = 0, alpha_0 = 0
    // a_0 = -gravity (to absorb gravity into the acceleration term)
    omega[0][0] = 0; omega[0][1] = 0; omega[0][2] = 0;
    alpha_ang[0][0] = 0; alpha_ang[0][1] = 0; alpha_ang[0][2] = 0;
    // Acceleration of base = -gravity (Craig convention: treat gravity as upward base acceleration)
    a_linear[0][0] = -grav[0];
    a_linear[0][1] = -grav[1];
    a_linear[0][2] = -grav[2];

    // =================== Forward Recursion (base to tip) ===================
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        // All joints are revolute, rotation axis is Z of frame i
        // R_{i, i-1} rotates vectors from frame i-1 to frame i
        float z_axis[3] = {0.0f, 0.0f, 1.0f};

        // omega_i = R_{i,i-1} * omega_{i-1} + dq_i * z_i
        float Rw[3];
        Mat3Vec3Mul(R_next[i], omega[i], Rw);
        float dqz[3];
        Vec3Scale(z_axis, dq_rad[i], dqz);
        Vec3Add(Rw, dqz, omega[i + 1]);

        // alpha_i = R_{i,i-1} * alpha_{i-1} + dq_i * (R_{i,i-1}*omega_{i-1}) x z_i + ddq_i * z_i
        float Ra[3];
        Mat3Vec3Mul(R_next[i], alpha_ang[i], Ra);
        float cross_Rw_z[3];
        Vec3Cross(Rw, dqz, cross_Rw_z);
        float ddqz[3];
        Vec3Scale(z_axis, ddq_rad[i], ddqz);
        float tmp[3];
        Vec3Add(Ra, cross_Rw_z, tmp);
        Vec3Add(tmp, ddqz, alpha_ang[i + 1]);

        // a_i = R_{i,i-1} * (a_{i-1} + alpha_{i-1} x p_{i}^{i-1} + omega_{i-1} x (omega_{i-1} x p_{i}^{i-1}))
        // But p_org[i] is in frame i-1, so we need to transform
        float alpha_cross_p[3], omega_cross_p[3], omega_cross_omega_cross_p[3];
        Vec3Cross(alpha_ang[i], p_org[i], alpha_cross_p);
        Vec3Cross(omega[i], p_org[i], omega_cross_p);
        Vec3Cross(omega[i], omega_cross_p, omega_cross_omega_cross_p);

        float a_prev_sum[3];
        Vec3Add(a_linear[i], alpha_cross_p, tmp);
        Vec3Add(tmp, omega_cross_omega_cross_p, a_prev_sum);
        Mat3Vec3Mul(R_next[i], a_prev_sum, a_linear[i + 1]);

        // a_com_i = a_i + alpha_i x r_com_i + omega_i x (omega_i x r_com_i)
        float alpha_cross_rc[3], omega_cross_rc[3], omega_cross_omega_cross_rc[3];
        Vec3Cross(alpha_ang[i + 1], dynConfig.links[i].com, alpha_cross_rc);
        Vec3Cross(omega[i + 1], dynConfig.links[i].com, omega_cross_rc);
        Vec3Cross(omega[i + 1], omega_cross_rc, omega_cross_omega_cross_rc);
        Vec3Add(a_linear[i + 1], alpha_cross_rc, tmp);
        Vec3Add(tmp, omega_cross_omega_cross_rc, a_com[i]);

        // f_i = m_i * a_com_i
        Vec3Scale(a_com[i], dynConfig.links[i].mass, f_link[i]);

        // n_i = I_i * alpha_i + omega_i x (I_i * omega_i)
        float I_alpha[3], I_omega[3], omega_cross_Iomega[3];
        Mat3Vec3Mul(dynConfig.links[i].inertia, alpha_ang[i + 1], I_alpha);
        Mat3Vec3Mul(dynConfig.links[i].inertia, omega[i + 1], I_omega);
        Vec3Cross(omega[i + 1], I_omega, omega_cross_Iomega);
        Vec3Add(I_alpha, omega_cross_Iomega, n_link[i]);
    }

    // Initialize force/torque at the end-effector tip (no external load)
    f_joint[NUM_JOINTS][0] = 0; f_joint[NUM_JOINTS][1] = 0; f_joint[NUM_JOINTS][2] = 0;
    n_joint[NUM_JOINTS][0] = 0; n_joint[NUM_JOINTS][1] = 0; n_joint[NUM_JOINTS][2] = 0;

    // =================== Backward Recursion (tip to base) ===================
    for (int i = NUM_JOINTS - 1; i >= 0; i--)
    {
        // f_i = R_{i, i+1} * f_{i+1} + F_i
        // For the last link, f_{n+1} = 0 (no external force)
        // R_{i, i+1} = R_prev[i+1] if i < NUM_JOINTS-1
        float Rf_next[3] = {0, 0, 0};
        float Rn_next[3] = {0, 0, 0};

        if (i < NUM_JOINTS - 1)
        {
            Mat3Vec3Mul(R_prev[i + 1], f_joint[i + 1], Rf_next);
            Mat3Vec3Mul(R_prev[i + 1], n_joint[i + 1], Rn_next);
        }
        // else: Rf_next = Rn_next = 0 (tip)

        Vec3Add(Rf_next, f_link[i], f_joint[i]);

        // n_joint_i = n_link_i + R_{i,i+1}*n_{i+1} + r_com_i x F_i + p_{i+1}^{i} x (R_{i,i+1}*f_{i+1})
        float rc_cross_F[3], p_next_cross_Rf[3];
        Vec3Cross(dynConfig.links[i].com, f_link[i], rc_cross_F);

        // p_{i+1} in frame i: for the next joint
        float p_next_in_i[3] = {0, 0, 0};
        if (i < NUM_JOINTS - 1)
        {
            // p_{i+1} in frame i is just p_org[i+1] expressed in frame i
            // Since p_org[i+1] is origin of frame i+1 in frame i, and R_prev[i+1]
            // transforms from frame i+1 to frame i, the position is directly p_org[i+1]
            // but we need it in frame i. p_org is already in the parent frame.
            // Actually: p_org[i+1] is in frame i (the parent of i+1), so we use it directly.
            p_next_in_i[0] = p_org[i + 1][0];
            p_next_in_i[1] = p_org[i + 1][1];
            p_next_in_i[2] = p_org[i + 1][2];
        }

        // Wait - we need to reconsider. In Craig's formulation for DH:
        // p_{i+1}^{i} is the position of origin of frame i+1 expressed in frame i.
        // But p_org[i+1] is the origin of frame i+1 in frame i (its parent). Correct.
        // However, the DH transform gives us p in the parent frame (frame i), which is what we want.
        // But for backward recursion, we need R_{i,i+1} which rotates from i+1 to i.
        // R_{i,i+1} = R_prev[i+1] (R from frame i+1 to frame i).
        // This is correct since R_prev[i+1] = R_{i, i+1} by our ComputeTransform definition.

        Vec3Cross(p_next_in_i, Rf_next, p_next_cross_Rf);

        float tmp1[3], tmp2[3];
        Vec3Add(n_link[i], Rn_next, tmp1);
        Vec3Add(tmp1, rc_cross_F, tmp2);
        Vec3Add(tmp2, p_next_cross_Rf, n_joint[i]);

        // tau_i = n_joint_i . z_i + friction
        float z_axis[3] = {0.0f, 0.0f, 1.0f};
        tau[i] = Vec3Dot(n_joint[i], z_axis);

        // Add friction torques
        tau[i] += dynConfig.links[i].friction_v * dq_rad[i];
        if (dq_rad[i] > 0.001f)
            tau[i] += dynConfig.links[i].friction_c;
        else if (dq_rad[i] < -0.001f)
            tau[i] -= dynConfig.links[i].friction_c;
    }
}


// ========================== Public API ==========================

void DOF6Dynamics::SolveInverseDynamics(const float _q[NUM_JOINTS],
                                         const float _dq[NUM_JOINTS],
                                         const float _ddq[NUM_JOINTS],
                                         float _tau[NUM_JOINTS])
{
    float q_rad[NUM_JOINTS], dq_rad[NUM_JOINTS], ddq_rad[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        q_rad[i] = _q[i] * DEG_TO_RAD;
        dq_rad[i] = _dq[i] * DEG_TO_RAD;
        ddq_rad[i] = _ddq[i] * DEG_TO_RAD;
    }
    RNEA_Core(q_rad, dq_rad, ddq_rad, dynConfig.gravity_vec, _tau);
}

void DOF6Dynamics::SolveGravityCompensation(const float _q[NUM_JOINTS],
                                             float _tau_g[NUM_JOINTS])
{
    float q_rad[NUM_JOINTS];
    float zero[NUM_JOINTS] = {0};
    for (int i = 0; i < NUM_JOINTS; i++)
        q_rad[i] = _q[i] * DEG_TO_RAD;

    RNEA_Core(q_rad, zero, zero, dynConfig.gravity_vec, _tau_g);
}

void DOF6Dynamics::SolveCoriolisTorques(const float _q[NUM_JOINTS],
                                         const float _dq[NUM_JOINTS],
                                         float _tau_c[NUM_JOINTS])
{
    float q_rad[NUM_JOINTS], dq_rad[NUM_JOINTS];
    float zero[NUM_JOINTS] = {0};
    float zero_grav[3] = {0, 0, 0};
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        q_rad[i] = _q[i] * DEG_TO_RAD;
        dq_rad[i] = _dq[i] * DEG_TO_RAD;
    }
    RNEA_Core(q_rad, dq_rad, zero, zero_grav, _tau_c);
}

void DOF6Dynamics::SolveMassMatrix(const float _q[NUM_JOINTS],
                                    float _M[NUM_JOINTS * NUM_JOINTS])
{
    float q_rad[NUM_JOINTS];
    float zero[NUM_JOINTS] = {0};
    float zero_grav[3] = {0, 0, 0};
    float tau_bias[NUM_JOINTS];
    float tau_col[NUM_JOINTS];
    float ddq_unit[NUM_JOINTS] = {0};

    for (int i = 0; i < NUM_JOINTS; i++)
        q_rad[i] = _q[i] * DEG_TO_RAD;

    // Compute bias torque (gravity + Coriolis at zero acceleration)
    RNEA_Core(q_rad, zero, zero, zero_grav, tau_bias);

    // Compute each column: M(:,k) = RNEA(q, 0, e_k) - tau_bias
    for (int k = 0; k < NUM_JOINTS; k++)
    {
        memset(ddq_unit, 0, sizeof(ddq_unit));
        ddq_unit[k] = 1.0f; // unit acceleration in joint k (already in rad)

        RNEA_Core(q_rad, zero, ddq_unit, zero_grav, tau_col);

        for (int j = 0; j < NUM_JOINTS; j++)
            _M[j * NUM_JOINTS + k] = tau_col[j] - tau_bias[j];
    }
}

void DOF6Dynamics::SolveJacobian(const float _q[NUM_JOINTS],
                                  float _J[NUM_JOINTS * NUM_JOINTS])
{
    // Geometric Jacobian computation using FK chain
    // J = [Jv; Jw] where Jv = z_{i-1} x (p_e - p_{i-1}), Jw = z_{i-1}
    float q_rad[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++)
        q_rad[i] = _q[i] * DEG_TO_RAD;

    // Compute cumulative transforms: T_0^i for each frame
    float R_0i[NUM_JOINTS + 1][9];  // R from frame i to frame 0
    float p_0i[NUM_JOINTS + 1][3];  // Origin of frame i in frame 0

    // Base frame
    R_0i[0][0] = 1; R_0i[0][1] = 0; R_0i[0][2] = 0;
    R_0i[0][3] = 0; R_0i[0][4] = 1; R_0i[0][5] = 0;
    R_0i[0][6] = 0; R_0i[0][7] = 0; R_0i[0][8] = 1;
    p_0i[0][0] = 0; p_0i[0][1] = 0; p_0i[0][2] = 0;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        float Ri[9], pi[3];
        ComputeTransform(i, q_rad[i], Ri, pi);

        // R_0^{i+1} = R_0^i * R_i^{i+1}
        Mat3Mul(R_0i[i], Ri, R_0i[i + 1]);

        // p_0^{i+1} = p_0^i + R_0^i * p_{i+1}^i
        float Rp[3];
        Mat3Vec3Mul(R_0i[i], pi, Rp);
        Vec3Add(p_0i[i], Rp, p_0i[i + 1]);
    }

    // End-effector position
    float p_e[3];
    p_e[0] = p_0i[NUM_JOINTS][0];
    p_e[1] = p_0i[NUM_JOINTS][1];
    p_e[2] = p_0i[NUM_JOINTS][2];

    // Build Jacobian columns
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        // z_{i} = R_0^i * [0, 0, 1]^T (rotation axis of joint i in base frame)
        float z_i[3];
        z_i[0] = R_0i[i][2]; // Third column of R_0i[i]
        z_i[1] = R_0i[i][5];
        z_i[2] = R_0i[i][8];

        // p_e - p_i
        float dp[3];
        Vec3Sub(p_e, p_0i[i], dp);

        // Jv column i = z_i x (p_e - p_i)
        float jv[3];
        Vec3Cross(z_i, dp, jv);

        // Jv (top 3 rows)
        _J[0 * NUM_JOINTS + i] = jv[0];
        _J[1 * NUM_JOINTS + i] = jv[1];
        _J[2 * NUM_JOINTS + i] = jv[2];

        // Jw (bottom 3 rows) = z_i
        _J[3 * NUM_JOINTS + i] = z_i[0];
        _J[4 * NUM_JOINTS + i] = z_i[1];
        _J[5 * NUM_JOINTS + i] = z_i[2];
    }
}

float DOF6Dynamics::ComputeManipulability(const float _q[NUM_JOINTS])
{
    float J[36];
    SolveJacobian(_q, J);

    // w = sqrt(det(J * J^T))
    // Compute JJt = J * J^T (6x6)
    float JJt[36];
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            JJt[i * 6 + j] = 0;
            for (int k = 0; k < 6; k++)
                JJt[i * 6 + j] += J[i * 6 + k] * J[j * 6 + k];
        }
    }

    // Compute determinant of 6x6 matrix (LU decomposition)
    float det = Mat6Det(JJt);
    if (det < 0) det = 0;
    float result;
    arm_sqrt_f32(det, &result);
    return result;
}

float DOF6Dynamics::Mat6Det(const float M[36])
{
    // 6x6 determinant via LU decomposition (partial pivoting)
    float L[36] = {0};
    float U[36];
    memcpy(U, M, 36 * sizeof(float));
    float det = 1.0f;

    for (int k = 0; k < 6; k++)
    {
        // Find pivot
        float maxVal = fabsf(U[k * 6 + k]);
        int maxRow = k;
        for (int i = k + 1; i < 6; i++)
        {
            if (fabsf(U[i * 6 + k]) > maxVal)
            {
                maxVal = fabsf(U[i * 6 + k]);
                maxRow = i;
            }
        }

        if (maxVal < 1e-12f) return 0.0f; // Singular

        if (maxRow != k)
        {
            // Swap rows
            for (int j = 0; j < 6; j++)
            {
                float tmp = U[k * 6 + j];
                U[k * 6 + j] = U[maxRow * 6 + j];
                U[maxRow * 6 + j] = tmp;
            }
            det = -det; // Row swap changes sign
        }

        det *= U[k * 6 + k];

        for (int i = k + 1; i < 6; i++)
        {
            float factor = U[i * 6 + k] / U[k * 6 + k];
            for (int j = k; j < 6; j++)
                U[i * 6 + j] -= factor * U[k * 6 + j];
        }
    }

    return det;
}
