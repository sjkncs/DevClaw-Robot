#ifndef DOF6_DYNAMICS_SOLVER_H
#define DOF6_DYNAMICS_SOLVER_H

#include "stm32f405xx.h"
#include "arm_math.h"
#include "memory.h"
#include "algorithms/kinematic/6dof_kinematic.h"

/**
 * @brief 6-DOF Robot Dynamics Solver using Recursive Newton-Euler Algorithm (RNEA)
 *
 * Computes inverse dynamics: given q, dq, ddq -> tau
 * Computes gravity compensation: given q -> tau_gravity
 * Computes Coriolis/centrifugal torques: given q, dq -> tau_coriolis
 *
 * Reference: Craig, "Introduction to Robotics: Mechanics and Control", Ch.6
 *
 * DH Convention (same as 6dof_kinematic):
 *   Joint i: theta_home[i], d[i], a[i], alpha[i]
 *
 * Link dynamics parameters (per link):
 *   mass, center of mass (in link frame), inertia tensor (3x3, in link frame)
 *   viscous friction coefficient, Coulomb friction coefficient
 */
class DOF6Dynamics
{
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr float GRAVITY = 9.81f;

    struct LinkParam_t
    {
        float mass;          // (kg)
        float com[3];        // center of mass in link frame (m)
        float inertia[9];    // 3x3 inertia tensor in link frame (kg*m^2), row-major
        float friction_v;    // viscous friction coefficient (N*m*s/rad)
        float friction_c;    // Coulomb friction coefficient (N*m)
    };

    struct DynConfig_t
    {
        LinkParam_t links[NUM_JOINTS];
        float gravity_vec[3]; // gravity vector in base frame, default {0, 0, -9.81}
    };

    /**
     * @brief Construct dynamics solver with DH parameters (same as kinematic solver)
     */
    DOF6Dynamics(float L_BS, float D_BS, float L_AM, float L_FA, float D_EW, float L_WT);

    /**
     * @brief Set link dynamic parameters (mass, CoM, inertia, friction)
     */
    void SetLinkParams(const DynConfig_t &_config);

    /**
     * @brief Set link params for a single joint (0-indexed)
     */
    void SetSingleLinkParam(int _jointIdx, const LinkParam_t &_param);

    /**
     * @brief Inverse dynamics via RNEA: compute required joint torques
     * @param _q       Joint angles (deg)
     * @param _dq      Joint velocities (deg/s)
     * @param _ddq     Joint accelerations (deg/s^2)
     * @param _tau     Output joint torques (N*m), after gear reduction
     */
    void SolveInverseDynamics(const float _q[NUM_JOINTS],
                              const float _dq[NUM_JOINTS],
                              const float _ddq[NUM_JOINTS],
                              float _tau[NUM_JOINTS]);

    /**
     * @brief Compute gravity compensation torques only (dq=0, ddq=0)
     * @param _q       Joint angles (deg)
     * @param _tau_g   Output gravity torques (N*m)
     */
    void SolveGravityCompensation(const float _q[NUM_JOINTS],
                                  float _tau_g[NUM_JOINTS]);

    /**
     * @brief Compute Coriolis + centrifugal torques (no gravity, no acceleration)
     * @param _q       Joint angles (deg)
     * @param _dq      Joint velocities (deg/s)
     * @param _tau_c   Output Coriolis torques (N*m)
     */
    void SolveCoriolisTorques(const float _q[NUM_JOINTS],
                              const float _dq[NUM_JOINTS],
                              float _tau_c[NUM_JOINTS]);

    /**
     * @brief Compute 6x6 mass/inertia matrix M(q) using column-by-column RNEA
     *        M(:,k) = RNEA(q, 0, e_k) - RNEA(q, 0, 0)
     * @param _q       Joint angles (deg)
     * @param _M       Output 6x6 mass matrix (row-major)
     */
    void SolveMassMatrix(const float _q[NUM_JOINTS],
                         float _M[NUM_JOINTS * NUM_JOINTS]);

    /**
     * @brief Compute Jacobian matrix (geometric Jacobian in base frame)
     * @param _q       Joint angles (deg)
     * @param _J       Output 6x6 Jacobian (row-major), top 3 rows = linear, bottom 3 = angular
     */
    void SolveJacobian(const float _q[NUM_JOINTS],
                        float _J[NUM_JOINTS * NUM_JOINTS]);

    /**
     * @brief Compute manipulability measure w(q) = sqrt(det(J*J^T))
     * @param _q       Joint angles (deg)
     * @return manipulability index
     */
    float ComputeManipulability(const float _q[NUM_JOINTS]);


private:
    const float DEG_TO_RAD = 0.01745329251994f;
    const float RAD_TO_DEG = 57.295777754771045f;

    // DH parameters: [theta_home, d, a, alpha] for each joint
    float DH_matrix[NUM_JOINTS][4];

    // Link dynamic parameters
    DynConfig_t dynConfig;
    bool paramsSet = false;

    // Intermediate RNEA variables (pre-allocated to avoid stack allocation in real-time)
    // Forward recursion: i from 0 to 5
    float omega[NUM_JOINTS + 1][3];      // angular velocity of frame i (in frame i)
    float alpha_ang[NUM_JOINTS + 1][3];  // angular acceleration of frame i (in frame i)
    float a_linear[NUM_JOINTS + 1][3];   // linear acceleration of origin of frame i (in frame i)
    float a_com[NUM_JOINTS][3];          // linear acceleration of CoM of link i (in frame i)
    float f_link[NUM_JOINTS][3];         // net force on link i (in frame i)
    float n_link[NUM_JOINTS][3];         // net torque on link i (in frame i)

    // Backward recursion: i from 5 to 0
    float f_joint[NUM_JOINTS + 1][3];    // force exerted on link i by link i-1 (in frame i)
    float n_joint[NUM_JOINTS + 1][3];    // torque exerted on link i by link i-1 (in frame i)

    // Rotation matrices R_{i}^{i-1} and position vectors p_{i}^{i-1}
    float R_prev[NUM_JOINTS][9];   // R from frame i to frame i-1 (3x3 row-major)
    float R_next[NUM_JOINTS][9];   // R from frame i-1 to frame i (transpose of above)
    float p_org[NUM_JOINTS][3];    // origin of frame i in frame i-1

    // Arm geometry (same as kinematic solver)
    struct ArmConfig_t
    {
        float L_BASE;
        float D_BASE;
        float L_ARM;
        float L_FOREARM;
        float D_ELBOW;
        float L_WRIST;
    };
    ArmConfig_t armConfig;

    /**
     * @brief Core RNEA computation
     * @param q_rad    Joint angles in radians
     * @param dq_rad   Joint velocities in rad/s
     * @param ddq_rad  Joint accelerations in rad/s^2
     * @param grav     Gravity vector in base frame
     * @param tau      Output torques in N*m
     */
    void RNEA_Core(const float q_rad[NUM_JOINTS],
                   const float dq_rad[NUM_JOINTS],
                   const float ddq_rad[NUM_JOINTS],
                   const float grav[3],
                   float tau[NUM_JOINTS]);

    /**
     * @brief Compute rotation matrix and position vector for joint i given angle q
     */
    void ComputeTransform(int i, float q_rad, float R_i_to_prev[9], float p_i_in_prev[3]);

    // Helper math functions
    static void Vec3Cross(const float a[3], const float b[3], float out[3]);
    static void Vec3Add(const float a[3], const float b[3], float out[3]);
    static void Vec3Sub(const float a[3], const float b[3], float out[3]);
    static void Vec3Scale(const float a[3], float s, float out[3]);
    static float Vec3Dot(const float a[3], const float b[3]);
    static void Mat3Vec3Mul(const float M[9], const float v[3], float out[3]);
    static void Mat3Transpose(const float M[9], float Mt[9]);
    static void Mat3Mul(const float A[9], const float B[9], float C[9]);
    static float Mat6Det(const float M[36]);
};

#endif // DOF6_DYNAMICS_SOLVER_H
