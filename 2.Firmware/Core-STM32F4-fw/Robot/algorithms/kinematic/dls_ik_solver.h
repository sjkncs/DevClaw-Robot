#ifndef DLS_IK_SOLVER_H
#define DLS_IK_SOLVER_H

#include "6dof_kinematic.h"
#include "algorithms/dynamics/6dof_dynamics.h"

/**
 * @brief Damped Least-Squares (Levenberg-Marquardt) Iterative IK Solver
 *        with Singularity-Robust Damping
 *
 * Solves IK iteratively:
 *   dq = J^T * (J*J^T + lambda^2 * I)^{-1} * dx
 *
 * When near singularity (low manipulability), lambda increases automatically
 * to trade accuracy for stability. Far from singularity, lambda -> 0 and
 * the solution converges to the standard pseudo-inverse.
 *
 * Also supports:
 *   - Joint limit avoidance via gradient projection
 *   - Manipulability-based adaptive damping (Nakamura & Hanafusa, 1986)
 *   - Weighted joint-space resolution
 *
 * Reference:
 *   Buss, "Introduction to Inverse Kinematics with Jacobian Transpose,
 *          Pseudoinverse and Damped Least Squares Methods", 2004
 *   Nakamura & Hanafusa, "Inverse Kinematic Solutions with Singularity
 *          Robustness for Robot Manipulator Control", JDSMC, 1986
 */
class DLS_IK_Solver
{
public:
    static constexpr int N = 6; // Number of joints = task DOF

    struct Config_t
    {
        float lambda_max;       // Maximum damping factor (default 0.5)
        float lambda_min;       // Minimum damping factor (default 0.001)
        float manip_threshold;  // Manipulability threshold to start damping (default 0.01)
        float pos_tolerance;    // Position convergence tolerance in mm (default 0.5)
        float rot_tolerance;    // Orientation convergence tolerance in rad (default 0.01)
        int max_iterations;     // Max iterations (default 50)
        float step_size;        // Step size alpha in (0, 1] (default 1.0)
        bool joint_limit_avoidance; // Enable joint limit avoidance (default true)
        float joint_limit_gain;     // Gain for joint limit avoidance (default 0.1)
    };

    struct JointLimits_t
    {
        float min[N];
        float max[N];
    };

    struct Result_t
    {
        float joints[N];       // Solution joint angles (deg)
        float posError;        // Final position error (mm)
        float rotError;        // Final orientation error (rad)
        float manipulability;  // Manipulability at solution
        int iterations;        // Iterations used
        bool converged;        // Whether it converged within tolerance
    };

    DLS_IK_Solver();

    /**
     * @brief Attach kinematic and dynamics solvers
     */
    void AttachSolvers(DOF6Kinematic *_kinSolver, DOF6Dynamics *_dynSolver);

    /**
     * @brief Set solver configuration
     */
    void SetConfig(const Config_t &_config);

    /**
     * @brief Set joint limits
     */
    void SetJointLimits(const JointLimits_t &_limits);

    /**
     * @brief Solve IK with DLS method
     * @param _targetPose    Desired end-effector pose (position in mm, angles in deg)
     * @param _initialJoints Initial guess for joint angles (deg)
     * @param _result        Output result
     * @return true if converged
     */
    bool Solve(const DOF6Kinematic::Pose6D_t &_targetPose,
               const float _initialJoints[N],
               Result_t &_result);

    /**
     * @brief Solve IK with DLS, using analytical IK as initial guess
     *        Falls back to DLS if analytical solution is near singularity
     * @param _targetPose    Desired pose
     * @param _currentJoints Current joint angles for continuity
     * @param _result        Output
     * @return true if a valid solution found
     */
    bool SolveHybrid(const DOF6Kinematic::Pose6D_t &_targetPose,
                     const float _currentJoints[N],
                     Result_t &_result);

private:
    DOF6Kinematic *kinSolver = nullptr;
    DOF6Dynamics *dynSolver = nullptr;

    Config_t config;
    JointLimits_t jointLimits;
    bool limitsSet = false;

    /**
     * @brief Compute 6D pose error (position + orientation) between current and target
     * @param _currentPose  Current FK pose
     * @param _targetPose   Target pose
     * @param _dx           Output 6D error vector [dp_x, dp_y, dp_z, dw_x, dw_y, dw_z]
     */
    void ComputePoseError(const DOF6Kinematic::Pose6D_t &_currentPose,
                          const DOF6Kinematic::Pose6D_t &_targetPose,
                          float _dx[N]);

    /**
     * @brief Compute adaptive damping factor based on manipulability
     *        lambda = lambda_max * sqrt(1 - (w/w0)^2)  when w < w0
     *        lambda = 0                                 when w >= w0
     */
    float ComputeAdaptiveDamping(float _manipulability);

    /**
     * @brief Solve 6x6 linear system (J*J^T + lambda^2*I) * x = b
     *        using Cholesky decomposition
     */
    bool Solve6x6(const float A[36], const float b[6], float x[6]);

    /**
     * @brief Compute joint limit avoidance gradient
     *        Pushes joints away from limits using the null-space
     */
    void ComputeJointLimitGradient(const float _q[N], float _grad[N]);

    /**
     * @brief Clamp joints to limits
     */
    void ClampJoints(float _q[N]);

    /**
     * @brief Compute orientation error using rotation matrix difference
     *        Returns angular velocity vector that would align current to target
     */
    static void OrientationError(const float R_current[9], const float R_target[9], float dw[3]);
};

#endif // DLS_IK_SOLVER_H
