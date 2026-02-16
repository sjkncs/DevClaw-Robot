#ifndef PARAM_IDENTIFIER_H
#define PARAM_IDENTIFIER_H

#include <cstdint>

/**
 * @brief Dynamic Parameter Identification for 6-DOF Robot
 *
 * Identifies the 10 standard inertial parameters per link:
 *   [m, m*cx, m*cy, m*cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
 * Plus 2 friction parameters per joint: [fv, fc]
 * Total: 10*6 + 2*6 = 72 parameters (reducible to ~40 base parameters)
 *
 * Method: Weighted Least Squares (WLS) on the linear regression form:
 *   tau = Y(q, dq, ddq) * pi + epsilon
 *
 * Where:
 *   tau  = measured/estimated joint torques (N*m)
 *   Y    = regressor matrix (depends only on kinematics, not on inertial params)
 *   pi   = parameter vector to identify
 *   epsilon = noise
 *
 * The regressor Y is computed from the RNEA algorithm by replacing
 * physical operations with symbolic "regressor column" operations.
 *
 * Excitation trajectory requirements:
 *   - Must excite all parameters (persistently exciting)
 *   - Fourier-based trajectories: q(t) = sum(ak*sin(wk*t) + bk*cos(wk*t))
 *   - Optimized via condition number minimization of Y
 *
 * Kalman smoother provides clean q, dq, ddq for the regressor.
 *
 * Reference:
 *   Khalil & Dombre, "Modeling, Identification & Control of Robots", Ch.13
 *   Swevers et al., "Optimal Robot Excitation and Identification", IEEE TRA, 1997
 *   Atkeson et al., "Estimation of Inertial Parameters", IJRR, 1986
 *
 * Target: IEEE RA-L / T-Mech (combined with DOB paper)
 */
class ParamIdentifier
{
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr int PARAMS_PER_LINK = 10;  // Standard inertial params
    static constexpr int FRICTION_PER_JOINT = 2; // Viscous + Coulomb
    static constexpr int TOTAL_PARAMS = NUM_JOINTS * PARAMS_PER_LINK + NUM_JOINTS * FRICTION_PER_JOINT;
    // = 72, but many are not identifiable. Base parameters typically ~36-42.

    // For practical purposes, we identify a reduced set:
    // Per joint: [m*lc (gravity param), I_motor (reflected inertia), fv, fc]
    // = 4 * 6 = 24 parameters (all identifiable)
    static constexpr int REDUCED_PARAMS = 4 * NUM_JOINTS; // = 24

    static constexpr int MAX_SAMPLES = 4000;  // ~2s at 2kHz sampling

    struct Sample_t
    {
        float q[NUM_JOINTS];       // Joint angles (rad)
        float dq[NUM_JOINTS];      // Joint velocities (rad/s)
        float ddq[NUM_JOINTS];     // Joint accelerations (rad/s^2)
        float tau[NUM_JOINTS];     // Joint torques (N*m) or current
        float weight;              // Sample weight (from velocity: higher speed = more reliable)
    };

    struct IdentResult_t
    {
        float params[REDUCED_PARAMS]; // Identified parameters
        float residual;               // Weighted least-squares residual
        float conditionNumber;        // Condition number of regressor matrix
        int numSamples;
        bool valid;

        // Decoded per-joint parameters
        float gravityParam[NUM_JOINTS];    // m*lc for each joint
        float reflectedInertia[NUM_JOINTS]; // Reflected inertia
        float viscousFriction[NUM_JOINTS];  // Viscous friction coeff
        float coulombFriction[NUM_JOINTS];  // Coulomb friction coeff
    };

    // ==================== Excitation Trajectory ====================

    struct FourierTrajectoryConfig_t
    {
        int numHarmonics;           // Number of Fourier harmonics (default 5)
        float fundamentalFreq;      // Fundamental frequency (Hz, default 0.1)
        float duration;             // Total trajectory duration (s)
        float amplitudes[NUM_JOINTS][10]; // Fourier amplitudes per joint per harmonic
        float jointLimitsMin[NUM_JOINTS];
        float jointLimitsMax[NUM_JOINTS];
        float velLimits[NUM_JOINTS];
        float accLimits[NUM_JOINTS];
    };

    ParamIdentifier();

    /**
     * @brief Generate optimized Fourier excitation trajectory
     * @param _config  Trajectory configuration
     * @param _numPoints  Number of trajectory points to generate
     * @param _qOut    Output: joint angles [numPoints][6] (deg)
     * @param _dqOut   Output: joint velocities [numPoints][6] (deg/s)
     * @param _ddqOut  Output: joint accelerations [numPoints][6] (deg/s^2)
     * @return Number of valid points generated
     */
    int GenerateExcitationTrajectory(const FourierTrajectoryConfig_t &_config,
                                      int _numPoints,
                                      float _qOut[][NUM_JOINTS],
                                      float _dqOut[][NUM_JOINTS],
                                      float _ddqOut[][NUM_JOINTS]);

    /**
     * @brief Record a data sample during excitation
     */
    void RecordSample(const float _q[NUM_JOINTS],
                       const float _dq[NUM_JOINTS],
                       const float _ddq[NUM_JOINTS],
                       const float _tau[NUM_JOINTS]);

    /**
     * @brief Run weighted least-squares identification
     * @return Identification result
     */
    IdentResult_t Identify();

    /**
     * @brief Reset recorded samples
     */
    void ResetSamples() { sampleCount = 0; }

    /**
     * @brief Get number of recorded samples
     */
    int GetSampleCount() const { return sampleCount; }

    /**
     * @brief Set DH parameters for regressor computation
     */
    void SetDHParams(const float _dh[NUM_JOINTS][4]);

    /**
     * @brief Set gear reduction ratios
     */
    void SetReductionRatios(const float _ratios[NUM_JOINTS]);

private:
    Sample_t samples[MAX_SAMPLES];
    int sampleCount = 0;

    float DH[NUM_JOINTS][4];     // DH parameters
    float reductions[NUM_JOINTS]; // Gear ratios

    /**
     * @brief Build reduced regressor row for one sample
     *
     * For the reduced parameter set [m*lc, I_reflected, fv, fc] per joint:
     *   tau_i = I_i * ddq_i + g_i(q) * m_i*lc_i + fv_i * dq_i + fc_i * sign(dq_i)
     *
     * This gives a 6x24 regressor block per sample.
     *
     * @param _q, _dq, _ddq  Kinematic data (rad, rad/s, rad/s^2)
     * @param _Y  Output: 6 x REDUCED_PARAMS regressor row block
     */
    void BuildReducedRegressor(const float _q[NUM_JOINTS],
                                const float _dq[NUM_JOINTS],
                                const float _ddq[NUM_JOINTS],
                                float _Y[NUM_JOINTS * REDUCED_PARAMS]);

    /**
     * @brief Compute gravity regressor columns using numerical differentiation
     *        g_i(q) = dV/dq_i where V is potential energy
     */
    void ComputeGravityRegressor(const float _q[NUM_JOINTS],
                                  float _grav[NUM_JOINTS][NUM_JOINTS]);

    /**
     * @brief Solve WLS: min || W^{1/2} * (Y*pi - tau) ||^2
     *        Solution: pi = (Y^T W Y)^{-1} Y^T W tau
     *
     * Uses normal equations with Cholesky decomposition.
     *
     * @param _YtWY   Normal matrix (REDUCED_PARAMS x REDUCED_PARAMS)
     * @param _YtWtau Right-hand side (REDUCED_PARAMS x 1)
     * @param _pi     Output: parameter vector
     * @return true if solvable
     */
    bool SolveNormalEquations(const float *_YtWY, const float *_YtWtau, float *_pi);

    /**
     * @brief Cholesky solver for symmetric positive definite NxN system
     */
    static bool CholeskySolve(const float *A, const float *b, float *x, int N);
};

#endif // PARAM_IDENTIFIER_H
