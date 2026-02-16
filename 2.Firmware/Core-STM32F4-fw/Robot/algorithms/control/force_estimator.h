#ifndef FORCE_ESTIMATOR_H
#define FORCE_ESTIMATOR_H

#include <cstdint>

/**
 * @brief Sensorless External Force/Torque Estimator
 *
 * Estimates external joint torques and Cartesian forces WITHOUT a F/T sensor
 * by comparing the commanded/measured motor torques against the model-predicted
 * torques from RNEA inverse dynamics.
 *
 * === Joint-Space Estimation ===
 *   tau_ext = tau_motor - tau_model
 *   tau_model = M(q)*ddq + C(q,dq)*dq + g(q) + f(dq)
 *
 * where:
 *   tau_motor = Kt * i_motor / gear_ratio  (from motor current measurement)
 *   tau_model = RNEA(q, dq, ddq)            (from dynamics model)
 *   f(dq) = fv*dq + fc*sign(dq)             (friction model)
 *
 * === Cartesian-Space Mapping ===
 *   F_ext = J^{-T} * tau_ext               (Jacobian transpose mapping)
 *
 * === Filtering ===
 *   Low-pass filter on tau_ext to reduce noise from acceleration estimation
 *   Deadzone to eliminate bias drift
 *
 * Dependencies:
 *   - KF-estimated velocity and acceleration (Phase 2A)
 *   - RNEA dynamics solver (Phase 1A)
 *   - Identified friction parameters (Phase 2D)
 *
 * Reference:
 *   Haddadin et al., "Robot Collisions: A Survey", IEEE TRO, 2017
 *   De Luca & Mattone, "Sensorless Robot Collision Detection and Hybrid
 *     Force/Motion Control", ICRA, 2005
 *
 * Target: IEEE RA-L / pedestrians
 */
class ForceEstimator
{
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr int CART_DOF = 6; // [Fx, Fy, Fz, Tx, Ty, Tz]

    struct Config_t
    {
        float dt;                           // Sample period (s)
        float motorKt[NUM_JOINTS];          // Torque constant per motor (N*m/A)
        float gearRatio[NUM_JOINTS];        // Gear reduction ratio
        float filterCutoff;                 // Low-pass filter cutoff (Hz)
        float deadzone[NUM_JOINTS];         // Per-joint deadzone (N*m)
        float frictionV[NUM_JOINTS];        // Viscous friction coeff
        float frictionC[NUM_JOINTS];        // Coulomb friction coeff
    };

    struct JointForce_t
    {
        float tauExt[NUM_JOINTS];           // Estimated external joint torque (N*m)
        float tauMotor[NUM_JOINTS];         // Motor-side torque (N*m)
        float tauModel[NUM_JOINTS];         // Model-predicted torque (N*m)
        float tauFriction[NUM_JOINTS];      // Friction torque (N*m)
        float confidence[NUM_JOINTS];       // Estimation confidence [0-1]
    };

    struct CartesianForce_t
    {
        float Fx, Fy, Fz;                  // External forces (N)
        float Tx, Ty, Tz;                  // External torques (N*m)
        float magnitude;                    // ||F|| (N)
    };

    ForceEstimator();

    /**
     * @brief Initialize with default parameters for DevClaw Robot
     */
    void InitDefault(float _dt);

    /**
     * @brief Initialize with custom configuration
     */
    void Init(const Config_t &_config);

    /**
     * @brief Set identified friction parameters (from ParamIdentifier)
     */
    void SetFrictionParams(const float _fv[NUM_JOINTS], const float _fc[NUM_JOINTS]);

    /**
     * @brief Main estimation tick
     *
     * @param _motorCurrent   Motor phase currents (A) per joint
     * @param _q              Joint positions (rad)
     * @param _dq             Joint velocities (rad/s) — from KF
     * @param _ddq            Joint accelerations (rad/s^2) — from KF
     * @param _tauRNEA        RNEA-predicted torque: M*ddq + C*dq + g (from dynamics solver)
     * @return Joint-space force estimate
     */
    JointForce_t Update(const float _motorCurrent[NUM_JOINTS],
                         const float _q[NUM_JOINTS],
                         const float _dq[NUM_JOINTS],
                         const float _ddq[NUM_JOINTS],
                         const float _tauRNEA[NUM_JOINTS]);

    /**
     * @brief Map joint torques to Cartesian forces using Jacobian
     *
     * F_ext = J^{-T} * tau_ext
     * For non-square J: F_ext = (J * J^T)^{-1} * J * tau_ext
     *
     * @param _jacobian   6x6 Jacobian matrix (row-major)
     * @return Cartesian force/torque estimate
     */
    CartesianForce_t MapToCartesian(const float _jacobian[36]);

    /**
     * @brief Get latest joint force estimate
     */
    const JointForce_t &GetJointForce() const { return jointForce; }

    /**
     * @brief Get latest Cartesian force estimate
     */
    const CartesianForce_t &GetCartesianForce() const { return cartForce; }

    /**
     * @brief Reset filter states
     */
    void Reset();

    /**
     * @brief Auto-calibrate deadzones from static data
     *        (robot must be stationary when called)
     */
    void CalibrateDeadzone(const float _motorCurrent[NUM_JOINTS],
                            const float _tauRNEA[NUM_JOINTS]);

private:
    Config_t config;
    JointForce_t jointForce;
    CartesianForce_t cartForce;

    // 2nd-order low-pass filter states per joint
    struct FilterState_t
    {
        float x1, x2;   // Filter states
        float y1, y2;   // Output history
    };
    FilterState_t filters[NUM_JOINTS];
    float filterA[3], filterB[3]; // Filter coefficients

    // Bias estimation (slow drift removal)
    float bias[NUM_JOINTS];
    float biasAlpha;   // Bias adaptation rate

    /**
     * @brief Compute 2nd-order Butterworth LP filter coefficients
     */
    void ComputeFilterCoeffs(float _cutoffHz, float _dt);

    /**
     * @brief Apply filter to single sample
     */
    float ApplyFilter(float _input, int _jointIdx);

    /**
     * @brief Compute friction torque from velocity
     */
    float ComputeFriction(float _dq, int _jointIdx) const;

    /**
     * @brief Solve 6x6 system for Cartesian force mapping
     */
    static bool Solve6x6(const float *A, const float *b, float *x);
};

#endif // FORCE_ESTIMATOR_H
