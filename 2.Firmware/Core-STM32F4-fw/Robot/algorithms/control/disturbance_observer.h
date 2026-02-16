#ifndef DISTURBANCE_OBSERVER_H
#define DISTURBANCE_OBSERVER_H

#include <cstdint>

/**
 * @brief Disturbance Observer (DOB) for Robust Joint-Level Control
 *
 * Estimates and compensates for:
 *   - Model uncertainty (inaccurate dynamics parameters)
 *   - External disturbances (contact forces, payload changes)
 *   - Unmodeled friction (nonlinear, temperature-dependent)
 *
 * Architecture:
 *   d_hat = Q(s) * [tau_cmd - M_n * ddq] - (1-Q(s)) * tau_cmd
 *   tau_compensated = tau_cmd - d_hat
 *
 * Where Q(s) is a low-pass filter that determines DOB bandwidth.
 * Higher bandwidth -> faster disturbance rejection but more noise.
 *
 * Discrete implementation using 2nd-order Butterworth low-pass filter:
 *   Q(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 *
 * State-space form for real-time:
 *   x_k+1 = A_f * x_k + B_f * u_k
 *   y_k   = C_f * x_k + D_f * u_k
 *
 * Reference:
 *   Ohnishi, "A New Servo Method in Mechatronics", Trans. JIEE, 1987
 *   Sariyildiz & Ohnishi, "Stability and Robustness of DOB", IEEE TIE, 2015
 *
 * Target: IEEE T-Mech / RA-L
 */
class DisturbanceObserver
{
public:
    struct Config_t
    {
        float cutoffFreq;       // Q-filter cutoff frequency (Hz), default 50
        float sampleFreq;       // Control loop frequency (Hz), default 20000
        float nominalInertia;   // Nominal inertia M_n (kg*m^2 at motor side)
        float nominalFriction;  // Nominal viscous friction (N*m*s/rad)
        float disturbanceLimit; // Maximum disturbance estimate (N*m), for safety
        bool  enabled;
    };

    DisturbanceObserver();

    /**
     * @brief Initialize DOB with configuration
     */
    void Init(const Config_t &_config);

    /**
     * @brief Reset internal states
     */
    void Reset();

    /**
     * @brief Update DOB estimate
     * @param _tauCmd       Commanded torque (or current proportional to torque)
     * @param _velocity     Measured/estimated velocity (rad/s)
     * @param _acceleration Measured/estimated acceleration (rad/s^2), from KF
     * @return Estimated disturbance torque d_hat
     */
    float Update(float _tauCmd, float _velocity, float _acceleration);

    /**
     * @brief Get compensation torque to add to control output
     *        tau_out = tau_controller + GetCompensation()
     */
    float GetCompensation() const { return -disturbanceEst; }

    /**
     * @brief Get raw disturbance estimate
     */
    float GetDisturbanceEstimate() const { return disturbanceEst; }

    /**
     * @brief Enable/disable DOB
     */
    void SetEnabled(bool _enable) { config.enabled = _enable; }

    /**
     * @brief Update nominal inertia (e.g., after parameter identification)
     */
    void SetNominalInertia(float _inertia) { config.nominalInertia = _inertia; }

    /**
     * @brief Dynamically adjust cutoff frequency
     */
    void SetCutoffFrequency(float _freq);

private:
    Config_t config;
    float disturbanceEst;

    // 2nd-order Butterworth low-pass filter coefficients
    float b0, b1, b2;  // Numerator
    float a1, a2;       // Denominator (a0 = 1)

    // Filter states (two cascaded sections)
    float x1_prev, x2_prev;    // Input history
    float y1_prev, y2_prev;    // Output history

    // Second filter instance for inner path
    float x1b_prev, x2b_prev;
    float y1b_prev, y2b_prev;

    /**
     * @brief Compute 2nd-order Butterworth coefficients for given cutoff
     */
    void ComputeFilterCoeffs(float _cutoffHz, float _sampleHz);

    /**
     * @brief Apply low-pass filter to input
     */
    float ApplyFilter(float _input, float &_x1, float &_x2, float &_y1, float &_y2);
};


/**
 * @brief Multi-Joint Disturbance Observer for 6-DOF Robot
 *
 * Wraps per-joint DOBs and integrates with the RNEA dynamics model
 * for model-based disturbance estimation in joint space.
 *
 * d_hat = Q(s) * [tau_cmd - tau_model(q, dq, ddq)]
 *
 * Where tau_model comes from RNEA inverse dynamics.
 */
class MultiJointDOB
{
public:
    static constexpr int NUM_JOINTS = 6;

    /**
     * @brief Initialize all joint DOBs
     * @param _cutoffFreq   Cutoff frequency for all joints (Hz)
     * @param _sampleFreq   Sample frequency (Hz)
     * @param _inertias     Nominal inertias per joint (kg*m^2)
     */
    void Init(float _cutoffFreq, float _sampleFreq, const float _inertias[NUM_JOINTS]);

    /**
     * @brief Reset all DOBs
     */
    void Reset();

    /**
     * @brief Update all joint DOBs
     * @param _tauCmd       Commanded torques (6)
     * @param _velocity     Joint velocities (6, rad/s)
     * @param _acceleration Joint accelerations (6, rad/s^2)
     * @param _tauComp      Output: compensation torques to add (6)
     */
    void Update(const float _tauCmd[NUM_JOINTS],
                const float _velocity[NUM_JOINTS],
                const float _acceleration[NUM_JOINTS],
                float _tauComp[NUM_JOINTS]);

    /**
     * @brief Get disturbance estimates for all joints
     */
    void GetDisturbances(float _dist[NUM_JOINTS]) const;

    /**
     * @brief Enable/disable
     */
    void SetEnabled(bool _enable);

private:
    DisturbanceObserver jointDOB[NUM_JOINTS];
};

#endif // DISTURBANCE_OBSERVER_H
