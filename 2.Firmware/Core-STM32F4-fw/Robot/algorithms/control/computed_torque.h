#ifndef COMPUTED_TORQUE_H
#define COMPUTED_TORQUE_H

#include <cstdint>

/**
 * @brief Computed Torque Controller (CTC) / Inverse Dynamics Control
 *
 * Full model-based feedforward + feedback linearization for high-performance
 * trajectory tracking. Decouples the nonlinear multi-joint dynamics into
 * independent linear double-integrator systems.
 *
 * === Control Law ===
 *   tau = M(q) * [ddq_d + Kv*(dq_d - dq) + Kp*(q_d - q)] + C(q,dq)*dq + g(q)
 *       = M(q) * u + h(q,dq)
 *
 * where:
 *   u = ddq_d + Kv*(dq_d - dq) + Kp*(q_d - q)  (outer loop PD + feedforward)
 *   h(q,dq) = C(q,dq)*dq + g(q)                  (nonlinear terms)
 *
 * After cancellation, each joint behaves as:
 *   e_ddot + Kv*e_dot + Kp*e = 0   (critically damped if Kv = 2*sqrt(Kp))
 *
 * === Augmented CTC with Integral Action ===
 *   u = ddq_d + Kv*(dq_d - dq) + Kp*(q_d - q) + Ki*integral(q_d - q)
 *
 * === Robust CTC (with DOB compensation) ===
 *   tau = M(q)*u + h(q,dq) + tau_DOB
 *
 * Reference:
 *   Spong, Hutchinson, Vidyasagar, "Robot Modeling and Control", 2006, Ch.8
 *   Craig, "Introduction to Robotics", 2005, Ch.10
 */
class ComputedTorqueController
{
public:
    static constexpr int NUM_JOINTS = 6;

    struct Gains_t
    {
        float Kp[NUM_JOINTS];     // Position gain per joint
        float Kv[NUM_JOINTS];     // Velocity gain per joint
        float Ki[NUM_JOINTS];     // Integral gain per joint (0 = no integral)
        float integralLimit;      // Anti-windup integral clamp (rad*s)
    };

    struct Reference_t
    {
        float q_d[NUM_JOINTS];    // Desired position (rad)
        float dq_d[NUM_JOINTS];   // Desired velocity (rad/s)
        float ddq_d[NUM_JOINTS];  // Desired acceleration (rad/s^2)
    };

    struct State_t
    {
        float posError[NUM_JOINTS];      // q_d - q
        float velError[NUM_JOINTS];      // dq_d - dq
        float integralError[NUM_JOINTS]; // integral(q_d - q)
        float uCommand[NUM_JOINTS];      // Outer loop command (before M multiplication)
        float tauCommand[NUM_JOINTS];    // Final torque command
    };

    ComputedTorqueController();

    /**
     * @brief Initialize with default critically-damped gains
     */
    void Init(float _dt, float _naturalFreq = 50.0f);

    /**
     * @brief Set custom gains
     */
    void SetGains(const Gains_t &_gains);

    /**
     * @brief Set gains for critically damped response at given natural frequency
     *        Kp = wn^2, Kv = 2*wn  (all joints same)
     */
    void SetNaturalFrequency(float _wn);

    /**
     * @brief Set per-joint natural frequency (different stiffness per joint)
     */
    void SetPerJointFrequency(const float _wn[NUM_JOINTS]);

    /**
     * @brief Reset integral terms
     */
    void Reset();

    /**
     * @brief Compute control torque
     *
     * @param _ref        Desired trajectory state (q_d, dq_d, ddq_d)
     * @param _q          Current joint positions (rad)
     * @param _dq         Current joint velocities (rad/s)
     * @param _massMatrix Mass matrix M(q), 6x6 row-major
     * @param _hTorque    Nonlinear terms h(q,dq) = C*dq + g (from RNEA)
     * @param _tauDOB     DOB compensation torque (optional, can be nullptr)
     * @param _tauOut     Output: computed torque command (N*m)
     */
    void Compute(const Reference_t &_ref,
                  const float _q[NUM_JOINTS],
                  const float _dq[NUM_JOINTS],
                  const float _massMatrix[36],
                  const float _hTorque[NUM_JOINTS],
                  const float *_tauDOB,
                  float _tauOut[NUM_JOINTS]);

    /**
     * @brief Simplified CTC: gravity compensation + PD control (no mass matrix)
     *        tau = Kp*(q_d - q) + Kv*(dq_d - dq) + g(q)
     *        Useful when mass matrix computation is too expensive
     */
    void ComputePDGravity(const Reference_t &_ref,
                           const float _q[NUM_JOINTS],
                           const float _dq[NUM_JOINTS],
                           const float _gravTorque[NUM_JOINTS],
                           float _tauOut[NUM_JOINTS]);

    /**
     * @brief Get current state/errors
     */
    const State_t &GetState() const { return state; }

private:
    Gains_t gains;
    State_t state;
    float dt;

    /**
     * @brief Multiply 6x6 matrix by 6-vector: y = A*x
     */
    static void MatVec6(const float *A, const float *x, float *y);
};

#endif // COMPUTED_TORQUE_H
