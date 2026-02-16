#ifndef IMPEDANCE_CONTROLLER_H
#define IMPEDANCE_CONTROLLER_H

#include <cstdint>

/**
 * @brief Joint-Space & Cartesian-Space Impedance/Admittance Controller
 *
 * Implements compliant behavior for safe physical Human-Robot Interaction (pHRI).
 *
 * === Impedance Control (torque-output) ===
 * Target dynamics:
 *   M_d * (ddx - ddx_d) + B_d * (dx - dx_d) + K_d * (x - x_d) = F_ext
 *
 * Control law (joint space):
 *   tau = J^T * [M_d*ddx_d + B_d*dx_d + K_d*x_d - (M_d*ddx + B_d*dx + K_d*x)] + G(q)
 *
 * Simplification for stepper motors (position-controlled inner loop):
 *   x_cmd = x_d + M_d^{-1} * integral(integral(F_ext - B_d*dx - K_d*(x - x_d)))
 *
 * === Admittance Control (position-output) ===
 * Given external force F_ext (estimated from current/DOB):
 *   M_d * ddx_c + B_d * dx_c + K_d * x_c = F_ext
 *   x_cmd = x_d + x_c  (compliance displacement added to reference)
 *
 * Admittance is preferred for position-controlled actuators (stepper motors).
 *
 * === Variable Impedance ===
 * K_d and B_d can vary based on:
 *   - Distance to obstacles/humans
 *   - Task phase (stiff during precision, soft during contact)
 *   - Energy-based passivity constraint
 *
 * Reference:
 *   Hogan, "Impedance Control: An Approach to Manipulation", ASME JDSMC, 1985
 *   Ott, "Cartesian Impedance Control of Redundant Robots", Springer, 2008
 *
 * Target: IEEE T-RO / RA-L
 */

class ImpedanceController
{
public:
    static constexpr int DOF = 6;

    enum Mode_t
    {
        MODE_IMPEDANCE,    // Torque output (requires torque-mode motors)
        MODE_ADMITTANCE,   // Position output (works with position-controlled steppers)
        MODE_VARIABLE      // Variable impedance with automatic stiffness adjustment
    };

    struct CartesianImpedance_t
    {
        float Md[DOF];  // Desired inertia (kg or kg*m^2), diagonal
        float Bd[DOF];  // Desired damping (N*s/m or N*m*s/rad), diagonal
        float Kd[DOF];  // Desired stiffness (N/m or N*m/rad), diagonal
    };

    struct JointImpedance_t
    {
        float Md[DOF];  // Joint inertia (kg*m^2)
        float Bd[DOF];  // Joint damping (N*m*s/rad)
        float Kd[DOF];  // Joint stiffness (N*m/rad)
    };

    struct Config_t
    {
        Mode_t mode;
        float dt;                    // Sample period (s)
        float forceDeadzone;         // Force below this threshold is ignored (N)
        float maxComplianceDisp;     // Maximum compliance displacement (mm or deg)
        float dampingRatio;          // For automatic B computation: B = 2*zeta*sqrt(K*M)
        bool passivityEnforced;      // Enforce energy passivity constraint
    };

    struct State_t
    {
        float complianceDisp[DOF];   // Compliance displacement (Cartesian: mm, Joint: deg)
        float complianceVel[DOF];    // Compliance velocity
        float estimatedForce[DOF];   // Estimated external force/torque
        float virtualEnergy;         // Stored virtual energy (for passivity)
    };

    ImpedanceController();

    /**
     * @brief Initialize with default parameters
     */
    void Init(float _dt, Mode_t _mode = MODE_ADMITTANCE);

    /**
     * @brief Set Cartesian impedance parameters
     */
    void SetCartesianImpedance(const CartesianImpedance_t &_imp);

    /**
     * @brief Set joint-space impedance parameters
     */
    void SetJointImpedance(const JointImpedance_t &_imp);

    /**
     * @brief Set stiffness for all axes uniformly
     */
    void SetUniformStiffness(float _stiffness);

    /**
     * @brief Set damping ratio (automatic B computation)
     */
    void SetDampingRatio(float _zeta);

    /**
     * @brief Reset compliance state
     */
    void Reset();

    /**
     * @brief Admittance control tick: compute compliance displacement from external force
     *
     * M_d * ddx_c + B_d * dx_c + K_d * x_c = F_ext
     * x_cmd = x_ref + x_c
     *
     * @param _forceExt     Estimated external force/torque (6D)
     * @param _posRef       Reference position (6D)
     * @param _posCmd       Output: commanded position (6D) = ref + compliance
     */
    void AdmittanceTick(const float _forceExt[DOF],
                         const float _posRef[DOF],
                         float _posCmd[DOF]);

    /**
     * @brief Impedance control tick: compute torque for compliant behavior
     *
     * tau = K_d*(x_d - x) + B_d*(dx_d - dx) + M_d*(ddx_d - ddx) + G(q)
     *
     * @param _posRef       Reference position (6D)
     * @param _posCurrent   Current position (6D)
     * @param _velCurrent   Current velocity (6D)
     * @param _accCurrent   Current acceleration (6D)
     * @param _gravTorque   Gravity compensation torque (6D)
     * @param _tauCmd       Output: commanded torque (6D)
     */
    void ImpedanceTick(const float _posRef[DOF],
                        const float _posCurrent[DOF],
                        const float _velCurrent[DOF],
                        const float _accCurrent[DOF],
                        const float _gravTorque[DOF],
                        float _tauCmd[DOF]);

    /**
     * @brief Variable impedance tick: auto-adjust stiffness based on force
     *
     * When contact force is high -> reduce stiffness (become soft)
     * When contact force is low  -> increase stiffness (stay precise)
     *
     * K(F) = K_max * exp(-alpha * |F|)
     *
     * @param _forceExt  External force (6D)
     * @param _posRef    Reference position (6D)
     * @param _posCmd    Output position (6D)
     */
    void VariableImpedanceTick(const float _forceExt[DOF],
                                const float _posRef[DOF],
                                float _posCmd[DOF]);

    /**
     * @brief Get current state
     */
    const State_t &GetState() const { return state; }

    /**
     * @brief Get current effective stiffness (may change in variable mode)
     */
    void GetEffectiveStiffness(float _K[DOF]) const;

private:
    Config_t config;
    CartesianImpedance_t cartImp;
    JointImpedance_t jointImp;
    State_t state;

    // Variable impedance parameters
    float Kd_max[DOF];   // Maximum stiffness
    float Kd_min[DOF];   // Minimum stiffness
    float Kd_current[DOF]; // Current stiffness
    float alpha_var;      // Force sensitivity for variable stiffness

    /**
     * @brief Enforce passivity: dissipate energy if virtual energy becomes positive
     *        (tank-based passivity, Ferraguti et al. 2013)
     */
    void EnforcePassivity(const float _forceExt[DOF], const float _vel[DOF]);

    /**
     * @brief Compute critical damping: B = 2*zeta*sqrt(K*M)
     */
    void ComputeDamping();
};

#endif // IMPEDANCE_CONTROLLER_H
