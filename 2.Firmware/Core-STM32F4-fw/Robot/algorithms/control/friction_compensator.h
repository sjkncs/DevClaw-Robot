#ifndef FRICTION_COMPENSATOR_H
#define FRICTION_COMPENSATOR_H

#include <cstdint>

/**
 * @brief LuGre Dynamic Friction Model & Compensator
 *
 * Models friction as a micro-bristle deformation phenomenon, capturing:
 *   - Stiction (static friction)
 *   - Coulomb friction
 *   - Viscous friction
 *   - Stribeck effect (velocity-dependent stiction drop)
 *   - Pre-sliding displacement (hysteresis at velocity reversal)
 *   - Frictional lag (friction depends on motion history)
 *
 * === LuGre Model (Canudas de Wit et al., 1995) ===
 *
 *   dz/dt = v - sigma_0 * |v| / g(v) * z          (bristle dynamics)
 *   g(v) = Fc + (Fs - Fc) * exp(-(v/vs)^2)        (Stribeck function)
 *   F_friction = sigma_0 * z + sigma_1 * dz/dt + sigma_2 * v
 *
 * Parameters:
 *   sigma_0 : Bristle stiffness (N*m/rad)
 *   sigma_1 : Bristle damping (N*m*s/rad)
 *   sigma_2 : Viscous friction coefficient (N*m*s/rad)
 *   Fc      : Coulomb friction torque (N*m)
 *   Fs      : Static friction torque (N*m), Fs > Fc
 *   vs      : Stribeck velocity (rad/s)
 *
 * === Compensation Strategy ===
 *   tau_comp = sigma_0 * z_hat + sigma_1 * dz_hat/dt + sigma_2 * v
 *
 * where z_hat is the estimated bristle state from the observer.
 *
 * Reference:
 *   Canudas de Wit et al., "A New Model for Control of Systems with Friction",
 *     IEEE TAC, 1995
 *   Olsson et al., "Friction Models and Friction Compensation", EJC, 1998
 *   Bona & Indri, "Friction Compensation in Robotics: an Overview", CDC, 2005
 */
class FrictionCompensator
{
public:
    static constexpr int NUM_JOINTS = 6;

    struct LuGreParams_t
    {
        float sigma0;   // Bristle stiffness (N*m/rad)
        float sigma1;   // Bristle damping (N*m*s/rad)
        float sigma2;   // Viscous friction (N*m*s/rad)
        float Fc;       // Coulomb friction (N*m)
        float Fs;       // Static friction (N*m), Fs >= Fc
        float vs;       // Stribeck velocity (rad/s)
    };

    struct Config_t
    {
        float dt;
        LuGreParams_t params[NUM_JOINTS];
        bool adaptiveEnabled;         // Online parameter adaptation
        float adaptationRate;         // Learning rate for online adaptation
    };

    struct State_t
    {
        float z[NUM_JOINTS];          // Bristle deflection state
        float dz[NUM_JOINTS];         // Bristle deflection rate
        float tauFriction[NUM_JOINTS]; // Total friction torque
        float tauComp[NUM_JOINTS];    // Compensation torque (= -tauFriction)
        float stribeckFn[NUM_JOINTS]; // g(v) Stribeck function value
    };

    FrictionCompensator();

    /**
     * @brief Initialize with default parameters for DevClaw Robot
     */
    void InitDefault(float _dt);

    /**
     * @brief Initialize with custom configuration
     */
    void Init(const Config_t &_config);

    /**
     * @brief Set LuGre parameters for a specific joint
     */
    void SetJointParams(int _joint, const LuGreParams_t &_params);

    /**
     * @brief Set parameters from identification results
     * @param _fv  Viscous friction coefficients [6]
     * @param _fc  Coulomb friction coefficients [6]
     */
    void SetFromIdentification(const float _fv[NUM_JOINTS],
                                const float _fc[NUM_JOINTS]);

    /**
     * @brief Reset bristle states
     */
    void Reset();

    /**
     * @brief Compute friction compensation torque
     *
     * @param _dq       Joint velocities (rad/s)
     * @param _tauComp  Output: compensation torque to add to control signal
     */
    void Compensate(const float _dq[NUM_JOINTS], float _tauComp[NUM_JOINTS]);

    /**
     * @brief Full update with optional online adaptation
     *
     * @param _dq           Joint velocities (rad/s)
     * @param _tauResidual  Torque residual for adaptation (tau_cmd - tau_needed)
     * @param _tauComp      Output: compensation torque
     */
    void Update(const float _dq[NUM_JOINTS],
                 const float _tauResidual[NUM_JOINTS],
                 float _tauComp[NUM_JOINTS]);

    /**
     * @brief Get current state
     */
    const State_t &GetState() const { return state; }

    /**
     * @brief Get estimated friction torque (for force estimator subtraction)
     */
    void GetFrictionTorque(float _tauFriction[NUM_JOINTS]) const;

private:
    Config_t config;
    State_t state;

    /**
     * @brief Compute Stribeck function: g(v) = Fc + (Fs - Fc)*exp(-(v/vs)^2)
     */
    static float StribeckFunction(float _v, const LuGreParams_t &_p);

    /**
     * @brief Online parameter adaptation using gradient descent
     */
    void AdaptParameters(int _joint, float _velocity, float _residual);
};

#endif // FRICTION_COMPENSATOR_H
