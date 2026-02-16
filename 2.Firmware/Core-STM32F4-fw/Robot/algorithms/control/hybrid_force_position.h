#ifndef HYBRID_FORCE_POSITION_H
#define HYBRID_FORCE_POSITION_H

#include <cstdint>

/**
 * @brief Parallel Force/Position Hybrid Controller
 *
 * Implements Raibert & Craig's (1981) hybrid control framework for tasks
 * requiring simultaneous force control in some Cartesian directions and
 * position control in the complementary directions.
 *
 * === Selection Matrix ===
 *   S = diag(s1, s2, ..., s6)   where s_i ∈ {0, 1}
 *   s_i = 1: force control in direction i
 *   s_i = 0: position control in direction i
 *
 * === Control Law (Task Space) ===
 *   F_cmd = S * [Kf_p*(F_d - F_est) + Kf_i*integral(F_d - F_est)]
 *         + (I-S) * [Kx_p*(x_d - x) + Kx_d*(dx_d - dx)]
 *         + G(q)   (gravity compensation)
 *
 *   tau = J^T * F_cmd   (map to joint space)
 *
 * === Typical Applications ===
 *   - Surface polishing: force control in Z, position in XY
 *   - Peg-in-hole: force in Z + compliance in XY
 *   - Contour following: force normal to surface, position along contour
 *   - Assembly: force along insertion axis, position for alignment
 *
 * === Spiral Search ===
 *   For peg-in-hole insertion, spiral search pattern in XY while
 *   maintaining insertion force in Z until contact is detected.
 *
 * Reference:
 *   Raibert & Craig, "Hybrid Position/Force Control of Manipulators",
 *     ASME JDSMC, 1981
 *   Siciliano & Villani, "Robot Force Control", Springer, 1999
 */
class HybridForcePositionController
{
public:
    static constexpr int DOF = 6; // [Fx,Fy,Fz, Tx,Ty,Tz] or [x,y,z, rx,ry,rz]

    struct Config_t
    {
        float dt;
        float selectionMatrix[DOF];       // 1 = force control, 0 = position control

        // Force control gains
        float Kf_p[DOF];                  // Force proportional gain
        float Kf_i[DOF];                  // Force integral gain
        float forceIntegralLimit;         // Anti-windup clamp (N or N*m)

        // Position control gains
        float Kx_p[DOF];                  // Position proportional gain
        float Kx_d[DOF];                  // Position derivative gain

        // Safety
        float maxForce;                   // Maximum commanded force (N)
        float maxTorque;                  // Maximum commanded torque (N*m)
    };

    struct Reference_t
    {
        float forceRef[DOF];             // Desired force/torque in force-controlled directions
        float posRef[DOF];               // Desired position in position-controlled directions
        float velRef[DOF];               // Desired velocity (feedforward)
    };

    struct State_t
    {
        float forceError[DOF];           // F_d - F_measured
        float posError[DOF];             // x_d - x
        float forceIntegral[DOF];        // Integral of force error
        float cmdForce[DOF];             // Commanded Cartesian force/torque
        float cmdTorque[DOF];            // Commanded joint torques (after J^T mapping)
    };

    // Spiral search for peg-in-hole
    struct SpiralConfig_t
    {
        float radius;           // Current spiral radius (mm)
        float radiusRate;       // Spiral expansion rate (mm/rev)
        float angularSpeed;     // Angular speed (rad/s)
        float insertionForce;   // Z-axis insertion force (N)
        float contactThreshold; // Force threshold for hole detection (N)
        float maxRadius;        // Maximum search radius (mm)
    };

    HybridForcePositionController();

    /**
     * @brief Initialize with default parameters
     */
    void InitDefault(float _dt);

    /**
     * @brief Initialize with custom config
     */
    void Init(const Config_t &_config);

    /**
     * @brief Set selection matrix (which axes are force-controlled)
     * @param _sel  Array of 6 floats: 1.0 = force, 0.0 = position
     */
    void SetSelectionMatrix(const float _sel[DOF]);

    /**
     * @brief Set force reference for force-controlled axes
     */
    void SetForceReference(const float _forceRef[DOF]);

    /**
     * @brief Set position reference for position-controlled axes
     */
    void SetPositionReference(const float _posRef[DOF]);

    /**
     * @brief Reset integral terms
     */
    void Reset();

    /**
     * @brief Main control tick
     *
     * @param _posCurrent   Current Cartesian position [x,y,z,rx,ry,rz]
     * @param _velCurrent   Current Cartesian velocity
     * @param _forceMeas    Measured/estimated external force [Fx,Fy,Fz,Tx,Ty,Tz]
     * @param _jacobian     6x6 Jacobian matrix (row-major)
     * @param _gravTorque   Gravity compensation torque (joint-space, N*m)
     * @param _tauOut       Output: joint torque command (N*m)
     */
    void Compute(const float _posCurrent[DOF],
                  const float _velCurrent[DOF],
                  const float _forceMeas[DOF],
                  const float _jacobian[36],
                  const float _gravTorque[DOF],
                  float _tauOut[DOF]);

    /**
     * @brief Spiral search tick for peg-in-hole insertion
     *
     * Generates XY spiral pattern while maintaining Z force.
     * Returns true when hole is found (Z force drops suddenly).
     *
     * @param _spiralCfg    Spiral search parameters
     * @param _forceMeas    Measured external force
     * @param _xyOffset     Output: XY displacement to add to reference (mm)
     * @return true if hole detected
     */
    bool SpiralSearchTick(const SpiralConfig_t &_spiralCfg,
                           const float _forceMeas[DOF],
                           float _xyOffset[2]);

    /**
     * @brief Get current state
     */
    const State_t &GetState() const { return state; }

private:
    Config_t config;
    Reference_t ref;
    State_t state;

    // Spiral search state
    float spiralAngle;
    float spiralRadius;
    float prevForceZ;
};

#endif // HYBRID_FORCE_POSITION_H
