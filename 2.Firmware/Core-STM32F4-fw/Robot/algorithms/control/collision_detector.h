#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <cstdint>

/**
 * @brief Generalized Momentum-Based Collision Detection & Reaction
 *
 * Detects unexpected external contacts without dedicated force/torque sensors
 * by monitoring the generalized momentum residual:
 *
 *   p(t) = M(q) * dq                           (generalized momentum)
 *   r(t) = K_I * integral_0^t [tau - C^T*dq - g + r] ds   (residual)
 *
 * Properties (De Luca et al., 2006):
 *   - r(t) converges to the external torque: r -> tau_ext
 *   - No acceleration measurement needed (only position + velocity)
 *   - Decoupled per-joint detection thresholds
 *   - Low-pass filtering built into the observer gain K_I
 *
 * Safety reaction strategies (ISO/TS 15066 compliant):
 *   1. STOP:     Immediate motor disable (safest)
 *   2. RETRACT:  Move away from contact along collision direction
 *   3. FLOAT:    Switch to zero-torque/gravity compensation mode
 *   4. REFLEX:   Bounce back briefly then hold
 *   5. COMPLY:   Switch to admittance control mode
 *
 * Reference:
 *   De Luca et al., "Collision Detection and Safe Reaction with the DLR-III
 *     Lightweight Manipulator Arm", IROS, 2006
 *   Haddadin et al., "Robot Collisions: A Survey on Detection, Isolation,
 *     and Identification", IEEE TRO, 2017
 *
 * Target: IEEE RA-L / pedestrians / Safety Track
 */
class CollisionDetector
{
public:
    static constexpr int NUM_JOINTS = 6;

    enum ReactionStrategy_t
    {
        REACTION_STOP,      // Emergency stop (disable motors)
        REACTION_RETRACT,   // Move away from contact
        REACTION_FLOAT,     // Zero torque / gravity comp only
        REACTION_REFLEX,    // Brief retract then hold
        REACTION_COMPLY     // Switch to admittance control
    };

    enum CollisionState_t
    {
        STATE_NORMAL,       // No collision
        STATE_DETECTED,     // Collision just detected
        STATE_REACTING,     // Executing reaction strategy
        STATE_RECOVERED     // Reaction complete, awaiting reset
    };

    struct Config_t
    {
        float dt;                           // Sample period (s)
        float observerGain[NUM_JOINTS];     // K_I per joint (higher = faster detection, more noise)
        float detectionThreshold[NUM_JOINTS]; // Per-joint collision threshold (N*m)
        float energyThreshold;              // Total energy threshold for collision
        ReactionStrategy_t defaultReaction;
        float retractDistance;              // Retract distance (deg) for RETRACT/REFLEX
        float retractSpeed;                // Retract speed (deg/s)
        float reflexDuration;              // Reflex hold time (s)
    };

    struct CollisionInfo_t
    {
        CollisionState_t state;
        bool detected;                      // Collision detected this tick
        int collisionJoint;                 // Primary collision joint (0-5)
        float residual[NUM_JOINTS];         // Current momentum residual per joint
        float collisionTorque[NUM_JOINTS];  // Estimated collision torque
        float totalEnergy;                  // Collision energy estimate
        float collisionTime;               // Time since collision detected (s)
        float isolationDirection[NUM_JOINTS]; // Collision direction in joint space
    };

    CollisionDetector();

    /**
     * @brief Initialize with configuration
     */
    void Init(const Config_t &_config);

    /**
     * @brief Initialize with default parameters tuned for DevClaw Robot
     */
    void InitDefault(float _dt);

    /**
     * @brief Reset detector state (call after collision handling)
     */
    void Reset();

    /**
     * @brief Main detection tick - call every control cycle
     *
     * @param _tauCmd       Commanded joint torques (N*m)
     * @param _q            Joint positions (rad)
     * @param _dq           Joint velocities (rad/s)
     * @param _gravTorque   Gravity compensation torques (N*m)
     * @param _corTorque    Coriolis/centrifugal torques (N*m)
     * @param _massDiag     Diagonal of mass matrix (for momentum computation)
     * @return CollisionInfo with detection result
     */
    CollisionInfo_t Update(const float _tauCmd[NUM_JOINTS],
                           const float _q[NUM_JOINTS],
                           const float _dq[NUM_JOINTS],
                           const float _gravTorque[NUM_JOINTS],
                           const float _corTorque[NUM_JOINTS],
                           const float _massDiag[NUM_JOINTS]);

    /**
     * @brief Simplified detection using only position/velocity and motor current
     *        (no dynamics model needed - uses threshold on filtered jerk)
     *
     * @param _dq       Joint velocities
     * @param _ddq      Joint accelerations (from KF)
     * @param _current  Motor currents
     * @return CollisionInfo
     */
    CollisionInfo_t UpdateSimplified(const float _dq[NUM_JOINTS],
                                      const float _ddq[NUM_JOINTS],
                                      const float _current[NUM_JOINTS]);

    /**
     * @brief Get reaction commands based on collision state
     * @param _currentJoints  Current joint angles (deg)
     * @param _reactionJoints Output: reaction joint targets (deg)
     * @return true if reaction is active
     */
    bool GetReactionCommand(const float _currentJoints[NUM_JOINTS],
                            float _reactionJoints[NUM_JOINTS]);

    /**
     * @brief Get current collision info
     */
    const CollisionInfo_t &GetInfo() const { return info; }

    /**
     * @brief Set reaction strategy
     */
    void SetReactionStrategy(ReactionStrategy_t _strategy);

    /**
     * @brief Set per-joint detection thresholds
     */
    void SetThresholds(const float _thresholds[NUM_JOINTS]);

    /**
     * @brief Check if collision is active
     */
    bool IsCollisionActive() const { return info.state != STATE_NORMAL; }

private:
    Config_t config;
    CollisionInfo_t info;

    // Generalized momentum observer state
    float momentum[NUM_JOINTS];           // p = M*dq
    float residualIntegral[NUM_JOINTS];   // Integral accumulator for r(t)
    float prevDq[NUM_JOINTS];             // Previous velocity for momentum change

    // Reaction state
    float reactionTimer;
    float reactionStartJoints[NUM_JOINTS];
    float reactionTargetJoints[NUM_JOINTS];

    // Filtering for simplified detector
    float filteredJerk[NUM_JOINTS];
    float prevAcc[NUM_JOINTS];
    static constexpr float JERK_FILTER_ALPHA = 0.1f;
};

#endif // COLLISION_DETECTOR_H
