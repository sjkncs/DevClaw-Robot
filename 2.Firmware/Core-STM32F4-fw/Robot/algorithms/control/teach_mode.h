#ifndef TEACH_MODE_H
#define TEACH_MODE_H

#include <cstdint>

/**
 * @brief Sensorless Lead-Through Teaching (Gravity Compensation Mode)
 *
 * Enables manual guidance of the robot arm by compensating gravity and friction
 * while allowing free movement. The operator can physically guide the end-effector
 * and record waypoints or continuous trajectories.
 *
 * === Operating Principle ===
 *   tau_cmd = g(q) + f_comp(dq) + B_virtual * dq
 *
 * where:
 *   g(q)         = gravity compensation from RNEA
 *   f_comp(dq)   = friction compensation from LuGre model
 *   B_virtual    = small virtual damping for stability (prevents drift)
 *
 * The external force from the operator directly accelerates the joints:
 *   M(q)*ddq = tau_ext   (after cancellation of g and friction)
 *
 * === Features ===
 *   - Gravity compensation (model-based, uses identified parameters)
 *   - Friction compensation (LuGre model, reduces stiction)
 *   - Virtual damping (configurable, prevents drift when released)
 *   - Workspace boundaries (soft limits with repulsive virtual walls)
 *   - Waypoint recording (save joint configurations)
 *   - Continuous trajectory recording (at configurable sample rate)
 *   - Kinesthetic teaching with DMP encoding
 *
 * === Safety ===
 *   - Velocity limiting (caps max joint speed in teach mode)
 *   - Workspace constraints (virtual walls near joint limits)
 *   - Auto-brake when released (virtual damping increases)
 *
 * Reference:
 *   Villani & De Schutter, "Force Control", Springer Handbook of Robotics, 2016
 *   Kronander & Billard, "Stability Considerations for Variable Impedance
 *     Control", IEEE TRO, 2016
 */
class TeachMode
{
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr int MAX_WAYPOINTS = 100;
    static constexpr int MAX_TRAJ_POINTS = 5000;

    enum TeachState_t
    {
        TEACH_IDLE,
        TEACH_ACTIVE,       // Gravity comp active, free to move
        TEACH_RECORDING,    // Recording continuous trajectory
        TEACH_PAUSED        // Paused (holding position with stiffness)
    };

    struct Config_t
    {
        float dt;
        float virtualDamping[NUM_JOINTS];    // B_virtual (N*m*s/rad)
        float velocityLimit[NUM_JOINTS];     // Max velocity in teach mode (deg/s)
        float wallStiffness;                 // Virtual wall stiffness at joint limits (N*m/rad)
        float wallMargin;                    // Distance from limit to start wall (deg)
        float brakeDamping;                  // Damping when released (higher = quicker stop)
        float releaseThreshold;              // Force below this = released (N*m)
        float recordRate;                    // Trajectory recording rate (Hz)
    };

    struct Waypoint_t
    {
        float joints[NUM_JOINTS];   // Joint angles (deg)
        float timestamp;            // Time since teach start (s)
    };

    TeachMode();

    /**
     * @brief Initialize with default parameters
     */
    void InitDefault(float _dt);

    /**
     * @brief Initialize with custom config
     */
    void Init(const Config_t &_config);

    /**
     * @brief Start teach mode (enable gravity compensation)
     */
    void Start();

    /**
     * @brief Stop teach mode (re-enable position control)
     */
    void Stop();

    /**
     * @brief Pause teach mode (hold current position with stiffness)
     */
    void Pause();

    /**
     * @brief Resume from pause
     */
    void Resume();

    /**
     * @brief Start continuous trajectory recording
     */
    void StartRecording();

    /**
     * @brief Stop recording
     * @return Number of points recorded
     */
    int StopRecording();

    /**
     * @brief Save current position as waypoint
     */
    bool SaveWaypoint(const float _joints[NUM_JOINTS]);

    /**
     * @brief Main teach mode control tick
     *
     * Computes gravity + friction compensation torque for each joint.
     *
     * @param _q              Current joint positions (deg)
     * @param _dq             Current joint velocities (deg/s)
     * @param _gravTorque     Gravity compensation torque from RNEA (N*m)
     * @param _frictionComp   Friction compensation torque from LuGre (N*m)
     * @param _tauCmd         Output: torque command to motors (N*m)
     */
    void Tick(const float _q[NUM_JOINTS],
              const float _dq[NUM_JOINTS],
              const float _gravTorque[NUM_JOINTS],
              const float _frictionComp[NUM_JOINTS],
              float _tauCmd[NUM_JOINTS]);

    /**
     * @brief Get teach state
     */
    TeachState_t GetState() const { return state; }

    /**
     * @brief Get recorded waypoints
     */
    int GetWaypointCount() const { return waypointCount; }
    const Waypoint_t &GetWaypoint(int _idx) const { return waypoints[_idx]; }

    /**
     * @brief Get recorded trajectory
     */
    int GetTrajectoryCount() const { return trajCount; }
    const float (*GetTrajectory() const)[NUM_JOINTS] { return trajBuffer; }

    /**
     * @brief Clear all recorded data
     */
    void ClearRecording();

private:
    Config_t config;
    TeachState_t state;
    float teachTimer;
    float recordTimer;

    Waypoint_t waypoints[MAX_WAYPOINTS];
    int waypointCount;

    float trajBuffer[MAX_TRAJ_POINTS][NUM_JOINTS];
    int trajCount;

    // Auto-brake detection
    float prevVelMag;
    float releaseTimer;
    bool isReleased;

    /**
     * @brief Compute virtual wall repulsive torque near joint limits
     */
    float ComputeWallTorque(float _q, float _qMin, float _qMax) const;
};

#endif // TEACH_MODE_H
