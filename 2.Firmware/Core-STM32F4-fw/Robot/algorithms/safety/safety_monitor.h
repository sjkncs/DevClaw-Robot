#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <cstdint>

/**
 * @brief Centralized Safety Monitor for Robot Operation
 *
 * Monitors all safety-critical parameters and triggers appropriate responses
 * when violations are detected. Implements ISO 10218 / ISO/TS 15066 concepts
 * for collaborative robot safety.
 *
 * === Monitored Parameters ===
 *   1. Joint position limits (hard + soft limits with margin)
 *   2. Joint velocity limits (speed monitoring, SSM)
 *   3. Joint torque/current limits (overload protection)
 *   4. Motor temperature estimation (I²t thermal model)
 *   5. Cartesian speed limit (TCP speed for cobot safety)
 *   6. External force magnitude (collision/overload)
 *   7. Communication watchdog (CAN bus timeout)
 *   8. Control loop timing (jitter detection)
 *
 * === Safety Response Hierarchy ===
 *   Level 0: Warning (log only)
 *   Level 1: Speed reduction (reduce to safe speed)
 *   Level 2: Controlled stop (decelerate to zero, category 1)
 *   Level 3: Emergency stop (immediate power cut, category 0)
 *
 * === Motor Thermal Model (I²t) ===
 *   dT/dt = (R * I² - (T - T_ambient) / R_thermal) / C_thermal
 *   Simplified: T_est += dt * (K_heat * I² - K_cool * (T_est - T_ambient))
 *
 * Reference:
 *   ISO 10218-1:2011, "Robots and robotic devices - Safety requirements"
 *   ISO/TS 15066:2016, "Collaborative robots - Safety requirements"
 *   Haddadin et al., "Requirements for Safe Robots", pedestrians, pedestrians, pedestrians, pedestrians, pedestrians, pedestrians
 */
class SafetyMonitor
{
public:
    static constexpr int NUM_JOINTS = 6;

    enum SafetyLevel_t
    {
        SAFETY_OK = 0,          // Normal operation
        SAFETY_WARNING = 1,     // Log warning, no action
        SAFETY_SPEED_REDUCE = 2,// Reduce speed to safe level
        SAFETY_STOP = 3,        // Controlled stop (cat.1)
        SAFETY_ESTOP = 4        // Emergency stop (cat.0)
    };

    enum ViolationType_t
    {
        VIOLATION_NONE = 0,
        VIOLATION_POS_LIMIT,
        VIOLATION_VEL_LIMIT,
        VIOLATION_TORQUE_LIMIT,
        VIOLATION_THERMAL,
        VIOLATION_TCP_SPEED,
        VIOLATION_EXT_FORCE,
        VIOLATION_WATCHDOG,
        VIOLATION_TIMING
    };

    struct JointLimits_t
    {
        float posMin[NUM_JOINTS];       // Hard position limit min (deg)
        float posMax[NUM_JOINTS];       // Hard position limit max (deg)
        float posSoftMargin;            // Soft limit margin before hard limit (deg)
        float velMax[NUM_JOINTS];       // Max velocity per joint (deg/s)
        float torqueMax[NUM_JOINTS];    // Max torque per joint (N*m)
        float currentMax[NUM_JOINTS];   // Max motor current (A)
    };

    struct ThermalConfig_t
    {
        float kHeat[NUM_JOINTS];        // Heating coefficient (°C/(A²·s))
        float kCool[NUM_JOINTS];        // Cooling coefficient (1/s)
        float tempWarning;              // Warning temperature (°C)
        float tempShutdown;             // Shutdown temperature (°C)
        float ambientTemp;              // Ambient temperature (°C)
    };

    struct SafetyConfig_t
    {
        float dt;
        JointLimits_t jointLimits;
        ThermalConfig_t thermal;
        float tcpSpeedMax;              // Max TCP speed for cobot (mm/s), ISO/TS 15066
        float extForceMax;              // Max external force before stop (N)
        float watchdogTimeout;          // CAN watchdog timeout (s)
        float timingJitterMax;          // Max acceptable loop jitter (ms)
    };

    struct SafetyStatus_t
    {
        SafetyLevel_t level;
        ViolationType_t violation;
        int violationJoint;             // Which joint triggered (-1 = none)
        float violationValue;           // The offending value

        float estTemperature[NUM_JOINTS]; // Estimated motor temperature (°C)
        float tcpSpeed;                   // Current TCP speed (mm/s)
        float extForceMag;                // Current external force magnitude (N)
        float speedScale;                 // Speed reduction factor [0-1]

        uint32_t warningCount;
        uint32_t stopCount;
        uint32_t estopCount;
        bool estopped;                    // Latched E-stop (requires manual reset)
    };

    SafetyMonitor();

    /**
     * @brief Initialize with default DevClaw Robot limits
     */
    void InitDefault(float _dt);

    /**
     * @brief Initialize with custom configuration
     */
    void Init(const SafetyConfig_t &_config);

    /**
     * @brief Main safety check tick — call every control cycle
     *
     * @param _q          Joint positions (deg)
     * @param _dq         Joint velocities (deg/s)
     * @param _current    Motor currents (A)
     * @param _tcpSpeed   TCP linear speed (mm/s)
     * @param _extForce   External force magnitude (N)
     * @param _loopTimeMs Actual loop time this cycle (ms)
     * @return Current safety level
     */
    SafetyLevel_t Check(const float _q[NUM_JOINTS],
                          const float _dq[NUM_JOINTS],
                          const float _current[NUM_JOINTS],
                          float _tcpSpeed,
                          float _extForce,
                          float _loopTimeMs);

    /**
     * @brief Feed CAN watchdog (call when valid CAN message received)
     */
    void FeedWatchdog(int _jointIdx);

    /**
     * @brief Reset E-stop latch (requires explicit user action)
     */
    bool ResetEstop();

    /**
     * @brief Get current safety status
     */
    const SafetyStatus_t &GetStatus() const { return status; }

    /**
     * @brief Get safe speed scale factor [0-1]
     *        Multiply trajectory velocity by this to enforce speed limits
     */
    float GetSpeedScale() const { return status.speedScale; }

    /**
     * @brief Check if robot is safe to operate
     */
    bool IsSafe() const { return status.level <= SAFETY_WARNING; }

    /**
     * @brief Set custom joint limits (e.g. after calibration)
     */
    void SetJointLimits(const float _posMin[NUM_JOINTS],
                         const float _posMax[NUM_JOINTS]);

    /**
     * @brief Set TCP speed limit (for cobot mode)
     */
    void SetTCPSpeedLimit(float _maxSpeed);

private:
    SafetyConfig_t config;
    SafetyStatus_t status;

    float watchdogTimers[NUM_JOINTS]; // Per-joint watchdog timer

    /**
     * @brief Update motor thermal model
     */
    void UpdateThermalModel(const float _current[NUM_JOINTS]);

    /**
     * @brief Check position limits (hard + soft)
     */
    SafetyLevel_t CheckPositionLimits(const float _q[NUM_JOINTS]);

    /**
     * @brief Check velocity limits
     */
    SafetyLevel_t CheckVelocityLimits(const float _dq[NUM_JOINTS]);

    /**
     * @brief Check torque/current limits
     */
    SafetyLevel_t CheckCurrentLimits(const float _current[NUM_JOINTS]);

    /**
     * @brief Check thermal limits
     */
    SafetyLevel_t CheckThermalLimits();

    /**
     * @brief Compute adaptive speed scale based on proximity to limits
     */
    float ComputeSpeedScale(const float _q[NUM_JOINTS],
                              const float _dq[NUM_JOINTS]) const;
};

#endif // SAFETY_MONITOR_H
