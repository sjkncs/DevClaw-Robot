#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#include <cstdint>

/**
 * @brief Trajectory Optimizer: Minimum-Jerk, Time-Optimal, and Constrained
 *
 * Generates smooth, optimal trajectories subject to kinematic and dynamic
 * constraints. Three modes:
 *
 * === 1. Minimum-Jerk Trajectory (Flash & Hogan, 1985) ===
 *   Minimizes integral of jerk squared: min ∫(d³q/dt³)² dt
 *   Solution: 5th-order polynomial (unique closed-form)
 *   q(s) = q0 + (qf-q0)*(10s³ - 15s⁴ + 6s⁵),  s = t/T
 *   Properties: zero velocity, acceleration, jerk at endpoints
 *   Biologically plausible (matches human reaching movements)
 *
 * === 2. Time-Optimal Trajectory (bang-bang acceleration) ===
 *   Minimizes total time T subject to:
 *     |dq| <= dq_max, |ddq| <= ddq_max
 *   Uses dynamic programming / iterative scaling on S-Curve profile
 *
 * === 3. Multi-Point Trajectory via Cubic/Quintic Splines ===
 *   Through-point interpolation with continuity constraints:
 *     - C2 (cubic spline): position + velocity continuous
 *     - C4 (quintic spline): position + velocity + acceleration continuous
 *   Natural, clamped, or periodic boundary conditions
 *
 * === 4. Energy-Optimal Trajectory ===
 *   Minimizes total energy: min ∫ tau^T * dq dt
 *   Approximated by minimizing jerk (low-jerk ≈ low-energy for robots)
 *
 * Reference:
 *   Flash & Hogan, "The Coordination of Arm Movements", J.Neuroscience, 1985
 *   Biagiotti & Melchiorri, "Trajectory Planning for Automatic Machines
 *     and Robots", Springer, 2008
 */
class TrajectoryOptimizer
{
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr int MAX_WAYPOINTS = 50;
    static constexpr int MAX_SPLINE_POINTS = 2000;

    // ==================== Minimum-Jerk ====================

    struct MinJerkConfig_t
    {
        float q0[NUM_JOINTS];      // Start position (deg)
        float qf[NUM_JOINTS];      // End position (deg)
        float v0[NUM_JOINTS];      // Start velocity (deg/s), usually 0
        float vf[NUM_JOINTS];      // End velocity (deg/s), usually 0
        float a0[NUM_JOINTS];      // Start acceleration, usually 0
        float af[NUM_JOINTS];      // End acceleration, usually 0
        float duration;             // Total time T (s)
    };

    struct TrajectoryPoint_t
    {
        float q[NUM_JOINTS];       // Position (deg)
        float dq[NUM_JOINTS];      // Velocity (deg/s)
        float ddq[NUM_JOINTS];     // Acceleration (deg/s^2)
        float dddq[NUM_JOINTS];   // Jerk (deg/s^3)
        float time;
    };

    // ==================== Spline ====================

    struct SplineWaypoint_t
    {
        float q[NUM_JOINTS];       // Joint position at waypoint (deg)
        float time;                 // Time at waypoint (s)
        bool hasVelocity;           // If true, velocity constraint is specified
        float dq[NUM_JOINTS];      // Constrained velocity (if hasVelocity)
    };

    enum SplineType_t
    {
        SPLINE_CUBIC,      // C2 continuous (position + velocity)
        SPLINE_QUINTIC     // C4 continuous (position + velocity + acceleration)
    };

    // ==================== Time-Optimal ====================

    struct Constraints_t
    {
        float velMax[NUM_JOINTS];   // Max velocity per joint (deg/s)
        float accMax[NUM_JOINTS];   // Max acceleration per joint (deg/s^2)
        float jerkMax[NUM_JOINTS];  // Max jerk per joint (deg/s^3)
        float torqueMax[NUM_JOINTS]; // Max torque per joint (N*m), 0=ignore
    };

    TrajectoryOptimizer();

    // ==================== Minimum-Jerk API ====================

    /**
     * @brief Plan minimum-jerk trajectory between two configurations
     * @return Planned duration
     */
    float PlanMinJerk(const MinJerkConfig_t &_config);

    /**
     * @brief Evaluate minimum-jerk trajectory at time t
     */
    TrajectoryPoint_t EvalMinJerk(float _t) const;

    // ==================== Time-Optimal API ====================

    /**
     * @brief Compute minimum time to traverse between two configurations
     *        subject to velocity and acceleration constraints
     */
    float ComputeMinTime(const float _q0[NUM_JOINTS],
                          const float _qf[NUM_JOINTS],
                          const Constraints_t &_constraints);

    /**
     * @brief Plan time-optimal trajectory
     * @return Minimum achievable time
     */
    float PlanTimeOptimal(const float _q0[NUM_JOINTS],
                           const float _qf[NUM_JOINTS],
                           const Constraints_t &_constraints);

    /**
     * @brief Evaluate time-optimal trajectory at time t
     */
    TrajectoryPoint_t EvalTimeOptimal(float _t) const;

    // ==================== Spline API ====================

    /**
     * @brief Add waypoint for spline trajectory
     */
    bool AddWaypoint(const SplineWaypoint_t &_wp);

    /**
     * @brief Plan spline trajectory through all waypoints
     */
    bool PlanSpline(SplineType_t _type = SPLINE_CUBIC);

    /**
     * @brief Evaluate spline at time t
     */
    TrajectoryPoint_t EvalSpline(float _t) const;

    /**
     * @brief Clear all waypoints
     */
    void ClearWaypoints() { waypointCount = 0; }

    /**
     * @brief Get total trajectory duration
     */
    float GetDuration() const { return totalDuration; }

    /**
     * @brief Get number of waypoints
     */
    int GetWaypointCount() const { return waypointCount; }

private:
    // Minimum-jerk state
    MinJerkConfig_t mjConfig;
    bool mjPlanned;

    // Time-optimal state
    float toQ0[NUM_JOINTS], toQf[NUM_JOINTS];
    float toTimes[NUM_JOINTS]; // Per-joint time
    float toTotalTime;
    Constraints_t toConstraints;
    bool toPlanned;

    // Spline state
    SplineWaypoint_t waypoints[MAX_WAYPOINTS];
    int waypointCount;
    SplineType_t splineType;
    bool splinePlanned;

    // Cubic spline coefficients: q(t) = a + b*s + c*s^2 + d*s^3 per segment per joint
    struct CubicCoeffs_t { float a, b, c, d; };
    CubicCoeffs_t cubicCoeffs[MAX_WAYPOINTS - 1][NUM_JOINTS];
    int numSegments;

    float totalDuration;

    /**
     * @brief Solve tridiagonal system for cubic spline (Thomas algorithm)
     */
    static void SolveTridiagonal(const float *lower, const float *diag,
                                   const float *upper, const float *rhs,
                                   float *solution, int N);

    /**
     * @brief Compute 5th-order polynomial coefficients for min-jerk
     */
    void ComputeMinJerkCoeffs(int _joint, float _T,
                                float &c3, float &c4, float &c5) const;
};

#endif // TRAJECTORY_OPTIMIZER_H
