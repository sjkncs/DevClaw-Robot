#ifndef S_CURVE_PLANNER_H
#define S_CURVE_PLANNER_H

#include <cstdint>
#include <cmath>

/**
 * @brief 7-Segment S-Curve (Jerk-Limited) Trajectory Planner
 *
 * Generates smooth position/velocity/acceleration profiles with bounded:
 *   - Maximum velocity (v_max)
 *   - Maximum acceleration (a_max)
 *   - Maximum jerk (j_max)
 *
 * The 7 segments are:
 *   T1: Increasing acceleration  (jerk = +j_max)
 *   T2: Constant acceleration    (jerk = 0)
 *   T3: Decreasing acceleration  (jerk = -j_max)
 *   T4: Constant velocity        (jerk = 0, acc = 0)
 *   T5: Increasing deceleration  (jerk = -j_max)
 *   T6: Constant deceleration    (jerk = 0)
 *   T7: Decreasing deceleration  (jerk = +j_max)
 *
 * Usage:
 *   1. Call PlanTrajectory() with start/end conditions and constraints
 *   2. Call Evaluate(t) to get position/velocity/acceleration at time t
 *
 * Reference:
 *   Biagiotti & Melchiorri, "Trajectory Planning for Automatic Machines and Robots"
 *   Springer, 2008, Chapter 3.
 */
class SCurvePlanner
{
public:
    struct Constraints_t
    {
        float v_max;    // Maximum velocity (absolute, positive)
        float a_max;    // Maximum acceleration (absolute, positive)
        float j_max;    // Maximum jerk (absolute, positive)
    };

    struct State_t
    {
        float position;
        float velocity;
        float acceleration;
    };

    struct Profile_t
    {
        float T[7];         // Duration of each of the 7 segments
        float totalTime;    // Total trajectory time
        float direction;    // +1 or -1
        float p_start;      // Start position
        float p_end;        // End position
        float v_start;      // Start velocity (currently 0)
        float v_end;        // End velocity (currently 0)
        float v_peak;       // Achieved peak velocity
        float a_peak;       // Achieved peak acceleration
        float j_used;       // Jerk used
        bool valid;         // Whether planning succeeded
    };

    SCurvePlanner() = default;

    /**
     * @brief Plan a point-to-point trajectory from p_start to p_end (rest to rest)
     * @param _pStart   Start position
     * @param _pEnd     End position
     * @param _constraints  Velocity/acceleration/jerk limits
     * @return true if planning succeeded
     */
    bool PlanTrajectory(float _pStart, float _pEnd, const Constraints_t &_constraints);

    /**
     * @brief Plan trajectory with initial and final velocities
     * @param _pStart   Start position
     * @param _pEnd     End position
     * @param _vStart   Start velocity
     * @param _vEnd     End velocity
     * @param _constraints  Limits
     * @return true if planning succeeded
     */
    bool PlanTrajectoryWithVelocity(float _pStart, float _pEnd,
                                     float _vStart, float _vEnd,
                                     const Constraints_t &_constraints);

    /**
     * @brief Evaluate trajectory state at time t
     * @param _t    Time since trajectory start (seconds)
     * @param _state Output: position, velocity, acceleration
     */
    void Evaluate(float _t, State_t &_state) const;

    /**
     * @brief Get total trajectory duration
     */
    float GetTotalTime() const { return profile.totalTime; }

    /**
     * @brief Check if trajectory is valid
     */
    bool IsValid() const { return profile.valid; }

    /**
     * @brief Get the profile for inspection
     */
    const Profile_t &GetProfile() const { return profile; }

private:
    Profile_t profile = {};
    Constraints_t constraints = {};

    /**
     * @brief Internal: compute 7-segment times for a given displacement
     */
    bool ComputeSegmentTimes(float displacement, float v_start, float v_end,
                              float v_max, float a_max, float j_max);

    /**
     * @brief Evaluate a single segment
     */
    static void EvaluateSegment(float dt, float p0, float v0, float a0, float j,
                                 float &p_out, float &v_out, float &a_out);
};


/**
 * @brief Multi-Axis Synchronized S-Curve Planner
 *
 * Synchronizes up to 6 axes so they all start and finish at the same time,
 * while respecting per-axis constraints. The slowest axis determines the
 * total time, and other axes are scaled to match.
 */
class MultiAxisSCurvePlanner
{
public:
    static constexpr int MAX_AXES = 6;

    struct AxisConstraints_t
    {
        float v_max;
        float a_max;
        float j_max;
    };

    /**
     * @brief Plan synchronized multi-axis trajectory
     * @param _numAxes      Number of axes (1-6)
     * @param _pStart       Start positions for each axis
     * @param _pEnd         End positions for each axis
     * @param _constraints  Per-axis constraints
     * @return true if all axes planned successfully
     */
    bool PlanSynchronized(int _numAxes,
                           const float _pStart[MAX_AXES],
                           const float _pEnd[MAX_AXES],
                           const AxisConstraints_t _constraints[MAX_AXES]);

    /**
     * @brief Evaluate all axes at time t
     * @param _t        Time since start
     * @param _states   Output states for each axis
     */
    void Evaluate(float _t, SCurvePlanner::State_t _states[MAX_AXES]) const;

    /**
     * @brief Get total synchronized time
     */
    float GetTotalTime() const { return totalTime; }

    /**
     * @brief Check if all axes are valid
     */
    bool IsValid() const { return valid; }

private:
    int numAxes = 0;
    float totalTime = 0;
    bool valid = false;
    SCurvePlanner axisPlanner[MAX_AXES];
};


#endif // S_CURVE_PLANNER_H
