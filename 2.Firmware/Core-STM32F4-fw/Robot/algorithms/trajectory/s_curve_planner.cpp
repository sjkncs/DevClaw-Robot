#include "s_curve_planner.h"
#include <cstring>

// ========================== SCurvePlanner ==========================

bool SCurvePlanner::PlanTrajectory(float _pStart, float _pEnd,
                                    const Constraints_t &_constraints)
{
    return PlanTrajectoryWithVelocity(_pStart, _pEnd, 0.0f, 0.0f, _constraints);
}


bool SCurvePlanner::PlanTrajectoryWithVelocity(float _pStart, float _pEnd,
                                                float _vStart, float _vEnd,
                                                const Constraints_t &_constraints)
{
    constraints = _constraints;
    memset(&profile, 0, sizeof(Profile_t));

    profile.p_start = _pStart;
    profile.p_end = _pEnd;
    profile.v_start = _vStart;
    profile.v_end = _vEnd;

    float displacement = _pEnd - _pStart;

    if (fabsf(displacement) < 1e-8f)
    {
        // Already at target
        profile.valid = true;
        profile.totalTime = 0;
        profile.direction = 1.0f;
        return true;
    }

    profile.direction = (displacement > 0) ? 1.0f : -1.0f;
    float dist = fabsf(displacement);

    // For rest-to-rest case (most common), vStart=0, vEnd=0
    bool ok = ComputeSegmentTimes(dist, fabsf(_vStart), fabsf(_vEnd),
                                   constraints.v_max, constraints.a_max, constraints.j_max);

    profile.valid = ok;
    return ok;
}


bool SCurvePlanner::ComputeSegmentTimes(float displacement, float v_start, float v_end,
                                          float v_max, float a_max, float j_max)
{
    /*
     * 7-Segment S-Curve Planning (Biagiotti & Melchiorri, 2008)
     *
     * Phase 1 (acceleration): T1 + T2 + T3
     *   T1 = T3 = Tj (jerk phase)
     *   T2 = Ta - 2*Tj (constant acceleration phase)
     *
     * Phase 2 (cruise): T4
     *
     * Phase 3 (deceleration): T5 + T6 + T7
     *   T5 = T7 = Tj (jerk phase)
     *   T6 = Td - 2*Tj (constant deceleration phase)
     *
     * For rest-to-rest (v_start = v_end = 0):
     */

    float Tj, Ta, Tv, Td;

    // Case 1: Check if a_max is reached during acceleration
    // Tj1 = a_max / j_max
    float Tj1 = a_max / j_max;

    // Check if v_max is reached
    // v_max = v_start + (Ta - Tj) * a_max
    // For rest-to-rest: v_max = (Ta - Tj) * a_max  ->  Ta = v_max/a_max + Tj

    float Ta_full = v_max / a_max + Tj1;
    float Td_full = v_max / a_max + Tj1; // Symmetric for rest-to-rest

    // Distance covered during acceleration phase
    float d_acc = 0.5f * (v_start + v_max) * Ta_full;
    // Distance covered during deceleration phase
    float d_dec = 0.5f * (v_max + v_end) * Td_full;

    if (displacement >= d_acc + d_dec)
    {
        // Case 1: v_max is reached, there is a cruise phase
        Tj = Tj1;
        Ta = Ta_full;
        Td = Td_full;
        Tv = (displacement - d_acc - d_dec) / v_max;

        profile.v_peak = v_max;
        profile.a_peak = a_max;
        profile.j_used = j_max;
    }
    else
    {
        // Case 2: v_max is NOT reached, no cruise phase
        Tv = 0;

        // Need to find the actual peak velocity v_peak < v_max
        // For rest-to-rest symmetric case:
        // displacement = v_peak * (Ta)  where Ta = v_peak/a_max + Tj1
        // But we also need to check if a_max is reached

        // Sub-case 2a: a_max is reached
        // Ta = Td = v_peak/a_max + a_max/j_max
        // displacement = v_peak * (v_peak/a_max + a_max/j_max)
        // v_peak^2/a_max + v_peak * a_max/j_max - displacement = 0
        // Solve quadratic: v_peak = (-a_max/j_max + sqrt((a_max/j_max)^2 + 4*displacement/a_max)) / 2 * a_max... 
        // Actually let's be more careful.

        // For symmetric rest-to-rest with a_max reached:
        // Ta = Td, displacement = v_peak * Ta
        // v_peak = a_max * (Ta - Tj1)
        // displacement = a_max * (Ta - Tj1) * Ta
        // Let x = Ta: a_max * x^2 - a_max * Tj1 * x - displacement = 0
        // x = (Tj1 + sqrt(Tj1^2 + 4*displacement/a_max)) / 2

        float disc_a = Tj1 * Tj1 + 4.0f * displacement / a_max;
        if (disc_a < 0) disc_a = 0;
        float Ta_reduced = (Tj1 + sqrtf(disc_a)) / 2.0f;

        float v_peak_a = a_max * (Ta_reduced - Tj1);

        if (v_peak_a > 0 && Ta_reduced >= 2.0f * Tj1)
        {
            // a_max is reached, T2 > 0
            Tj = Tj1;
            Ta = Ta_reduced;
            Td = Ta_reduced; // Symmetric
            profile.v_peak = v_peak_a;
            profile.a_peak = a_max;
            profile.j_used = j_max;
        }
        else
        {
            // Sub-case 2b: a_max is NOT reached either
            // Ta = Td = 2*Tj, T2 = T6 = 0
            // a_actual = j_max * Tj
            // v_peak = j_max * Tj^2
            // displacement = 2 * v_peak * 2*Tj / 2 = 2 * j_max * Tj^3
            // Tj = cbrt(displacement / (2 * j_max))

            float Tj_reduced = cbrtf(displacement / (2.0f * j_max));

            Tj = Tj_reduced;
            Ta = 2.0f * Tj;
            Td = 2.0f * Tj;

            float a_actual = j_max * Tj;
            float v_peak_b = a_actual * Tj; // = j_max * Tj^2

            profile.v_peak = v_peak_b;
            profile.a_peak = a_actual;
            profile.j_used = j_max;
        }
    }

    // Ensure no negative times
    if (Ta < 0) Ta = 0;
    if (Tv < 0) Tv = 0;
    if (Td < 0) Td = 0;
    if (Tj < 0) Tj = 0;

    // Assign 7 segment durations
    profile.T[0] = Tj;                                         // T1: jerk = +j_max
    profile.T[1] = (Ta >= 2.0f * Tj) ? (Ta - 2.0f * Tj) : 0;  // T2: jerk = 0 (const acc)
    profile.T[2] = Tj;                                         // T3: jerk = -j_max
    profile.T[3] = Tv;                                         // T4: cruise
    profile.T[4] = Tj;                                         // T5: jerk = -j_max
    profile.T[5] = (Td >= 2.0f * Tj) ? (Td - 2.0f * Tj) : 0;  // T6: jerk = 0 (const dec)
    profile.T[6] = Tj;                                         // T7: jerk = +j_max

    profile.totalTime = 0;
    for (int i = 0; i < 7; i++)
        profile.totalTime += profile.T[i];

    return (profile.totalTime > 0);
}


void SCurvePlanner::EvaluateSegment(float dt, float p0, float v0, float a0, float j,
                                     float &p_out, float &v_out, float &a_out)
{
    // p(dt) = p0 + v0*dt + 0.5*a0*dt^2 + (1/6)*j*dt^3
    // v(dt) = v0 + a0*dt + 0.5*j*dt^2
    // a(dt) = a0 + j*dt
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;

    p_out = p0 + v0 * dt + 0.5f * a0 * dt2 + (1.0f / 6.0f) * j * dt3;
    v_out = v0 + a0 * dt + 0.5f * j * dt2;
    a_out = a0 + j * dt;
}


void SCurvePlanner::Evaluate(float _t, State_t &_state) const
{
    if (!profile.valid || profile.totalTime <= 0)
    {
        _state.position = profile.p_start;
        _state.velocity = 0;
        _state.acceleration = 0;
        return;
    }

    // Clamp time
    if (_t <= 0)
    {
        _state.position = profile.p_start;
        _state.velocity = profile.v_start;
        _state.acceleration = 0;
        return;
    }
    if (_t >= profile.totalTime)
    {
        _state.position = profile.p_end;
        _state.velocity = profile.v_end;
        _state.acceleration = 0;
        return;
    }

    float dir = profile.direction;
    float j_max = profile.j_used;

    // Jerk values for each segment (in positive direction)
    float jerk[7] = {
        j_max,      // T1: increasing acceleration
        0.0f,       // T2: constant acceleration
        -j_max,     // T3: decreasing acceleration
        0.0f,       // T4: cruise
        -j_max,     // T5: increasing deceleration
        0.0f,       // T6: constant deceleration
        j_max       // T7: decreasing deceleration
    };

    // Evaluate by walking through segments
    float p = 0;  // Position relative to start (in positive direction)
    float v = fabsf(profile.v_start);
    float a = 0;
    float t_remaining = _t;

    for (int seg = 0; seg < 7; seg++)
    {
        float T_seg = profile.T[seg];
        if (T_seg <= 0) continue;

        if (t_remaining <= T_seg)
        {
            // We are within this segment
            float p_new, v_new, a_new;
            EvaluateSegment(t_remaining, p, v, a, jerk[seg], p_new, v_new, a_new);
            _state.position = profile.p_start + dir * p_new;
            _state.velocity = dir * v_new;
            _state.acceleration = dir * a_new;
            return;
        }
        else
        {
            // Complete this segment and move to next
            float p_new, v_new, a_new;
            EvaluateSegment(T_seg, p, v, a, jerk[seg], p_new, v_new, a_new);
            p = p_new;
            v = v_new;
            a = a_new;
            t_remaining -= T_seg;
        }
    }

    // Should not reach here, but just in case
    _state.position = profile.p_end;
    _state.velocity = profile.v_end;
    _state.acceleration = 0;
}


// ========================== MultiAxisSCurvePlanner ==========================

bool MultiAxisSCurvePlanner::PlanSynchronized(int _numAxes,
                                               const float _pStart[MAX_AXES],
                                               const float _pEnd[MAX_AXES],
                                               const AxisConstraints_t _constraints[MAX_AXES])
{
    numAxes = _numAxes;
    valid = false;
    totalTime = 0;

    if (_numAxes <= 0 || _numAxes > MAX_AXES)
        return false;

    // Step 1: Plan each axis independently to find individual optimal times
    float axisTimes[MAX_AXES] = {0};
    float maxTime = 0;
    int slowestAxis = 0;

    for (int i = 0; i < numAxes; i++)
    {
        SCurvePlanner::Constraints_t c = {
            _constraints[i].v_max,
            _constraints[i].a_max,
            _constraints[i].j_max
        };

        bool ok = axisPlanner[i].PlanTrajectory(_pStart[i], _pEnd[i], c);
        if (!ok) return false;

        axisTimes[i] = axisPlanner[i].GetTotalTime();
        if (axisTimes[i] > maxTime)
        {
            maxTime = axisTimes[i];
            slowestAxis = i;
        }
    }

    totalTime = maxTime;

    // Step 2: Re-plan faster axes to match the slowest axis time
    // Strategy: reduce v_max for faster axes so they take the same total time
    for (int i = 0; i < numAxes; i++)
    {
        if (i == slowestAxis) continue;
        if (axisTimes[i] <= 0) continue; // Zero displacement

        float displacement = fabsf(_pEnd[i] - _pStart[i]);
        if (displacement < 1e-8f) continue;

        // Binary search for v_max that gives totalTime
        float v_low = displacement / totalTime * 0.5f; // Rough lower bound
        float v_high = _constraints[i].v_max;
        float target_time = totalTime;

        SCurvePlanner::Constraints_t c_adj = {
            _constraints[i].v_max,
            _constraints[i].a_max,
            _constraints[i].j_max
        };

        // Try reducing v_max iteratively (bisection, max 20 iterations)
        for (int iter = 0; iter < 20; iter++)
        {
            float v_mid = (v_low + v_high) * 0.5f;
            c_adj.v_max = v_mid;

            SCurvePlanner tempPlanner;
            tempPlanner.PlanTrajectory(_pStart[i], _pEnd[i], c_adj);
            float t_mid = tempPlanner.GetTotalTime();

            if (t_mid < target_time)
                v_high = v_mid; // Need to slow down more
            else
                v_low = v_mid;  // Can go faster

            if (fabsf(t_mid - target_time) < 0.001f)
                break;
        }

        // Final plan with adjusted v_max
        c_adj.v_max = (v_low + v_high) * 0.5f;
        axisPlanner[i].PlanTrajectory(_pStart[i], _pEnd[i], c_adj);
    }

    valid = true;
    return true;
}


void MultiAxisSCurvePlanner::Evaluate(float _t, SCurvePlanner::State_t _states[MAX_AXES]) const
{
    for (int i = 0; i < numAxes; i++)
    {
        axisPlanner[i].Evaluate(_t, _states[i]);
    }
}
