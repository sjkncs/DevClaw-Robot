#include "trajectory_optimizer.h"
#include <cmath>
#include <cstring>


TrajectoryOptimizer::TrajectoryOptimizer()
{
    memset(&mjConfig, 0, sizeof(MinJerkConfig_t));
    mjPlanned = false;
    toPlanned = false;
    splinePlanned = false;
    waypointCount = 0;
    numSegments = 0;
    totalDuration = 0;
    toTotalTime = 0;
}


// ========================== Minimum-Jerk ==========================

void TrajectoryOptimizer::ComputeMinJerkCoeffs(int _joint, float _T,
                                                  float &c3, float &c4, float &c5) const
{
    // 5th-order polynomial: q(t) = q0 + v0*t + a0/2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
    // Boundary conditions at t=T: q(T)=qf, dq(T)=vf, ddq(T)=af
    // Solving 3x3 system for [c3, c4, c5]:

    float T = _T;
    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;

    float dq = mjConfig.qf[_joint] - mjConfig.q0[_joint]
             - mjConfig.v0[_joint] * T - 0.5f * mjConfig.a0[_joint] * T2;
    float dv = mjConfig.vf[_joint] - mjConfig.v0[_joint] - mjConfig.a0[_joint] * T;
    float da = mjConfig.af[_joint] - mjConfig.a0[_joint];

    // [T3  T4  T5 ] [c3]   [dq]
    // [3T2 4T3 5T4] [c4] = [dv]
    // [6T  12T2 20T3] [c5]   [da]

    float det = T3 * (4*T3*20*T3 - 5*T4*12*T2)
              - T4 * (3*T2*20*T3 - 5*T4*6*T)
              + T5 * (3*T2*12*T2 - 4*T3*6*T);

    // Simplified closed-form for minimum-jerk with zero BCs:
    if (fabsf(mjConfig.v0[_joint]) < 1e-6f && fabsf(mjConfig.vf[_joint]) < 1e-6f &&
        fabsf(mjConfig.a0[_joint]) < 1e-6f && fabsf(mjConfig.af[_joint]) < 1e-6f)
    {
        // Standard min-jerk: q(s) = q0 + D*(10s^3 - 15s^4 + 6s^5), s=t/T
        float D = mjConfig.qf[_joint] - mjConfig.q0[_joint];
        c3 = 10.0f * D / T3;
        c4 = -15.0f * D / T4;
        c5 = 6.0f * D / T5;
    }
    else
    {
        // General case: solve via Cramer's rule
        float A[9] = {
            T3,     T4,      T5,
            3*T2,   4*T3,    5*T4,
            6*T,    12*T2,   20*T3
        };

        float detA = A[0]*(A[4]*A[8]-A[5]*A[7])
                    -A[1]*(A[3]*A[8]-A[5]*A[6])
                    +A[2]*(A[3]*A[7]-A[4]*A[6]);

        if (fabsf(detA) < 1e-10f)
        {
            c3 = c4 = c5 = 0;
            return;
        }

        float invDet = 1.0f / detA;

        c3 = invDet * (dq*(A[4]*A[8]-A[5]*A[7])
                      -A[1]*(dv*A[8]-A[5]*da)
                      +A[2]*(dv*A[7]-A[4]*da));
        c4 = invDet * (A[0]*(dv*A[8]-A[5]*da)
                      -dq*(A[3]*A[8]-A[5]*A[6])
                      +A[2]*(A[3]*da-dv*A[6]));
        c5 = invDet * (A[0]*(A[4]*da-dv*A[7])
                      -A[1]*(A[3]*da-dv*A[6])
                      +dq*(A[3]*A[7]-A[4]*A[6]));
    }
}


float TrajectoryOptimizer::PlanMinJerk(const MinJerkConfig_t &_config)
{
    memcpy(&mjConfig, &_config, sizeof(MinJerkConfig_t));
    totalDuration = _config.duration;
    mjPlanned = true;
    return totalDuration;
}


TrajectoryOptimizer::TrajectoryPoint_t TrajectoryOptimizer::EvalMinJerk(float _t) const
{
    TrajectoryPoint_t pt;
    memset(&pt, 0, sizeof(TrajectoryPoint_t));
    pt.time = _t;

    if (!mjPlanned) return pt;

    float T = mjConfig.duration;
    if (T < 1e-6f) T = 1e-6f;
    float t = _t;
    if (t < 0) t = 0;
    if (t > T) t = T;

    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;

    for (int j = 0; j < NUM_JOINTS; j++)
    {
        float c3, c4, c5;
        ComputeMinJerkCoeffs(j, T, c3, c4, c5);

        pt.q[j] = mjConfig.q0[j] + mjConfig.v0[j]*t + 0.5f*mjConfig.a0[j]*t2
                 + c3*t3 + c4*t4 + c5*t4*t;

        pt.dq[j] = mjConfig.v0[j] + mjConfig.a0[j]*t
                  + 3*c3*t2 + 4*c4*t3 + 5*c5*t4;

        pt.ddq[j] = mjConfig.a0[j] + 6*c3*t + 12*c4*t2 + 20*c5*t3;

        pt.dddq[j] = 6*c3 + 24*c4*t + 60*c5*t2;
    }

    return pt;
}


// ========================== Time-Optimal ==========================

float TrajectoryOptimizer::ComputeMinTime(const float _q0[NUM_JOINTS],
                                            const float _qf[NUM_JOINTS],
                                            const Constraints_t &_constraints)
{
    // Per-joint minimum time under trapezoidal velocity profile
    // T_j = max(|dq|/v_max, sqrt(4*|dq|/a_max))
    float maxTime = 0;

    for (int j = 0; j < NUM_JOINTS; j++)
    {
        float dq = fabsf(_qf[j] - _q0[j]);
        if (dq < 1e-6f) continue;

        float vMax = _constraints.velMax[j];
        float aMax = _constraints.accMax[j];

        // Check if triangular or trapezoidal profile
        float tTriang = 2.0f * sqrtf(dq / aMax); // Triangular (can't reach vMax)
        float tTrapez = dq / vMax + vMax / aMax;  // Trapezoidal

        float tJ;
        if (dq < vMax * vMax / aMax)
            tJ = tTriang; // Triangular
        else
            tJ = tTrapez; // Trapezoidal

        if (tJ > maxTime) maxTime = tJ;
    }

    return maxTime;
}


float TrajectoryOptimizer::PlanTimeOptimal(const float _q0[NUM_JOINTS],
                                             const float _qf[NUM_JOINTS],
                                             const Constraints_t &_constraints)
{
    memcpy(toQ0, _q0, NUM_JOINTS * sizeof(float));
    memcpy(toQf, _qf, NUM_JOINTS * sizeof(float));
    memcpy(&toConstraints, &_constraints, sizeof(Constraints_t));

    toTotalTime = ComputeMinTime(_q0, _qf, _constraints);
    totalDuration = toTotalTime;

    // Use minimum-jerk with the computed minimum time
    // This gives a smooth profile that respects the time constraint
    MinJerkConfig_t mjCfg;
    memcpy(mjCfg.q0, _q0, NUM_JOINTS * sizeof(float));
    memcpy(mjCfg.qf, _qf, NUM_JOINTS * sizeof(float));
    memset(mjCfg.v0, 0, NUM_JOINTS * sizeof(float));
    memset(mjCfg.vf, 0, NUM_JOINTS * sizeof(float));
    memset(mjCfg.a0, 0, NUM_JOINTS * sizeof(float));
    memset(mjCfg.af, 0, NUM_JOINTS * sizeof(float));
    mjCfg.duration = toTotalTime;
    PlanMinJerk(mjCfg);

    toPlanned = true;
    return toTotalTime;
}


TrajectoryOptimizer::TrajectoryPoint_t TrajectoryOptimizer::EvalTimeOptimal(float _t) const
{
    // Uses the min-jerk profile planned with the time-optimal duration
    return EvalMinJerk(_t);
}


// ========================== Cubic Spline ==========================

bool TrajectoryOptimizer::AddWaypoint(const SplineWaypoint_t &_wp)
{
    if (waypointCount >= MAX_WAYPOINTS) return false;
    waypoints[waypointCount] = _wp;
    waypointCount++;
    splinePlanned = false;
    return true;
}


void TrajectoryOptimizer::SolveTridiagonal(const float *lower, const float *diag,
                                             const float *upper, const float *rhs,
                                             float *solution, int N)
{
    // Thomas algorithm for tridiagonal system
    float c[MAX_WAYPOINTS], d[MAX_WAYPOINTS];

    // Forward sweep
    c[0] = upper[0] / diag[0];
    d[0] = rhs[0] / diag[0];

    for (int i = 1; i < N; i++)
    {
        float m = diag[i] - lower[i] * c[i - 1];
        if (fabsf(m) < 1e-10f) m = 1e-10f;
        c[i] = (i < N - 1) ? upper[i] / m : 0;
        d[i] = (rhs[i] - lower[i] * d[i - 1]) / m;
    }

    // Back substitution
    solution[N - 1] = d[N - 1];
    for (int i = N - 2; i >= 0; i--)
        solution[i] = d[i] - c[i] * solution[i + 1];
}


bool TrajectoryOptimizer::PlanSpline(SplineType_t _type)
{
    if (waypointCount < 2) return false;

    splineType = _type;
    numSegments = waypointCount - 1;
    totalDuration = waypoints[waypointCount - 1].time;

    // Natural cubic spline: solve for second derivatives (moments) M_i
    // M_0 = 0, M_n = 0 (natural boundary)
    // For each joint independently

    int N = waypointCount;

    for (int j = 0; j < NUM_JOINTS; j++)
    {
        float h[MAX_WAYPOINTS]; // Segment durations
        float delta[MAX_WAYPOINTS]; // Slopes

        for (int i = 0; i < numSegments; i++)
        {
            h[i] = waypoints[i + 1].time - waypoints[i].time;
            if (h[i] < 1e-6f) h[i] = 1e-6f;
            delta[i] = (waypoints[i + 1].q[j] - waypoints[i].q[j]) / h[i];
        }

        // Tridiagonal system for M (second derivatives at knots)
        // Natural BC: M[0] = 0, M[N-1] = 0
        int M = N - 2; // Internal knots
        if (M <= 0)
        {
            // Only 2 points: linear interpolation
            cubicCoeffs[0][j].a = waypoints[0].q[j];
            cubicCoeffs[0][j].b = delta[0];
            cubicCoeffs[0][j].c = 0;
            cubicCoeffs[0][j].d = 0;
            continue;
        }

        float lower[MAX_WAYPOINTS], diag_arr[MAX_WAYPOINTS];
        float upper_arr[MAX_WAYPOINTS], rhs_arr[MAX_WAYPOINTS];
        float moments[MAX_WAYPOINTS];

        for (int i = 0; i < M; i++)
        {
            int k = i + 1; // Actual knot index
            lower[i] = (i > 0) ? h[k - 1] : 0;
            diag_arr[i] = 2.0f * (h[k - 1] + h[k]);
            upper_arr[i] = (i < M - 1) ? h[k] : 0;
            rhs_arr[i] = 6.0f * (delta[k] - delta[k - 1]);
        }

        SolveTridiagonal(lower, diag_arr, upper_arr, rhs_arr, moments, M);

        // Reconstruct coefficients
        float fullMoments[MAX_WAYPOINTS];
        fullMoments[0] = 0; // Natural BC
        for (int i = 0; i < M; i++)
            fullMoments[i + 1] = moments[i];
        fullMoments[N - 1] = 0; // Natural BC

        for (int i = 0; i < numSegments; i++)
        {
            float hi = h[i];
            cubicCoeffs[i][j].a = waypoints[i].q[j];
            cubicCoeffs[i][j].b = delta[i] - hi * (2.0f * fullMoments[i] + fullMoments[i + 1]) / 6.0f;
            cubicCoeffs[i][j].c = fullMoments[i] / 2.0f;
            cubicCoeffs[i][j].d = (fullMoments[i + 1] - fullMoments[i]) / (6.0f * hi);
        }
    }

    splinePlanned = true;
    return true;
}


TrajectoryOptimizer::TrajectoryPoint_t TrajectoryOptimizer::EvalSpline(float _t) const
{
    TrajectoryPoint_t pt;
    memset(&pt, 0, sizeof(TrajectoryPoint_t));
    pt.time = _t;

    if (!splinePlanned || numSegments <= 0) return pt;

    // Clamp time
    float t = _t;
    if (t < waypoints[0].time) t = waypoints[0].time;
    if (t > waypoints[waypointCount - 1].time) t = waypoints[waypointCount - 1].time;

    // Find segment
    int seg = 0;
    for (int i = 0; i < numSegments; i++)
    {
        if (t >= waypoints[i].time && t <= waypoints[i + 1].time)
        {
            seg = i;
            break;
        }
        if (i == numSegments - 1) seg = i; // Last segment
    }

    float dt = t - waypoints[seg].time;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;

    for (int j = 0; j < NUM_JOINTS; j++)
    {
        const CubicCoeffs_t &c = cubicCoeffs[seg][j];
        pt.q[j] = c.a + c.b * dt + c.c * dt2 + c.d * dt3;
        pt.dq[j] = c.b + 2.0f * c.c * dt + 3.0f * c.d * dt2;
        pt.ddq[j] = 2.0f * c.c + 6.0f * c.d * dt;
        pt.dddq[j] = 6.0f * c.d;
    }

    return pt;
}
