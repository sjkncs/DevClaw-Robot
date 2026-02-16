#include "cartesian_planner.h"
#include <cmath>
#include <cstring>


// ========================== Quaternion Utilities ==========================

CartesianPlanner::Quaternion_t CartesianPlanner::RotMatToQuat(const float R[9])
{
    // Shepperd's method for robust rotation matrix to quaternion conversion
    Quaternion_t q;
    float trace = R[0] + R[4] + R[8];

    if (trace > 0)
    {
        float s = 0.5f / sqrtf(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (R[7] - R[5]) * s;
        q.y = (R[2] - R[6]) * s;
        q.z = (R[3] - R[1]) * s;
    }
    else if (R[0] > R[4] && R[0] > R[8])
    {
        float s = 2.0f * sqrtf(1.0f + R[0] - R[4] - R[8]);
        q.w = (R[7] - R[5]) / s;
        q.x = 0.25f * s;
        q.y = (R[1] + R[3]) / s;
        q.z = (R[2] + R[6]) / s;
    }
    else if (R[4] > R[8])
    {
        float s = 2.0f * sqrtf(1.0f + R[4] - R[0] - R[8]);
        q.w = (R[2] - R[6]) / s;
        q.x = (R[1] + R[3]) / s;
        q.y = 0.25f * s;
        q.z = (R[5] + R[7]) / s;
    }
    else
    {
        float s = 2.0f * sqrtf(1.0f + R[8] - R[0] - R[4]);
        q.w = (R[3] - R[1]) / s;
        q.x = (R[2] + R[6]) / s;
        q.y = (R[5] + R[7]) / s;
        q.z = 0.25f * s;
    }

    return QuatNormalize(q);
}


void CartesianPlanner::QuatToRotMat(const Quaternion_t &q, float R[9])
{
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

    R[0] = 1.0f - 2.0f * (yy + zz);
    R[1] = 2.0f * (xy - wz);
    R[2] = 2.0f * (xz + wy);
    R[3] = 2.0f * (xy + wz);
    R[4] = 1.0f - 2.0f * (xx + zz);
    R[5] = 2.0f * (yz - wx);
    R[6] = 2.0f * (xz - wy);
    R[7] = 2.0f * (yz + wx);
    R[8] = 1.0f - 2.0f * (xx + yy);
}


CartesianPlanner::Quaternion_t CartesianPlanner::QuatNormalize(const Quaternion_t &q)
{
    float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < 1e-10f) return {1, 0, 0, 0};
    float inv = 1.0f / norm;
    return {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}


float CartesianPlanner::QuatDot(const Quaternion_t &q1, const Quaternion_t &q2)
{
    return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}


CartesianPlanner::Quaternion_t CartesianPlanner::Slerp(const Quaternion_t &_q1,
                                                        const Quaternion_t &_q2,
                                                        float _t)
{
    Quaternion_t q2 = _q2;
    float dot = QuatDot(_q1, q2);

    // Ensure shortest path (flip q2 if dot product is negative)
    if (dot < 0.0f)
    {
        q2.w = -q2.w;
        q2.x = -q2.x;
        q2.y = -q2.y;
        q2.z = -q2.z;
        dot = -dot;
    }

    // If quaternions are very close, use linear interpolation to avoid numerical issues
    if (dot > 0.9995f)
    {
        Quaternion_t result = {
            _q1.w + _t * (q2.w - _q1.w),
            _q1.x + _t * (q2.x - _q1.x),
            _q1.y + _t * (q2.y - _q1.y),
            _q1.z + _t * (q2.z - _q1.z)
        };
        return QuatNormalize(result);
    }

    float theta = acosf(dot);
    float sinTheta = sinf(theta);
    float w1 = sinf((1.0f - _t) * theta) / sinTheta;
    float w2 = sinf(_t * theta) / sinTheta;

    Quaternion_t result = {
        w1 * _q1.w + w2 * q2.w,
        w1 * _q1.x + w2 * q2.x,
        w1 * _q1.y + w2 * q2.y,
        w1 * _q1.z + w2 * q2.z
    };
    return QuatNormalize(result);
}


// ========================== CartesianPlanner ==========================

void CartesianPlanner::AttachKinematicSolver(DOF6Kinematic *_solver)
{
    kinSolver = _solver;
}


bool CartesianPlanner::SelectBestIK(const DOF6Kinematic::Pose6D_t &_pose,
                                     const DOF6Kinematic::Joint6D_t &_prevJoints,
                                     DOF6Kinematic::Joint6D_t &_bestJoints,
                                     float _jointLimitsMin[6],
                                     float _jointLimitsMax[6])
{
    if (!kinSolver) return false;

    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D = _prevJoints;

    kinSolver->SolveIK(_pose, lastJoint6D, ikSolves);

    float minMaxAngle = 1e6f;
    int bestConfig = -1;

    for (int i = 0; i < 8; i++)
    {
        bool valid = true;

        // Check joint limits if provided
        if (_jointLimitsMin && _jointLimitsMax)
        {
            for (int j = 0; j < 6; j++)
            {
                if (ikSolves.config[i].a[j] < _jointLimitsMin[j] ||
                    ikSolves.config[i].a[j] > _jointLimitsMax[j])
                {
                    valid = false;
                    break;
                }
            }
        }

        if (!valid) continue;

        // Find max joint angle change
        float maxAngle = 0;
        for (int j = 0; j < 6; j++)
        {
            float diff = fabsf(ikSolves.config[i].a[j] - _prevJoints.a[j]);
            if (diff > maxAngle) maxAngle = diff;
        }

        if (maxAngle < minMaxAngle)
        {
            minMaxAngle = maxAngle;
            bestConfig = i;
        }
    }

    if (bestConfig < 0) return false;

    for (int j = 0; j < 6; j++)
        _bestJoints.a[j] = ikSolves.config[bestConfig].a[j];

    return true;
}


bool CartesianPlanner::PlanLinearPath(const DOF6Kinematic::Joint6D_t &_startJoints,
                                       const DOF6Kinematic::Pose6D_t &_targetPose,
                                       float _linearSpeed,
                                       float _linearAcc,
                                       float _linearJerk,
                                       int _numSamples)
{
    if (!kinSolver) return false;
    if (_numSamples < 2) _numSamples = 2;
    if (_numSamples > MAX_WAYPOINTS) _numSamples = MAX_WAYPOINTS;

    memset(&path, 0, sizeof(CartesianPath_t));

    // Step 1: Compute start pose via FK
    DOF6Kinematic::Pose6D_t startPose{};
    kinSolver->SolveFK(_startJoints, startPose);
    // FK returns position in meters, target is in mm -> convert start to mm
    startPose.X *= 1000.0f;
    startPose.Y *= 1000.0f;
    startPose.Z *= 1000.0f;

    // Step 2: Get start and end rotation matrices as quaternions
    Quaternion_t qStart = RotMatToQuat(startPose.R);

    // Get target rotation matrix
    float targetR[9];
    if (_targetPose.hasR)
    {
        memcpy(targetR, _targetPose.R, 9 * sizeof(float));
    }
    else
    {
        // Convert Euler angles (deg) to rotation matrix
        float euler_rad[3] = {
            _targetPose.A / 57.295777754771045f,
            _targetPose.B / 57.295777754771045f,
            _targetPose.C / 57.295777754771045f
        };
        float ca = cosf(euler_rad[2]), sa = sinf(euler_rad[2]);
        float cb = cosf(euler_rad[1]), sb = sinf(euler_rad[1]);
        float cc = cosf(euler_rad[0]), sc = sinf(euler_rad[0]);

        targetR[0] = ca * cb;
        targetR[1] = ca * sb * sc - sa * cc;
        targetR[2] = ca * sb * cc + sa * sc;
        targetR[3] = sa * cb;
        targetR[4] = sa * sb * sc + ca * cc;
        targetR[5] = sa * sb * cc - ca * sc;
        targetR[6] = -sb;
        targetR[7] = cb * sc;
        targetR[8] = cb * cc;
    }
    Quaternion_t qEnd = RotMatToQuat(targetR);

    // Step 3: Compute total Cartesian distance
    float dx = _targetPose.X - startPose.X;
    float dy = _targetPose.Y - startPose.Y;
    float dz = _targetPose.Z - startPose.Z;
    float totalLength = sqrtf(dx * dx + dy * dy + dz * dz);

    if (totalLength < 0.01f) // Less than 0.01mm
    {
        path.valid = true;
        path.numWaypoints = 1;
        path.totalLength = 0;
        path.totalTime = 0;
        for (int j = 0; j < 6; j++)
            path.waypoints[0].jointAngles[j] = _startJoints.a[j];
        path.waypoints[0].ikValid = true;
        return true;
    }

    path.totalLength = totalLength;

    // Step 4: Plan S-Curve profile along path parameter (0 to totalLength)
    SCurvePlanner::Constraints_t constraints = {
        _linearSpeed,   // mm/s
        _linearAcc,     // mm/s^2
        _linearJerk     // mm/s^3
    };
    bool scOk = pathPlanner.PlanTrajectory(0, totalLength, constraints);
    if (!scOk) return false;

    path.totalTime = pathPlanner.GetTotalTime();

    // Step 5: Sample waypoints along the path
    path.numWaypoints = _numSamples;
    DOF6Kinematic::Joint6D_t prevJoints = _startJoints;

    for (int i = 0; i < _numSamples; i++)
    {
        float s = (float)i / (float)(_numSamples - 1); // Parameter [0, 1]

        // Linear position interpolation (Cartesian straight line)
        float px = startPose.X + s * dx;
        float py = startPose.Y + s * dy;
        float pz = startPose.Z + s * dz;

        path.waypoints[i].position[0] = px;
        path.waypoints[i].position[1] = py;
        path.waypoints[i].position[2] = pz;

        // SLERP orientation interpolation
        Quaternion_t qInterp = Slerp(qStart, qEnd, s);
        QuatToRotMat(qInterp, path.waypoints[i].orientation);

        // Build Pose6D for IK (position in mm, orientation as rotation matrix)
        DOF6Kinematic::Pose6D_t pose;
        pose.X = px;
        pose.Y = py;
        pose.Z = pz;
        memcpy(pose.R, path.waypoints[i].orientation, 9 * sizeof(float));
        pose.hasR = true;

        // Solve IK
        DOF6Kinematic::Joint6D_t bestJoints{};
        bool ikOk = SelectBestIK(pose, prevJoints, bestJoints);

        path.waypoints[i].ikValid = ikOk;
        if (ikOk)
        {
            for (int j = 0; j < 6; j++)
                path.waypoints[i].jointAngles[j] = bestJoints.a[j];
            prevJoints = bestJoints;
        }
        else
        {
            // IK failed at this waypoint - path is invalid
            path.valid = false;
            return false;
        }
    }

    path.valid = true;
    return true;
}


bool CartesianPlanner::PlanCircularPath(const DOF6Kinematic::Joint6D_t &_startJoints,
                                         const DOF6Kinematic::Pose6D_t &_viaPose,
                                         const DOF6Kinematic::Pose6D_t &_targetPose,
                                         float _linearSpeed,
                                         float _linearAcc,
                                         float _linearJerk,
                                         int _numSamples)
{
    if (!kinSolver) return false;
    if (_numSamples < 3) _numSamples = 3;
    if (_numSamples > MAX_WAYPOINTS) _numSamples = MAX_WAYPOINTS;

    memset(&path, 0, sizeof(CartesianPath_t));

    // Step 1: FK to get start position
    DOF6Kinematic::Pose6D_t startPose{};
    kinSolver->SolveFK(_startJoints, startPose);
    startPose.X *= 1000.0f;
    startPose.Y *= 1000.0f;
    startPose.Z *= 1000.0f;

    // Three points: P1 (start), P2 (via), P3 (end)
    float P1[3] = {startPose.X, startPose.Y, startPose.Z};
    float P2[3] = {_viaPose.X, _viaPose.Y, _viaPose.Z};
    float P3[3] = {_targetPose.X, _targetPose.Y, _targetPose.Z};

    // Step 2: Find circle center and radius from 3 points
    // Vectors from P1 to P2 and P1 to P3
    float v12[3] = {P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]};
    float v13[3] = {P3[0] - P1[0], P3[1] - P1[1], P3[2] - P1[2]};

    // Normal to the plane containing P1, P2, P3
    float normal[3];
    normal[0] = v12[1] * v13[2] - v12[2] * v13[1];
    normal[1] = v12[2] * v13[0] - v12[0] * v13[2];
    normal[2] = v12[0] * v13[1] - v12[1] * v13[0];
    float nLen = sqrtf(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);

    if (nLen < 1e-6f)
    {
        // Points are collinear, fall back to linear
        return PlanLinearPath(_startJoints, _targetPose, _linearSpeed, _linearAcc, _linearJerk, _numSamples);
    }

    normal[0] /= nLen; normal[1] /= nLen; normal[2] /= nLen;

    // Find circle center using perpendicular bisectors
    float mid12[3] = {(P1[0] + P2[0]) * 0.5f, (P1[1] + P2[1]) * 0.5f, (P1[2] + P2[2]) * 0.5f};
    float mid13[3] = {(P1[0] + P3[0]) * 0.5f, (P1[1] + P3[1]) * 0.5f, (P1[2] + P3[2]) * 0.5f};

    // Perpendicular bisector directions (within the plane)
    float perp12[3], perp13[3];
    perp12[0] = v12[1] * normal[2] - v12[2] * normal[1];
    perp12[1] = v12[2] * normal[0] - v12[0] * normal[2];
    perp12[2] = v12[0] * normal[1] - v12[1] * normal[0];
    perp13[0] = v13[1] * normal[2] - v13[2] * normal[1];
    perp13[1] = v13[2] * normal[0] - v13[0] * normal[2];
    perp13[2] = v13[0] * normal[1] - v13[1] * normal[0];

    // Solve for intersection: mid12 + s*perp12 = mid13 + t*perp13
    // Use least-squares for 3D case
    float A[2][2], b[2];
    A[0][0] = perp12[0] * perp12[0] + perp12[1] * perp12[1] + perp12[2] * perp12[2];
    A[0][1] = -(perp12[0] * perp13[0] + perp12[1] * perp13[1] + perp12[2] * perp13[2]);
    A[1][0] = A[0][1];
    A[1][1] = perp13[0] * perp13[0] + perp13[1] * perp13[1] + perp13[2] * perp13[2];

    float dm[3] = {mid13[0] - mid12[0], mid13[1] - mid12[1], mid13[2] - mid12[2]};
    b[0] = dm[0] * perp12[0] + dm[1] * perp12[1] + dm[2] * perp12[2];
    b[1] = -(dm[0] * perp13[0] + dm[1] * perp13[1] + dm[2] * perp13[2]);

    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (fabsf(det) < 1e-10f)
        return PlanLinearPath(_startJoints, _targetPose, _linearSpeed, _linearAcc, _linearJerk, _numSamples);

    float s_param = (b[0] * A[1][1] - b[1] * A[0][1]) / det;

    float center[3] = {
        mid12[0] + s_param * perp12[0],
        mid12[1] + s_param * perp12[1],
        mid12[2] + s_param * perp12[2]
    };

    // Radius
    float r1[3] = {P1[0] - center[0], P1[1] - center[1], P1[2] - center[2]};
    float radius = sqrtf(r1[0] * r1[0] + r1[1] * r1[1] + r1[2] * r1[2]);

    // Compute angles for P1, P2, P3 relative to center
    // Local coordinate system: u = r1/|r1|, v = normal x u
    float u[3] = {r1[0] / radius, r1[1] / radius, r1[2] / radius};
    float v[3] = {
        normal[1] * u[2] - normal[2] * u[1],
        normal[2] * u[0] - normal[0] * u[2],
        normal[0] * u[1] - normal[1] * u[0]
    };

    auto computeAngle = [&](float P[3]) -> float {
        float rp[3] = {P[0] - center[0], P[1] - center[1], P[2] - center[2]};
        float cu = rp[0] * u[0] + rp[1] * u[1] + rp[2] * u[2];
        float cv = rp[0] * v[0] + rp[1] * v[1] + rp[2] * v[2];
        return atan2f(cv, cu);
    };

    float angle1 = 0; // Start is always 0
    float angle2 = computeAngle(P2);
    float angle3 = computeAngle(P3);

    // Ensure monotonic angle progression
    if (angle2 < 0) angle2 += 2.0f * (float)M_PI;
    if (angle3 < 0) angle3 += 2.0f * (float)M_PI;
    if (angle3 < angle2) angle3 += 2.0f * (float)M_PI;

    float totalAngle = angle3;
    float arcLength = radius * totalAngle;
    path.totalLength = arcLength;

    // Step 3: S-Curve profile along arc length
    SCurvePlanner::Constraints_t constraints = {_linearSpeed, _linearAcc, _linearJerk};
    if (!pathPlanner.PlanTrajectory(0, arcLength, constraints)) return false;
    path.totalTime = pathPlanner.GetTotalTime();

    // Step 4: Sample waypoints
    path.numWaypoints = _numSamples;
    DOF6Kinematic::Joint6D_t prevJoints = _startJoints;

    Quaternion_t qStart = RotMatToQuat(startPose.R);
    float targetR[9];
    if (_targetPose.hasR) memcpy(targetR, _targetPose.R, 9 * sizeof(float));
    else {
        float e[3] = {_targetPose.A / 57.295777754771045f, _targetPose.B / 57.295777754771045f, _targetPose.C / 57.295777754771045f};
        float ca2 = cosf(e[2]), sa2 = sinf(e[2]), cb2 = cosf(e[1]), sb2 = sinf(e[1]), cc2 = cosf(e[0]), sc2 = sinf(e[0]);
        targetR[0] = ca2*cb2; targetR[1] = ca2*sb2*sc2 - sa2*cc2; targetR[2] = ca2*sb2*cc2 + sa2*sc2;
        targetR[3] = sa2*cb2; targetR[4] = sa2*sb2*sc2 + ca2*cc2; targetR[5] = sa2*sb2*cc2 - ca2*sc2;
        targetR[6] = -sb2; targetR[7] = cb2*sc2; targetR[8] = cb2*cc2;
    }
    Quaternion_t qEnd = RotMatToQuat(targetR);

    for (int i = 0; i < _numSamples; i++)
    {
        float s_frac = (float)i / (float)(_numSamples - 1);
        float angle = s_frac * totalAngle;

        float px = center[0] + radius * (cosf(angle) * u[0] + sinf(angle) * v[0]);
        float py = center[1] + radius * (cosf(angle) * u[1] + sinf(angle) * v[1]);
        float pz = center[2] + radius * (cosf(angle) * u[2] + sinf(angle) * v[2]);

        path.waypoints[i].position[0] = px;
        path.waypoints[i].position[1] = py;
        path.waypoints[i].position[2] = pz;

        Quaternion_t qInterp = Slerp(qStart, qEnd, s_frac);
        QuatToRotMat(qInterp, path.waypoints[i].orientation);

        DOF6Kinematic::Pose6D_t pose;
        pose.X = px; pose.Y = py; pose.Z = pz;
        memcpy(pose.R, path.waypoints[i].orientation, 9 * sizeof(float));
        pose.hasR = true;

        DOF6Kinematic::Joint6D_t bestJoints{};
        bool ikOk = SelectBestIK(pose, prevJoints, bestJoints);
        path.waypoints[i].ikValid = ikOk;

        if (ikOk)
        {
            for (int j = 0; j < 6; j++)
                path.waypoints[i].jointAngles[j] = bestJoints.a[j];
            prevJoints = bestJoints;
        }
        else
        {
            path.valid = false;
            return false;
        }
    }

    path.valid = true;
    return true;
}


bool CartesianPlanner::Evaluate(float _t,
                                 DOF6Kinematic::Joint6D_t &_joints,
                                 SCurvePlanner::State_t &_cartState) const
{
    if (!path.valid || path.numWaypoints < 2) return false;

    // Get S-Curve state at time _t -> gives us distance along path
    pathPlanner.Evaluate(_t, _cartState);
    float dist = _cartState.position; // Distance traveled along path

    // Map distance to waypoint index (linear mapping)
    float s = dist / path.totalLength; // Normalized parameter [0, 1]
    if (s < 0) s = 0;
    if (s > 1.0f) s = 1.0f;

    float fIdx = s * (float)(path.numWaypoints - 1);
    int idx0 = (int)fIdx;
    int idx1 = idx0 + 1;
    if (idx0 < 0) idx0 = 0;
    if (idx1 >= path.numWaypoints) idx1 = path.numWaypoints - 1;
    if (idx0 >= path.numWaypoints - 1)
    {
        idx0 = path.numWaypoints - 1;
        idx1 = idx0;
    }

    float alpha = fIdx - (float)idx0;
    if (alpha < 0) alpha = 0;
    if (alpha > 1.0f) alpha = 1.0f;

    // Linear interpolation between adjacent waypoints' joint angles
    for (int j = 0; j < 6; j++)
    {
        _joints.a[j] = path.waypoints[idx0].jointAngles[j] * (1.0f - alpha) +
                        path.waypoints[idx1].jointAngles[j] * alpha;
    }

    return true;
}
