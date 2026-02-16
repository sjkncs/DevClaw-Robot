#ifndef CARTESIAN_PLANNER_H
#define CARTESIAN_PLANNER_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "s_curve_planner.h"

/**
 * @brief Cartesian Space Linear Interpolation Planner with SLERP Orientation
 *
 * Implements true Cartesian straight-line motion (MoveL) by:
 *   1. Sampling waypoints along the line in Cartesian space
 *   2. Using SLERP for orientation interpolation (via quaternions)
 *   3. Applying S-Curve velocity profile along the path parameter s in [0,1]
 *   4. Converting each waypoint to joint angles via IK
 *
 * This replaces the naive MoveL which simply did one IK + MoveJ.
 *
 * Reference:
 *   Siciliano et al., "Robotics: Modelling, Planning and Control", Ch.3 & Ch.4
 */
class CartesianPlanner
{
public:
    static constexpr int MAX_WAYPOINTS = 200;  // Max interpolation points
    static constexpr int NUM_JOINTS = 6;

    struct Quaternion_t
    {
        float w, x, y, z;
    };

    struct CartesianWaypoint_t
    {
        float position[3];     // X, Y, Z (mm)
        float orientation[9];  // Rotation matrix (row-major)
        float jointAngles[6];  // Corresponding joint angles (deg)
        bool ikValid;
    };

    struct CartesianPath_t
    {
        CartesianWaypoint_t waypoints[MAX_WAYPOINTS];
        int numWaypoints;
        float totalLength;     // Total Cartesian path length (mm)
        float totalTime;       // Total time from S-Curve profile (s)
        bool valid;
    };

    CartesianPlanner() = default;

    /**
     * @brief Set the kinematic solver reference (shared with DevClawRobot)
     */
    void AttachKinematicSolver(DOF6Kinematic *_solver);

    /**
     * @brief Plan a Cartesian linear path from current pose to target pose
     * @param _startJoints   Current joint angles (deg)
     * @param _targetPose    Target Cartesian pose (X,Y,Z in mm; A,B,C in deg)
     * @param _linearSpeed   Cartesian linear speed (mm/s)
     * @param _linearAcc     Cartesian linear acceleration (mm/s^2)
     * @param _linearJerk    Cartesian linear jerk (mm/s^3)
     * @param _numSamples    Number of interpolation samples (more = smoother)
     * @return true if path is valid (all IK solutions found)
     */
    bool PlanLinearPath(const DOF6Kinematic::Joint6D_t &_startJoints,
                        const DOF6Kinematic::Pose6D_t &_targetPose,
                        float _linearSpeed,
                        float _linearAcc,
                        float _linearJerk,
                        int _numSamples = 50);

    /**
     * @brief Plan a Cartesian circular arc path
     * @param _startJoints   Current joint angles
     * @param _viaPose       Via point on the arc
     * @param _targetPose    End point of the arc
     * @param _linearSpeed   Speed along the arc (mm/s)
     * @param _linearAcc     Acceleration (mm/s^2)
     * @param _linearJerk    Jerk (mm/s^3)
     * @param _numSamples    Number of interpolation samples
     * @return true if path is valid
     */
    bool PlanCircularPath(const DOF6Kinematic::Joint6D_t &_startJoints,
                          const DOF6Kinematic::Pose6D_t &_viaPose,
                          const DOF6Kinematic::Pose6D_t &_targetPose,
                          float _linearSpeed,
                          float _linearAcc,
                          float _linearJerk,
                          int _numSamples = 50);

    /**
     * @brief Get joint angles at a given time along the planned path
     * @param _t             Time since path start (s)
     * @param _joints        Output joint angles (deg)
     * @param _cartState     Output Cartesian state (position, velocity, acceleration along path)
     * @return true if valid
     */
    bool Evaluate(float _t,
                  DOF6Kinematic::Joint6D_t &_joints,
                  SCurvePlanner::State_t &_cartState) const;

    /**
     * @brief Get the planned path for inspection
     */
    const CartesianPath_t &GetPath() const { return path; }

    /**
     * @brief Get total path time
     */
    float GetTotalTime() const { return path.totalTime; }

    /**
     * @brief Check validity
     */
    bool IsValid() const { return path.valid; }


    // ==================== Quaternion Utilities ====================

    /**
     * @brief Convert rotation matrix to quaternion
     */
    static Quaternion_t RotMatToQuat(const float R[9]);

    /**
     * @brief Convert quaternion to rotation matrix
     */
    static void QuatToRotMat(const Quaternion_t &q, float R[9]);

    /**
     * @brief SLERP: Spherical Linear Interpolation between two quaternions
     * @param _q1  Start quaternion
     * @param _q2  End quaternion
     * @param _t   Interpolation parameter [0, 1]
     * @return Interpolated quaternion
     */
    static Quaternion_t Slerp(const Quaternion_t &_q1, const Quaternion_t &_q2, float _t);

    /**
     * @brief Normalize a quaternion
     */
    static Quaternion_t QuatNormalize(const Quaternion_t &q);

    /**
     * @brief Quaternion dot product
     */
    static float QuatDot(const Quaternion_t &q1, const Quaternion_t &q2);


private:
    DOF6Kinematic *kinSolver = nullptr;
    CartesianPath_t path = {};
    SCurvePlanner pathPlanner;  // S-Curve along the path parameter

    /**
     * @brief Select best IK solution closest to previous joint angles
     */
    bool SelectBestIK(const DOF6Kinematic::Pose6D_t &_pose,
                      const DOF6Kinematic::Joint6D_t &_prevJoints,
                      DOF6Kinematic::Joint6D_t &_bestJoints,
                      float _jointLimitsMin[6] = nullptr,
                      float _jointLimitsMax[6] = nullptr);
};


#endif // CARTESIAN_PLANNER_H
