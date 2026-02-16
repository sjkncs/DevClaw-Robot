#ifndef WORKSPACE_ANALYZER_H
#define WORKSPACE_ANALYZER_H

#include <cstdint>

/**
 * @brief Workspace Analysis & Singularity Monitoring
 *
 * Real-time monitoring of robot configuration quality:
 *   1. Manipulability index (Yoshikawa measure)
 *   2. Singularity proximity warning
 *   3. Joint limit margin analysis
 *   4. Condition number of Jacobian
 *   5. Reachability check for target poses
 *
 * === Manipulability ===
 *   w = sqrt(det(J * J^T))   (Yoshikawa, 1985)
 *   w → 0 near singularities, w_max at best configurations
 *
 * === Condition Number ===
 *   κ(J) = σ_max / σ_min    (ratio of largest to smallest singular value)
 *   κ → ∞ at singularities, κ = 1 at isotropic configurations
 *   Approximated without full SVD using power iteration
 *
 * === Joint Limit Proximity ===
 *   JLP_i = min(q_i - q_min_i, q_max_i - q_i) / (q_max_i - q_min_i)
 *   JLP ∈ [0, 0.5], low values = near limit
 *
 * Reference:
 *   Yoshikawa, "Manipulability of Robotic Mechanisms", pedestrians pedestrians pedestrians
 *   Siciliano et al., "Robotics: Modelling, Planning and Control", 2009
 */
class WorkspaceAnalyzer
{
public:
    static constexpr int NUM_JOINTS = 6;

    struct AnalysisResult_t
    {
        float manipulability;             // Yoshikawa measure w
        float conditionNumber;            // κ(J), ∞ if singular
        float jointLimitProximity[NUM_JOINTS]; // Per-joint margin [0-0.5]
        float minJointMargin;             // Worst-case margin
        int worstJoint;                   // Joint with lowest margin
        float singularityDistance;         // Distance to nearest singularity [0-1]
        bool nearSingularity;             // True if κ > threshold
        bool nearJointLimit;              // True if any joint margin < threshold
        bool isReachable;                 // True if current pose is valid
    };

    struct Config_t
    {
        float jointMin[NUM_JOINTS];       // Joint limits min (deg)
        float jointMax[NUM_JOINTS];       // Joint limits max (deg)
        float singularityThreshold;       // Condition number threshold (default 50)
        float jointMarginThreshold;       // Joint limit margin threshold (default 0.05)
        float manipulabilityMin;          // Minimum acceptable manipulability
    };

    WorkspaceAnalyzer();

    /**
     * @brief Initialize with DevClaw Robot joint limits
     */
    void InitDefault();

    /**
     * @brief Initialize with custom configuration
     */
    void Init(const Config_t &_config);

    /**
     * @brief Full analysis of current configuration
     *
     * @param _q          Joint positions (deg)
     * @param _jacobian   6x6 Jacobian matrix (row-major)
     * @return Analysis result
     */
    AnalysisResult_t Analyze(const float _q[NUM_JOINTS],
                               const float _jacobian[36]);

    /**
     * @brief Quick singularity check (lighter computation)
     */
    bool CheckSingularity(const float _jacobian[36]) const;

    /**
     * @brief Compute manipulability: w = sqrt(det(J*J^T))
     */
    float ComputeManipulability(const float _jacobian[36]) const;

    /**
     * @brief Compute condition number approximation
     */
    float ComputeConditionNumber(const float _jacobian[36]) const;

    /**
     * @brief Check if target joints are within limits
     */
    bool CheckJointLimits(const float _q[NUM_JOINTS]) const;

    /**
     * @brief Get gradient direction to move away from singularity
     *        (for singularity avoidance in IK)
     * @param _jacobian   6x6 Jacobian
     * @param _gradient   Output: 6-vector joint velocity to increase manipulability
     */
    void GetSingularityAvoidanceGradient(const float _jacobian[36],
                                           float _gradient[NUM_JOINTS]) const;

    /**
     * @brief Get latest analysis result
     */
    const AnalysisResult_t &GetResult() const { return result; }

private:
    Config_t config;
    AnalysisResult_t result;

    /**
     * @brief Compute determinant of 6x6 matrix (LU decomposition)
     */
    static float Det6x6(const float *A);

    /**
     * @brief Multiply J * J^T for 6x6 matrices
     */
    static void JJT(const float *J, float *out);

    /**
     * @brief Power iteration for largest eigenvalue of symmetric matrix
     */
    static float LargestEigenvalue(const float *A, int N, int maxIter = 20);

    /**
     * @brief Inverse power iteration for smallest eigenvalue
     */
    static float SmallestEigenvalue(const float *A, int N, int maxIter = 20);
};

#endif // WORKSPACE_ANALYZER_H
