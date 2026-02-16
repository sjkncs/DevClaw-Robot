#ifndef DMP_H
#define DMP_H

#include <cstdint>

/**
 * @brief Dynamic Movement Primitives (DMP) for Learning from Demonstration
 *
 * Encodes demonstrated trajectories as a dynamical system that can:
 *   1. Reproduce the original trajectory
 *   2. Generalize to new start/goal positions
 *   3. Scale in time (speed up/slow down)
 *   4. Blend multiple demonstrations
 *
 * DMP formulation (Ijspeert et al., 2013):
 *
 * Transformation system (per DOF):
 *   tau * dz = alpha_z * (beta_z * (g - y) - z) + f(x)
 *   tau * dy = z
 *
 * Canonical system (phase variable):
 *   tau * dx = -alpha_x * x
 *
 * Forcing function (learned from demonstration):
 *   f(x) = sum_i(psi_i(x) * w_i * x) / sum_i(psi_i(x)) * (g - y0)
 *
 * Basis functions (Gaussian):
 *   psi_i(x) = exp(-h_i * (x - c_i)^2)
 *
 * Properties:
 *   - Guaranteed convergence to goal (attractor dynamics)
 *   - Spatial scaling via (g - y0) factor
 *   - Temporal scaling via tau
 *   - No explicit trajectory storage needed (compact weight representation)
 *
 * Reference:
 *   Ijspeert et al., "Dynamical Movement Primitives: Learning Attractor Models
 *     for Motor Behaviors", Neural Computation, 2013
 *   Pastor et al., "Learning and Generalization of Motor Skills by Learning
 *     from Demonstration", ICRA, 2009
 *
 * Target: IEEE T-RO / CoRL / RSS
 */
class DMP
{
public:
    static constexpr int MAX_BASIS = 30;      // Max Gaussian basis functions
    static constexpr int MAX_DEMO_POINTS = 2000; // Max demonstration points

    struct Config_t
    {
        int numBasis;           // Number of Gaussian basis functions (default 25)
        float alphaZ;           // Transformation system gain (default 25)
        float betaZ;            // Transformation system gain (default alphaZ/4)
        float alphaX;           // Canonical system decay rate (default 1.0)
        float tau;              // Temporal scaling factor (default = demo duration)
        float dt;               // Sample period (s)
    };

    struct LearnedDMP_t
    {
        float weights[MAX_BASIS];   // Learned forcing function weights
        float centers[MAX_BASIS];   // Basis function centers
        float widths[MAX_BASIS];    // Basis function widths (h_i)
        int numBasis;
        float y0;                   // Demonstrated start position
        float goal;                 // Demonstrated goal position
        float tau;                  // Demonstrated duration
        bool learned;
    };

    struct State_t
    {
        float y;        // Position
        float z;        // Scaled velocity (z = tau * dy)
        float x;        // Phase variable [1 -> 0]
        float f;        // Current forcing function value
        float dy;       // Velocity
        float ddy;      // Acceleration
    };

    DMP();

    /**
     * @brief Initialize DMP with configuration
     */
    void Init(const Config_t &_config);

    /**
     * @brief Initialize with sensible defaults
     */
    void InitDefault(float _dt);

    /**
     * @brief Learn DMP weights from a demonstrated trajectory
     *
     * @param _trajectory   Demonstrated positions (1D, single DOF)
     * @param _numPoints    Number of trajectory points
     * @param _duration     Total demonstration duration (s)
     * @return true if learning succeeded
     */
    bool LearnFromDemo(const float *_trajectory, int _numPoints, float _duration);

    /**
     * @brief Learn DMP from demonstrated position + velocity + acceleration
     *        (more accurate, uses KF-smoothed data)
     */
    bool LearnFromDemoFull(const float *_pos, const float *_vel, const float *_acc,
                            int _numPoints, float _duration);

    /**
     * @brief Reset DMP for execution with new start and goal
     * @param _y0   New start position
     * @param _goal New goal position
     * @param _tau  Time scaling (0 = use learned duration)
     */
    void Reset(float _y0, float _goal, float _tau = 0);

    /**
     * @brief Execute one DMP step
     * @return Current state (position, velocity, acceleration)
     */
    State_t Step();

    /**
     * @brief Execute one step with online goal modulation
     * @param _newGoal  Potentially changing goal
     */
    State_t StepWithGoal(float _newGoal);

    /**
     * @brief Check if DMP execution is complete (phase x ≈ 0)
     */
    bool IsComplete() const { return state.x < 0.01f; }

    /**
     * @brief Get current state
     */
    const State_t &GetState() const { return state; }

    /**
     * @brief Get learned DMP for serialization/storage
     */
    const LearnedDMP_t &GetLearnedDMP() const { return learned; }

    /**
     * @brief Set learned DMP (load from storage)
     */
    void SetLearnedDMP(const LearnedDMP_t &_dmp);

private:
    Config_t config;
    LearnedDMP_t learned;
    State_t state;

    /**
     * @brief Compute forcing function f(x) from weights and basis functions
     */
    float ComputeForcing(float _x) const;

    /**
     * @brief Compute single Gaussian basis function
     */
    static float GaussianBasis(float _x, float _center, float _width);

    /**
     * @brief Setup basis function centers and widths evenly in phase space
     */
    void SetupBasisFunctions();
};


/**
 * @brief Multi-DOF DMP for 6-axis robot joint trajectories
 *
 * Wraps 6 independent DMPs with a shared canonical system
 * (ensuring temporal coupling across all joints).
 */
class MultiDOF_DMP
{
public:
    static constexpr int NUM_DOF = 6;

    struct MultiState_t
    {
        float y[NUM_DOF];
        float dy[NUM_DOF];
        float ddy[NUM_DOF];
        float x;           // Shared phase variable
        bool complete;
    };

    MultiDOF_DMP();

    /**
     * @brief Initialize all DOF DMPs
     */
    void Init(float _dt, int _numBasis = 25);

    /**
     * @brief Learn from multi-DOF demonstration
     * @param _trajectories  [numPoints][NUM_DOF] joint positions (deg)
     * @param _numPoints     Number of demonstration points
     * @param _duration      Total duration (s)
     */
    bool LearnFromDemo(const float _trajectories[][NUM_DOF],
                        int _numPoints, float _duration);

    /**
     * @brief Reset for execution with new start/goal
     */
    void Reset(const float _start[NUM_DOF], const float _goal[NUM_DOF],
               float _tau = 0);

    /**
     * @brief Execute one step for all DOFs
     */
    MultiState_t Step();

    /**
     * @brief Step with online goal modulation
     */
    MultiState_t StepWithGoal(const float _newGoal[NUM_DOF]);

    /**
     * @brief Check if execution is complete
     */
    bool IsComplete() const;

    /**
     * @brief Get current multi-DOF state
     */
    const MultiState_t &GetState() const { return multiState; }

    /**
     * @brief Record demonstration point (for online teaching)
     */
    void RecordPoint(const float _joints[NUM_DOF]);

    /**
     * @brief Finish recording and learn
     */
    bool FinishRecording(float _duration);

    /**
     * @brief Get number of recorded points
     */
    int GetRecordCount() const { return recordCount; }

private:
    DMP dmpDof[NUM_DOF];
    MultiState_t multiState;
    float sharedPhase;
    float sharedAlphaX;
    float sharedTau;
    float sharedDt;

    // Online recording buffer
    float recordBuffer[DMP::MAX_DEMO_POINTS][NUM_DOF];
    int recordCount;
};

#endif // DMP_H
