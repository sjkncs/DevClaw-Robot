#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <cstdint>

/**
 * @brief Kalman Filter for Joint State Estimation
 *
 * State vector x = [position, velocity, acceleration]^T
 * Observation  y = [position_measured]
 *
 * State space model (discrete, constant dt = 1/20kHz = 50us):
 *   x_k = A * x_{k-1} + B * u_{k-1} + w    (state transition)
 *   y_k = C * x_k + v                        (observation)
 *
 * where:
 *   A = [1  dt  0.5*dt^2]    B = [0]     C = [1 0 0]
 *       [0  1   dt      ]        [0]
 *       [0  0   1       ]        [1]
 *
 *   w ~ N(0, Q)  process noise
 *   v ~ N(0, R)  measurement noise
 *
 * The Kalman gain K optimally fuses prediction and measurement:
 *   K = P_pred * C^T * (C * P_pred * C^T + R)^{-1}
 *
 * Prediction step:
 *   x_pred = A * x_hat + B * u
 *   P_pred = A * P * A^T + Q
 *
 * Update step:
 *   innovation = y - C * x_pred
 *   K = P_pred * C^T / (C * P_pred * C^T + R)
 *   x_hat = x_pred + K * innovation
 *   P = (I - K * C) * P_pred
 *
 * Reference:
 *   - Kalman, "A New Approach to Linear Filtering and Prediction Problems", 1960
 *   - Screenshots: State Space Model -> Filtering & Smoothing -> Prediction Update
 *
 * Design choices for 20kHz real-time on STM32F1:
 *   - 3-state KF (not EKF, since model is linear)
 *   - Fixed-point friendly (but using float for clarity/portability)
 *   - Symmetric P matrix: only store upper triangle (6 elements)
 *   - ~2us execution time target at 72MHz
 */
class JointKalmanFilter
{
public:
    struct State_t
    {
        float position;      // Estimated position (encoder counts)
        float velocity;      // Estimated velocity (counts/s)
        float acceleration;  // Estimated acceleration (counts/s^2)
    };

    struct Config_t
    {
        float dt;            // Sample period (s), default 1/20000
        float q_pos;         // Process noise: position variance
        float q_vel;         // Process noise: velocity variance
        float q_acc;         // Process noise: acceleration variance
        float r_pos;         // Measurement noise: position variance
        float p_init;        // Initial covariance diagonal value
    };

    JointKalmanFilter();

    /**
     * @brief Initialize filter with configuration
     */
    void Init(const Config_t &_config);

    /**
     * @brief Initialize with default parameters tuned for Ctrl-Step encoder
     * @param _controlFrequency  Control loop frequency in Hz (default 20000)
     */
    void InitDefault(int32_t _controlFrequency = 20000);

    /**
     * @brief Reset filter state to a known position
     */
    void Reset(float _initialPosition);

    /**
     * @brief Run one Kalman filter cycle: predict + update
     * @param _measuredPosition  Encoder position measurement (counts)
     * @param _controlInput      Optional: known torque/current input for B*u term
     * @return Estimated state
     */
    State_t Update(float _measuredPosition, float _controlInput = 0.0f);

    /**
     * @brief Prediction step only (no measurement available)
     * @param _controlInput  Known input
     */
    void Predict(float _controlInput = 0.0f);

    /**
     * @brief Get current state estimate
     */
    const State_t &GetState() const { return state; }

    /**
     * @brief Get estimated position (for drop-in replacement)
     */
    float GetPosition() const { return state.position; }

    /**
     * @brief Get estimated velocity (for drop-in replacement)
     */
    float GetVelocity() const { return state.velocity; }

    /**
     * @brief Get estimated acceleration
     */
    float GetAcceleration() const { return state.acceleration; }

    /**
     * @brief Get innovation (measurement residual) - useful for fault detection
     */
    float GetInnovation() const { return innovation; }

    /**
     * @brief Adaptive noise tuning: adjust Q based on innovation sequence
     *        If innovations are too large, increase Q (model uncertainty)
     *        If innovations are too small, decrease Q (trust model more)
     */
    void AdaptiveNoiseUpdate();

private:
    Config_t config;
    State_t state;

    // Covariance matrix P (3x3 symmetric, stored as upper triangle)
    // P = [P00 P01 P02]
    //     [P01 P11 P12]
    //     [P02 P12 P22]
    float P[6]; // {P00, P01, P02, P11, P12, P22}

    // Kalman gain (3x1 since observation is scalar)
    float K[3];

    // Innovation (measurement residual)
    float innovation;

    // Innovation running statistics (for adaptive tuning)
    float innovationSqSum;
    float innovationCount;
    static constexpr float INNOVATION_WINDOW = 1000.0f; // samples

    // Precomputed constants
    float dt;
    float dt2_half; // 0.5 * dt^2
    float B_acc;    // Input gain for acceleration (current -> acceleration)

    // Helper: pack/unpack symmetric 3x3 matrix
    // idx: [0]=00, [1]=01, [2]=02, [3]=11, [4]=12, [5]=22
    static inline int symIdx(int i, int j)
    {
        if (i > j) { int t = i; i = j; j = t; } // ensure i <= j
        if (i == 0) return j;       // 00->0, 01->1, 02->2
        if (i == 1) return j + 2;   // 11->3, 12->4
        return 5;                    // 22->5
    }
};


/**
 * @brief Kalman Smoother (Rauch-Tung-Striebel) for offline trajectory refinement
 *
 * Given a sequence of KF forward estimates, the smoother runs backward
 * to produce optimal estimates using all data (past AND future).
 *
 * Useful for:
 *   - Offline dynamic parameter identification (need clean velocity/acceleration)
 *   - Post-processing recorded trajectories for paper figures
 *
 * x_smooth_k = x_filt_k + G_k * (x_smooth_{k+1} - x_pred_{k+1})
 * G_k = P_filt_k * A^T * P_pred_{k+1}^{-1}
 */
class KalmanSmoother
{
public:
    static constexpr int MAX_SAMPLES = 2000; // ~100ms at 20kHz

    struct SmoothedData_t
    {
        float position;
        float velocity;
        float acceleration;
    };

    /**
     * @brief Record a filter step for later smoothing
     * @param _filtState   Filtered state from KF
     * @param _predState   Predicted state (before update)
     * @param _P_filt      Filtered covariance (6 elements, upper triangle)
     * @param _P_pred      Predicted covariance (6 elements)
     */
    void RecordStep(const JointKalmanFilter::State_t &_filtState,
                    const JointKalmanFilter::State_t &_predState,
                    const float _P_filt[6],
                    const float _P_pred[6]);

    /**
     * @brief Run RTS smoother on recorded data
     * @param _dt  Sample period
     * @return Number of smoothed samples
     */
    int RunSmoother(float _dt);

    /**
     * @brief Get smoothed data at index
     */
    const SmoothedData_t &GetSmoothed(int _idx) const { return smoothed[_idx]; }

    /**
     * @brief Get number of recorded samples
     */
    int GetCount() const { return count; }

    /**
     * @brief Reset recorder
     */
    void Reset() { count = 0; }

private:
    struct Record_t
    {
        JointKalmanFilter::State_t filtState;
        JointKalmanFilter::State_t predState;
        float P_filt[6];
        float P_pred[6];
    };

    Record_t records[MAX_SAMPLES];
    SmoothedData_t smoothed[MAX_SAMPLES];
    int count = 0;
};

#endif // KALMAN_FILTER_H
