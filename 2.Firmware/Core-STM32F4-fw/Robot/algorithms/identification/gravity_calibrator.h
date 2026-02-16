#ifndef GRAVITY_CALIBRATOR_H
#define GRAVITY_CALIBRATOR_H

#include <cstdint>

/**
 * @brief Gravity Compensation Calibrator
 *
 * Calibrates payload and link mass parameters from static pose measurements.
 * The robot holds multiple poses while stationary, and motor currents are
 * recorded. A WLS regression fits the gravity model parameters.
 *
 * === Gravity Model ===
 *   tau_g(q) = Y_g(q) * pi_g
 *
 * where Y_g is the gravity regressor (depends only on joint angles) and
 * pi_g contains the gravity-relevant dynamic parameters (mass * COM positions).
 *
 * For static equilibrium: tau_motor = tau_g(q)
 * Therefore: Kt * I * gear_ratio = Y_g(q) * pi_g
 *
 * === Calibration Procedure ===
 *   1. Move robot to N diverse static poses (N >= 10)
 *   2. Wait for steady state (zero velocity)
 *   3. Record joint angles q_k and motor currents I_k
 *   4. Build regressor: Y = [Y_g(q_1); Y_g(q_2); ...; Y_g(q_N)]
 *   5. Build measurement: b = [tau_1; tau_2; ...; tau_N]
 *   6. Solve: pi_g = (Y^T W Y)^{-1} Y^T W b  (WLS)
 *
 * === Payload Identification ===
 *   Also identifies end-effector payload (mass + COM offset)
 *   by comparing with/without payload.
 *
 * Reference:
 *   Atkeson et al., "Estimation of Inertial Parameters of Manipulator Loads
 *     and Links", IJRR, 1986
 *   Swevers et al., "Optimal Robot Excitation and Identification", IEEE TRA, 1997
 */
class GravityCalibrator
{
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr int MAX_POSES = 50;
    // Gravity parameters: 6 links × 3 (m*cx, m*cy, m*cz) + 1 payload mass = 19
    // Simplified: 6 links × 2 (m*cx, m*cz in gravity plane) + payload = 13
    static constexpr int NUM_PARAMS = 13;

    struct Sample_t
    {
        float q[NUM_JOINTS];        // Joint angles (deg)
        float tauMeas[NUM_JOINTS];  // Measured torque (from current)
    };

    struct CalibResult_t
    {
        float params[NUM_PARAMS];        // Identified gravity parameters
        float residualNorm;              // ||Y*pi - b|| / N
        float conditionNumber;           // Condition number of regression
        int numPoses;                    // Number of poses used
        bool valid;                      // True if calibration succeeded
        float payloadMass;               // Identified payload mass (kg)
        float payloadCOM[3];             // Payload COM offset (m)
    };

    GravityCalibrator();

    /**
     * @brief Initialize calibrator
     * @param _motorKt     Torque constants per joint (N*m/A)
     * @param _gearRatio   Gear reduction ratios
     */
    void Init(const float _motorKt[NUM_JOINTS],
              const float _gearRatio[NUM_JOINTS]);

    /**
     * @brief Record a calibration sample at current static pose
     * @param _q          Joint angles (deg)
     * @param _current    Motor currents (A)
     * @return Sample index, -1 if buffer full
     */
    int RecordSample(const float _q[NUM_JOINTS],
                      const float _current[NUM_JOINTS]);

    /**
     * @brief Run WLS calibration on all recorded samples
     * @return Calibration result
     */
    CalibResult_t Calibrate();

    /**
     * @brief Get calibrated gravity torque at given configuration
     * @param _q      Joint angles (deg)
     * @param _tauG   Output: gravity torque (N*m)
     */
    void ComputeGravityTorque(const float _q[NUM_JOINTS],
                                float _tauG[NUM_JOINTS]) const;

    /**
     * @brief Clear all recorded samples
     */
    void ClearSamples() { sampleCount = 0; }

    /**
     * @brief Get sample count
     */
    int GetSampleCount() const { return sampleCount; }

    /**
     * @brief Get calibration result
     */
    const CalibResult_t &GetResult() const { return result; }

private:
    float motorKt[NUM_JOINTS];
    float gearRatio[NUM_JOINTS];

    Sample_t samples[MAX_POSES];
    int sampleCount;

    CalibResult_t result;

    /**
     * @brief Build gravity regressor row for one pose
     *        Y_g(q) maps parameters to joint torques
     * @param _q      Joint angles (rad)
     * @param _Y      Output: NUM_JOINTS × NUM_PARAMS regressor
     */
    void BuildGravityRegressor(const float _q[NUM_JOINTS],
                                 float _Y[NUM_JOINTS * NUM_PARAMS]) const;

    /**
     * @brief Solve WLS: pi = (Y^T W Y + lambda*I)^{-1} Y^T W b
     */
    bool SolveWLS(const float *Y, const float *b, const float *W,
                   int nRows, int nCols, float lambda, float *x) const;
};

#endif // GRAVITY_CALIBRATOR_H
