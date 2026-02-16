#ifndef KINEMATIC_CALIBRATOR_H
#define KINEMATIC_CALIBRATOR_H

#include <cstdint>

/**
 * @brief Kinematic Calibration: DH Parameter Error Compensation
 *
 * Identifies and compensates manufacturing/assembly errors in DH parameters
 * by measuring end-effector poses at known configurations and solving for
 * the parameter deviations.
 *
 * === Error Model ===
 *   T_actual = T_nominal * dT
 *   dT = f(delta_a, delta_d, delta_alpha, delta_theta)
 *
 *   Linearized: dp = J_calib * delta_phi
 *   where:
 *     dp     = measured pose error (6×1: dx,dy,dz,drx,dry,drz)
 *     J_calib = calibration Jacobian (∂p/∂phi)
 *     delta_phi = DH parameter deviations (4 per joint × 6 = 24 max)
 *
 * === Identifiable Parameters ===
 *   Not all 24 DH deviations are identifiable. Typically ~18 are observable
 *   for a 6-DOF arm. We use QR decomposition to find the identifiable subset.
 *
 * === Calibration Procedure ===
 *   1. Move robot to N diverse poses (N >= 20, well-distributed)
 *   2. Measure actual TCP position (e.g., laser tracker or touch probe)
 *   3. Compare with FK prediction → pose error dp_k
 *   4. Build calibration Jacobian for each pose
 *   5. Solve: delta_phi = (J^T J + λI)^{-1} J^T dp  (regularized LS)
 *   6. Update DH parameters: phi_new = phi_nominal + delta_phi
 *
 * === Simplified Mode (Joint-Only) ===
 *   For low-cost robots without external measurement:
 *   - Use known reference points (fixtures, alignment pins)
 *   - Calibrate joint offsets only (6 parameters)
 *   - Touch N reference points and solve for offsets
 *
 * Reference:
 *   Hollerbach et al., "A Survey of Kinematic Calibration", pedestrians
 *   Elatta et al., "An Overview of Robot Calibration", IJCSE, 2004
 */
class KinematicCalibrator
{
public:
    static constexpr int NUM_JOINTS = 6;
    static constexpr int MAX_SAMPLES = 50;
    // DH params per joint: a, d, alpha, theta_offset → 4*6 = 24
    // But we calibrate identifiable subset, typically joint offsets + key lengths
    static constexpr int NUM_FULL_PARAMS = 24;
    static constexpr int NUM_OFFSET_PARAMS = 6; // Joint offsets only

    struct DHParams_t
    {
        float a[NUM_JOINTS];       // Link length (m)
        float d[NUM_JOINTS];       // Link offset (m)
        float alpha[NUM_JOINTS];   // Link twist (rad)
        float theta0[NUM_JOINTS];  // Joint offset (rad)
    };

    struct PoseSample_t
    {
        float q[NUM_JOINTS];       // Joint angles (deg)
        float measPos[3];          // Measured TCP position [x,y,z] (mm)
        float measRot[3];          // Measured TCP orientation [rx,ry,rz] (deg), optional
        bool hasOrientation;       // True if orientation is measured
    };

    struct CalibResult_t
    {
        float jointOffsets[NUM_JOINTS];  // Calibrated joint offsets (deg)
        float dhCorrections[NUM_FULL_PARAMS]; // Full DH corrections
        float residualRMS;               // RMS position error after calibration (mm)
        float residualMax;               // Max position error after calibration (mm)
        float initialRMS;                // RMS error before calibration (mm)
        int numSamples;
        bool valid;
        bool fullCalib;                  // True if full DH, false if offsets only
    };

    KinematicCalibrator();

    /**
     * @brief Initialize with nominal DH parameters
     */
    void Init(const DHParams_t &_nominalDH);

    /**
     * @brief Record a calibration sample
     * @param _q          Joint angles (deg)
     * @param _measPos    Measured TCP position [x,y,z] (mm)
     * @param _measRot    Measured TCP orientation (deg), can be nullptr
     * @return Sample index, -1 if full
     */
    int RecordSample(const float _q[NUM_JOINTS],
                      const float _measPos[3],
                      const float _measRot[3] = nullptr);

    /**
     * @brief Run joint offset calibration (simplified, 6 parameters)
     */
    CalibResult_t CalibrateOffsets();

    /**
     * @brief Run full DH parameter calibration (24 parameters, needs more data)
     */
    CalibResult_t CalibrateFull();

    /**
     * @brief Apply calibration corrections to a joint reading
     * @param _qRaw      Raw joint angles (deg)
     * @param _qCorrected Output: corrected joint angles (deg)
     */
    void ApplyCorrection(const float _qRaw[NUM_JOINTS],
                           float _qCorrected[NUM_JOINTS]) const;

    /**
     * @brief Get corrected DH parameters
     */
    void GetCorrectedDH(DHParams_t &_correctedDH) const;

    /**
     * @brief Clear all samples
     */
    void ClearSamples() { sampleCount = 0; }

    int GetSampleCount() const { return sampleCount; }
    const CalibResult_t &GetResult() const { return result; }

private:
    DHParams_t nominalDH;
    DHParams_t correctedDH;
    PoseSample_t samples[MAX_SAMPLES];
    int sampleCount;
    CalibResult_t result;

    /**
     * @brief Compute FK with given DH parameters + offsets
     */
    void ComputeFK(const float _q[NUM_JOINTS],
                     const DHParams_t &_dh,
                     const float _offsets[NUM_JOINTS],
                     float _pos[3]) const;

    /**
     * @brief Build calibration Jacobian for joint offsets
     *        J_ij = ∂pos_i / ∂offset_j (numerical differentiation)
     */
    void BuildOffsetJacobian(const float _q[NUM_JOINTS],
                               const DHParams_t &_dh,
                               float _J[3 * NUM_JOINTS]) const;

    /**
     * @brief Solve regularized least squares
     */
    bool SolveRegLS(const float *J, const float *b,
                      int nRows, int nCols, float lambda, float *x) const;
};

#endif // KINEMATIC_CALIBRATOR_H
