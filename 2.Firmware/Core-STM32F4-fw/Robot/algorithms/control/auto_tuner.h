#ifndef AUTO_TUNER_H
#define AUTO_TUNER_H

#include <cstdint>

/**
 * @brief Online PID/DCE Auto-Tuner for Motor Controllers
 *
 * Replaces hard-coded DCE parameters (kp, kv, ki, kd) with adaptive values.
 *
 * Three complementary approaches:
 *
 * 1. Relay Feedback Auto-Tuning (Åström-Hägglund method):
 *    - Applies relay (bang-bang) excitation to identify ultimate gain Ku and period Tu
 *    - Ziegler-Nichols rules: Kp = 0.6*Ku, Ki = 2*Kp/Tu, Kd = Kp*Tu/8
 *    - Used for initial tuning or re-tuning after hardware changes
 *
 * 2. Model Reference Adaptive Control (MRAC):
 *    - Reference model: G_m(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
 *    - Adaptation law (MIT rule): dtheta/dt = -gamma * e * x
 *    - Continuously adjusts gains to match desired closed-loop behavior
 *
 * 3. Extremum Seeking (ES) for DCE gain optimization:
 *    - Minimizes a cost function J = integral(e^2 + rho*u^2)
 *    - Perturbation-based gradient estimation
 *    - Slower but handles nonlinear systems
 *
 * Reference:
 *   Åström & Hägglund, "Automatic Tuning of PID Controllers", ISA, 1988
 *   Ioannou & Sun, "Robust Adaptive Control", Dover, 2012
 *
 * Target: IEEE T-Mech / RA-L
 */
class AutoTuner
{
public:
    // ==================== Relay Feedback Tuning ====================
    struct RelayConfig_t
    {
        float relayAmplitude;   // Relay output amplitude (current units)
        float hysteresis;       // Switching hysteresis (position counts)
        int32_t maxCycles;      // Number of oscillation cycles to measure (default 5)
        float samplePeriod;     // Sample period (s)
    };

    struct RelayResult_t
    {
        float ultimateGain;     // Ku: ultimate gain
        float ultimatePeriod;   // Tu: ultimate period (s)
        float amplitude;        // Oscillation amplitude (counts)
        int32_t cyclesDetected;
        bool valid;
    };

    struct PIDGains_t
    {
        float kp;
        float ki;
        float kd;
        float kv;  // For DCE: velocity feedforward gain
    };

    // ==================== MRAC ====================
    struct MRACConfig_t
    {
        float wn;               // Reference model natural frequency (rad/s)
        float zeta;             // Reference model damping ratio (default 0.8)
        float gamma_kp;         // Adaptation rate for kp
        float gamma_ki;         // Adaptation rate for ki
        float gamma_kd;         // Adaptation rate for kd
        float gain_min;         // Minimum allowed gain value
        float gain_max;         // Maximum allowed gain value
        float samplePeriod;     // (s)
    };

    AutoTuner();

    // ==================== Relay Feedback Methods ====================

    /**
     * @brief Start relay feedback auto-tuning experiment
     */
    void StartRelayTuning(const RelayConfig_t &_config);

    /**
     * @brief Feed one sample to relay tuning
     * @param _position  Current position (counts)
     * @param _setpoint  Desired position (counts)
     * @return Control output (current command) during relay experiment
     */
    float RelayTuningTick(float _position, float _setpoint);

    /**
     * @brief Check if relay tuning is complete
     */
    bool IsRelayTuningDone() const { return relayDone; }

    /**
     * @brief Get relay tuning result
     */
    const RelayResult_t &GetRelayResult() const { return relayResult; }

    /**
     * @brief Compute PID gains from relay result using Ziegler-Nichols
     */
    PIDGains_t ComputeZNGains() const;

    /**
     * @brief Compute PID gains using Cohen-Coon (better for integrating processes)
     */
    PIDGains_t ComputeCohenCoonGains() const;

    /**
     * @brief Compute PID gains using Tyreus-Luyben (more conservative, less overshoot)
     */
    PIDGains_t ComputeTyreusLuybenGains() const;

    // ==================== MRAC Methods ====================

    /**
     * @brief Initialize MRAC with configuration
     */
    void InitMRAC(const MRACConfig_t &_config, const PIDGains_t &_initialGains);

    /**
     * @brief MRAC adaptation tick - call every control cycle
     * @param _position  Current position
     * @param _velocity  Current velocity
     * @param _setpoint  Desired position
     * @param _controlOutput  Current control output (for normalization)
     * @return Updated PID gains
     */
    PIDGains_t MRACTick(float _position, float _velocity,
                         float _setpoint, float _controlOutput);

    /**
     * @brief Get current MRAC gains
     */
    const PIDGains_t &GetMRACGains() const { return mracGains; }

    /**
     * @brief Reset MRAC to initial gains
     */
    void ResetMRAC();

    // ==================== General ====================

    /**
     * @brief Check if any tuning process is active
     */
    bool IsTuningActive() const { return relayActive || mracActive; }

private:
    // Relay feedback state
    RelayConfig_t relayConfig;
    RelayResult_t relayResult;
    bool relayActive = false;
    bool relayDone = false;
    float relayOutput = 0;
    int32_t relayCycleCount = 0;
    float relayPeakMax = 0, relayPeakMin = 0;
    float relayPeakSum = 0;
    float relayPeriodSum = 0;
    int32_t relaySampleCount = 0;
    int32_t relayLastCrossing = 0;
    bool relayPositive = true;
    float relayPrevError = 0;

    // MRAC state
    MRACConfig_t mracConfig;
    PIDGains_t mracGains;
    bool mracActive = false;
    float mracRefState[2] = {0, 0};  // Reference model state [pos, vel]
    float mracErrorIntegral = 0;
    float mracPrevError = 0;
};

#endif // AUTO_TUNER_H
