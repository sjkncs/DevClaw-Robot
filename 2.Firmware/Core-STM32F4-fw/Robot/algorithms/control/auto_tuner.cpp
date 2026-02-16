#include "auto_tuner.h"
#include <cmath>
#include <cstring>


// ========================== Constructor ==========================

AutoTuner::AutoTuner()
{
    memset(&relayConfig, 0, sizeof(RelayConfig_t));
    memset(&relayResult, 0, sizeof(RelayResult_t));
    memset(&mracConfig, 0, sizeof(MRACConfig_t));
    memset(&mracGains, 0, sizeof(PIDGains_t));
}


// ========================== Relay Feedback Tuning ==========================

void AutoTuner::StartRelayTuning(const RelayConfig_t &_config)
{
    relayConfig = _config;
    relayActive = true;
    relayDone = false;
    relayOutput = _config.relayAmplitude;
    relayCycleCount = 0;
    relayPeakMax = -1e10f;
    relayPeakMin = 1e10f;
    relayPeakSum = 0;
    relayPeriodSum = 0;
    relaySampleCount = 0;
    relayLastCrossing = 0;
    relayPositive = true;
    relayPrevError = 0;

    memset(&relayResult, 0, sizeof(RelayResult_t));
}


float AutoTuner::RelayTuningTick(float _position, float _setpoint)
{
    if (!relayActive || relayDone) return 0;

    float error = _setpoint - _position;
    relaySampleCount++;

    // Track peaks
    if (_position > relayPeakMax) relayPeakMax = _position;
    if (_position < relayPeakMin) relayPeakMin = _position;

    // Relay switching with hysteresis
    if (relayPositive && error < -relayConfig.hysteresis)
    {
        // Switch to negative
        relayPositive = false;
        relayOutput = -relayConfig.relayAmplitude;

        // Record half-period
        if (relayCycleCount > 0)
        {
            float halfPeriod = (float)(relaySampleCount - relayLastCrossing)
                              * relayConfig.samplePeriod;
            relayPeriodSum += halfPeriod * 2.0f; // Full period = 2 * half
        }
        relayLastCrossing = relaySampleCount;

        // Record amplitude (peak-to-peak / 2)
        if (relayCycleCount > 0)
        {
            relayPeakSum += (relayPeakMax - relayPeakMin) * 0.5f;
        }
        relayPeakMax = -1e10f;
        relayPeakMin = 1e10f;

        relayCycleCount++;
    }
    else if (!relayPositive && error > relayConfig.hysteresis)
    {
        // Switch to positive
        relayPositive = true;
        relayOutput = relayConfig.relayAmplitude;

        float halfPeriod = (float)(relaySampleCount - relayLastCrossing)
                          * relayConfig.samplePeriod;
        relayPeriodSum += halfPeriod * 2.0f;
        relayLastCrossing = relaySampleCount;

        relayPeakSum += (relayPeakMax - relayPeakMin) * 0.5f;
        relayPeakMax = -1e10f;
        relayPeakMin = 1e10f;

        relayCycleCount++;
    }

    relayPrevError = error;

    // Check completion
    if (relayCycleCount >= relayConfig.maxCycles * 2) // *2 because we count half-cycles
    {
        relayDone = true;
        relayActive = false;

        int fullCycles = relayCycleCount / 2;
        if (fullCycles > 0)
        {
            float avgAmplitude = relayPeakSum / (float)fullCycles;
            float avgPeriod = relayPeriodSum / (float)fullCycles;

            // Ultimate gain: Ku = 4*d / (pi*a)
            // where d = relay amplitude, a = oscillation amplitude
            relayResult.ultimateGain = 4.0f * relayConfig.relayAmplitude
                                     / (3.14159265f * avgAmplitude + 1e-10f);
            relayResult.ultimatePeriod = avgPeriod;
            relayResult.amplitude = avgAmplitude;
            relayResult.cyclesDetected = fullCycles;
            relayResult.valid = (avgAmplitude > 0.1f && avgPeriod > 0.001f);
        }

        relayOutput = 0;
    }

    return relayOutput;
}


AutoTuner::PIDGains_t AutoTuner::ComputeZNGains() const
{
    PIDGains_t gains = {0, 0, 0, 0};
    if (!relayResult.valid) return gains;

    float Ku = relayResult.ultimateGain;
    float Tu = relayResult.ultimatePeriod;

    // Ziegler-Nichols PID rules
    gains.kp = 0.6f * Ku;
    gains.ki = 2.0f * gains.kp / Tu;
    gains.kd = gains.kp * Tu / 8.0f;

    // DCE velocity gain: approximate from bandwidth
    gains.kv = gains.kd * 4.0f; // Heuristic

    return gains;
}


AutoTuner::PIDGains_t AutoTuner::ComputeCohenCoonGains() const
{
    PIDGains_t gains = {0, 0, 0, 0};
    if (!relayResult.valid) return gains;

    float Ku = relayResult.ultimateGain;
    float Tu = relayResult.ultimatePeriod;

    // Cohen-Coon rules (better for processes with dead time)
    // Assuming L/T ≈ 0.1 (small dead time for stepper motors)
    gains.kp = 0.45f * Ku;
    gains.ki = gains.kp / (0.83f * Tu);
    gains.kd = 0.25f * gains.kp * Tu;
    gains.kv = gains.kd * 3.0f;

    return gains;
}


AutoTuner::PIDGains_t AutoTuner::ComputeTyreusLuybenGains() const
{
    PIDGains_t gains = {0, 0, 0, 0};
    if (!relayResult.valid) return gains;

    float Ku = relayResult.ultimateGain;
    float Tu = relayResult.ultimatePeriod;

    // Tyreus-Luyben: more conservative, less overshoot
    gains.kp = Ku / 2.2f;
    gains.ki = gains.kp / (2.2f * Tu);
    gains.kd = gains.kp * Tu / 6.3f;
    gains.kv = gains.kd * 2.5f;

    return gains;
}


// ========================== MRAC ==========================

void AutoTuner::InitMRAC(const MRACConfig_t &_config, const PIDGains_t &_initialGains)
{
    mracConfig = _config;
    mracGains = _initialGains;
    mracActive = true;
    mracRefState[0] = 0;
    mracRefState[1] = 0;
    mracErrorIntegral = 0;
    mracPrevError = 0;
}


AutoTuner::PIDGains_t AutoTuner::MRACTick(float _position, float _velocity,
                                            float _setpoint, float _controlOutput)
{
    if (!mracActive) return mracGains;

    float dt = mracConfig.samplePeriod;
    float wn = mracConfig.wn;
    float zeta = mracConfig.zeta;

    // 1. Update reference model: ddx_m = -2*zeta*wn*dx_m - wn^2*(x_m - r)
    float refAcc = -2.0f * zeta * wn * mracRefState[1]
                   - wn * wn * (mracRefState[0] - _setpoint);
    mracRefState[1] += refAcc * dt;
    mracRefState[0] += mracRefState[1] * dt;

    // 2. Compute tracking error: e = x - x_m
    float e = _position - mracRefState[0];
    float de = _velocity - mracRefState[1];

    // 3. MIT rule adaptation
    // dKp/dt = -gamma_kp * e * (setpoint - position)
    // dKi/dt = -gamma_ki * e * integral(setpoint - position)
    // dKd/dt = -gamma_kd * e * d/dt(setpoint - position)

    float posError = _setpoint - _position;
    mracErrorIntegral += posError * dt;
    float dError = (posError - mracPrevError) / dt;
    mracPrevError = posError;

    // Normalize by control output to prevent gain explosion
    float normFactor = 1.0f / (fabsf(_controlOutput) + 1.0f);

    mracGains.kp += -mracConfig.gamma_kp * e * posError * normFactor * dt;
    mracGains.ki += -mracConfig.gamma_ki * e * mracErrorIntegral * normFactor * dt;
    mracGains.kd += -mracConfig.gamma_kd * e * dError * normFactor * dt;

    // 4. Clamp gains to safe range
    if (mracGains.kp < mracConfig.gain_min) mracGains.kp = mracConfig.gain_min;
    if (mracGains.kp > mracConfig.gain_max) mracGains.kp = mracConfig.gain_max;
    if (mracGains.ki < 0) mracGains.ki = 0;
    if (mracGains.ki > mracConfig.gain_max * 0.5f) mracGains.ki = mracConfig.gain_max * 0.5f;
    if (mracGains.kd < 0) mracGains.kd = 0;
    if (mracGains.kd > mracConfig.gain_max * 0.3f) mracGains.kd = mracConfig.gain_max * 0.3f;

    // 5. Kv tracks Kd proportionally
    mracGains.kv = mracGains.kd * 3.0f;

    return mracGains;
}


void AutoTuner::ResetMRAC()
{
    mracActive = false;
    mracRefState[0] = 0;
    mracRefState[1] = 0;
    mracErrorIntegral = 0;
    mracPrevError = 0;
}
