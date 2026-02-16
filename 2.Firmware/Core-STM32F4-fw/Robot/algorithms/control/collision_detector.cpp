#include "collision_detector.h"
#include <cmath>
#include <cstring>


CollisionDetector::CollisionDetector()
{
    memset(&config, 0, sizeof(Config_t));
    memset(&info, 0, sizeof(CollisionInfo_t));
    memset(momentum, 0, sizeof(momentum));
    memset(residualIntegral, 0, sizeof(residualIntegral));
    memset(prevDq, 0, sizeof(prevDq));
    memset(filteredJerk, 0, sizeof(filteredJerk));
    memset(prevAcc, 0, sizeof(prevAcc));
    reactionTimer = 0;
}


void CollisionDetector::Init(const Config_t &_config)
{
    config = _config;
    Reset();
}


void CollisionDetector::InitDefault(float _dt)
{
    config.dt = _dt;

    // Observer gains: higher = faster detection but more noise sensitivity
    // Tuned for DevClaw Robot with KF-estimated velocities
    float defaultGain[NUM_JOINTS] = {20.0f, 20.0f, 25.0f, 30.0f, 30.0f, 35.0f};
    memcpy(config.observerGain, defaultGain, sizeof(defaultGain));

    // Detection thresholds (N*m at joint side, before reduction)
    // Lower joints need higher thresholds (more gravity/inertia)
    float defaultThresh[NUM_JOINTS] = {1.5f, 2.0f, 1.5f, 0.8f, 0.6f, 0.4f};
    memcpy(config.detectionThreshold, defaultThresh, sizeof(defaultThresh));

    config.energyThreshold = 5.0f;   // Joules
    config.defaultReaction = REACTION_COMPLY;
    config.retractDistance = 5.0f;    // 5 degrees
    config.retractSpeed = 30.0f;     // deg/s
    config.reflexDuration = 0.3f;    // 300ms hold

    Reset();
}


void CollisionDetector::Reset()
{
    memset(&info, 0, sizeof(CollisionInfo_t));
    info.state = STATE_NORMAL;
    memset(momentum, 0, sizeof(momentum));
    memset(residualIntegral, 0, sizeof(residualIntegral));
    memset(prevDq, 0, sizeof(prevDq));
    memset(filteredJerk, 0, sizeof(filteredJerk));
    memset(prevAcc, 0, sizeof(prevAcc));
    reactionTimer = 0;
}


CollisionDetector::CollisionInfo_t CollisionDetector::Update(
    const float _tauCmd[NUM_JOINTS],
    const float _q[NUM_JOINTS],
    const float _dq[NUM_JOINTS],
    const float _gravTorque[NUM_JOINTS],
    const float _corTorque[NUM_JOINTS],
    const float _massDiag[NUM_JOINTS])
{
    float dt = config.dt;

    // ========= Generalized Momentum Observer (De Luca 2006) =========
    //
    // p = M(q) * dq                    (generalized momentum)
    // dp/dt = tau + C^T*dq - g         (momentum dynamics, no external force)
    //       = tau + C^T*dq - g + tau_ext (with external)
    //
    // Residual: r(t) = K_I * integral[tau_cmd - C^T*dq - g + r] dt - p(t) + p(0)
    //
    // Simplified (diagonal M approximation):
    //   r_i(t) = K_i * integral[tau_cmd_i + cor_i - grav_i + r_i] dt - M_i*dq_i

    bool collisionDetected = false;
    int primaryJoint = -1;
    float maxResidualRatio = 0;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        // Current momentum
        float p_current = _massDiag[i] * _dq[i];

        // Integrand: tau_cmd - gravity + coriolis + previous residual
        float integrand = _tauCmd[i] - _gravTorque[i] + _corTorque[i]
                        + info.residual[i];

        // Update integral
        residualIntegral[i] += integrand * dt;

        // Residual = K_I * integral - (p_current - p_initial)
        // Since we reset p_initial to 0 at start:
        info.residual[i] = config.observerGain[i] * residualIntegral[i] - p_current;

        // Store collision torque estimate
        info.collisionTorque[i] = info.residual[i];

        // Check threshold
        float ratio = fabsf(info.residual[i]) / (config.detectionThreshold[i] + 1e-6f);
        if (ratio > 1.0f)
        {
            collisionDetected = true;
            if (ratio > maxResidualRatio)
            {
                maxResidualRatio = ratio;
                primaryJoint = i;
            }
        }

        prevDq[i] = _dq[i];
    }

    // Compute collision energy estimate
    float energy = 0;
    for (int i = 0; i < NUM_JOINTS; i++)
        energy += fabsf(info.residual[i] * _dq[i]);
    info.totalEnergy = energy;

    // Energy-based detection (catches distributed low-level contacts)
    if (energy > config.energyThreshold)
        collisionDetected = true;

    // ========= State Machine =========
    info.detected = false;

    switch (info.state)
    {
    case STATE_NORMAL:
        if (collisionDetected)
        {
            info.state = STATE_DETECTED;
            info.detected = true;
            info.collisionJoint = primaryJoint;
            info.collisionTime = 0;

            // Compute isolation direction (normalized residual)
            float norm = 0;
            for (int i = 0; i < NUM_JOINTS; i++)
                norm += info.residual[i] * info.residual[i];
            norm = sqrtf(norm + 1e-10f);
            for (int i = 0; i < NUM_JOINTS; i++)
                info.isolationDirection[i] = info.residual[i] / norm;

            // Transition to reacting
            info.state = STATE_REACTING;
            reactionTimer = 0;
        }
        break;

    case STATE_DETECTED:
        info.state = STATE_REACTING;
        reactionTimer = 0;
        break;

    case STATE_REACTING:
        info.collisionTime += dt;
        reactionTimer += dt;

        // Check if collision has cleared
        if (!collisionDetected && reactionTimer > 0.1f)
        {
            info.state = STATE_RECOVERED;
        }

        // Timeout: force recovery after 5s
        if (reactionTimer > 5.0f)
        {
            info.state = STATE_RECOVERED;
        }
        break;

    case STATE_RECOVERED:
        // Stay in recovered until explicit Reset() call
        break;
    }

    return info;
}


CollisionDetector::CollisionInfo_t CollisionDetector::UpdateSimplified(
    const float _dq[NUM_JOINTS],
    const float _ddq[NUM_JOINTS],
    const float _current[NUM_JOINTS])
{
    // Simplified collision detection without dynamics model:
    // Monitor filtered jerk (derivative of acceleration) for sudden spikes
    // Also monitor current vs expected (velocity-proportional) current

    float dt = config.dt;
    bool collisionDetected = false;
    int primaryJoint = -1;
    float maxRatio = 0;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        // Jerk estimation (filtered)
        float jerk = (_ddq[i] - prevAcc[i]) / dt;
        filteredJerk[i] = JERK_FILTER_ALPHA * jerk + (1.0f - JERK_FILTER_ALPHA) * filteredJerk[i];
        prevAcc[i] = _ddq[i];

        // Residual = |filtered_jerk| / threshold
        // Jerk spike indicates sudden force change (collision)
        float jerkThreshold = config.detectionThreshold[i] * 1000.0f; // Scale for jerk units
        info.residual[i] = filteredJerk[i];

        float ratio = fabsf(filteredJerk[i]) / (jerkThreshold + 1e-6f);

        // Also check current anomaly: if |current| >> expected
        // Expected current ≈ proportional to |velocity| (friction model)
        float expectedCurrent = fabsf(_dq[i]) * 0.1f + 0.5f; // Rough linear model
        float currentRatio = fabsf(_current[i]) / (expectedCurrent + 0.1f);
        if (currentRatio > 3.0f)
            ratio = fmaxf(ratio, currentRatio / 3.0f);

        if (ratio > 1.0f)
        {
            collisionDetected = true;
            if (ratio > maxRatio)
            {
                maxRatio = ratio;
                primaryJoint = i;
            }
        }
    }

    // Use same state machine as full detector
    info.detected = false;
    if (info.state == STATE_NORMAL && collisionDetected)
    {
        info.state = STATE_DETECTED;
        info.detected = true;
        info.collisionJoint = primaryJoint;
        info.collisionTime = 0;
        info.state = STATE_REACTING;
        reactionTimer = 0;

        float norm = 0;
        for (int i = 0; i < NUM_JOINTS; i++)
            norm += info.residual[i] * info.residual[i];
        norm = sqrtf(norm + 1e-10f);
        for (int i = 0; i < NUM_JOINTS; i++)
            info.isolationDirection[i] = info.residual[i] / norm;
    }
    else if (info.state == STATE_REACTING)
    {
        reactionTimer += dt;
        if (!collisionDetected && reactionTimer > 0.1f)
            info.state = STATE_RECOVERED;
        if (reactionTimer > 5.0f)
            info.state = STATE_RECOVERED;
    }

    return info;
}


bool CollisionDetector::GetReactionCommand(const float _currentJoints[NUM_JOINTS],
                                            float _reactionJoints[NUM_JOINTS])
{
    if (info.state != STATE_REACTING)
    {
        memcpy(_reactionJoints, _currentJoints, NUM_JOINTS * sizeof(float));
        return false;
    }

    switch (config.defaultReaction)
    {
    case REACTION_STOP:
        // Hold current position (motors will be disabled externally)
        memcpy(_reactionJoints, _currentJoints, NUM_JOINTS * sizeof(float));
        return true;

    case REACTION_RETRACT:
        // Move opposite to collision direction
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            float retractDelta = -info.isolationDirection[i] * config.retractDistance;
            _reactionJoints[i] = _currentJoints[i] + retractDelta;
        }
        return true;

    case REACTION_FLOAT:
        // Return current position (gravity comp applied externally)
        memcpy(_reactionJoints, _currentJoints, NUM_JOINTS * sizeof(float));
        return true;

    case REACTION_REFLEX:
        if (reactionTimer < 0.05f)
        {
            // Quick retract phase
            for (int i = 0; i < NUM_JOINTS; i++)
            {
                float retractDelta = -info.isolationDirection[i] * config.retractDistance * 0.5f;
                _reactionJoints[i] = _currentJoints[i] + retractDelta;
            }
        }
        else
        {
            // Hold phase
            memcpy(_reactionJoints, _currentJoints, NUM_JOINTS * sizeof(float));
        }
        return true;

    case REACTION_COMPLY:
        // Return current position (admittance controller handles compliance externally)
        memcpy(_reactionJoints, _currentJoints, NUM_JOINTS * sizeof(float));
        return true;
    }

    return false;
}


void CollisionDetector::SetReactionStrategy(ReactionStrategy_t _strategy)
{
    config.defaultReaction = _strategy;
}


void CollisionDetector::SetThresholds(const float _thresholds[NUM_JOINTS])
{
    memcpy(config.detectionThreshold, _thresholds, NUM_JOINTS * sizeof(float));
}
