#include "kalman_filter.h"
#include <cmath>
#include <cstring>


// ========================== JointKalmanFilter ==========================

JointKalmanFilter::JointKalmanFilter()
{
    memset(&state, 0, sizeof(State_t));
    memset(P, 0, sizeof(P));
    memset(K, 0, sizeof(K));
    innovation = 0;
    innovationSqSum = 0;
    innovationCount = 0;
    dt = 0;
    dt2_half = 0;
    B_acc = 0;
}


void JointKalmanFilter::Init(const Config_t &_config)
{
    config = _config;
    dt = config.dt;
    dt2_half = 0.5f * dt * dt;

    // B_acc: maps control input (current in mA) to acceleration (counts/s^2)
    // For a stepper motor: tau = Kt * I, alpha = tau / J_total
    // Approximate: 1mA -> ~1 count/s^2 (needs calibration)
    B_acc = 1.0f;

    // Initialize covariance
    P[0] = config.p_init;  // P00: position variance
    P[1] = 0;              // P01
    P[2] = 0;              // P02
    P[3] = config.p_init;  // P11: velocity variance
    P[4] = 0;              // P12
    P[5] = config.p_init;  // P22: acceleration variance

    innovation = 0;
    innovationSqSum = 0;
    innovationCount = 0;
}


void JointKalmanFilter::InitDefault(int32_t _controlFrequency)
{
    Config_t cfg;
    cfg.dt = 1.0f / (float)_controlFrequency;

    // Process noise Q - tuned for stepper motor encoder at 20kHz
    // Higher Q = trust measurements more, faster response but more noise
    // Lower Q = trust model more, smoother but slower response
    cfg.q_pos = 0.1f;       // Position process noise (counts^2)
    cfg.q_vel = 100.0f;     // Velocity process noise (counts/s)^2
    cfg.q_acc = 50000.0f;   // Acceleration process noise (counts/s^2)^2

    // Measurement noise R - encoder quantization noise
    // For 14-bit encoder with 256 subdivision: ~1 count RMS noise
    cfg.r_pos = 1.0f;       // Position measurement noise (counts^2)

    cfg.p_init = 1000.0f;   // Initial uncertainty

    Init(cfg);
}


void JointKalmanFilter::Reset(float _initialPosition)
{
    state.position = _initialPosition;
    state.velocity = 0;
    state.acceleration = 0;

    P[0] = config.p_init;
    P[1] = 0;
    P[2] = 0;
    P[3] = config.p_init;
    P[4] = 0;
    P[5] = config.p_init;

    innovation = 0;
    innovationSqSum = 0;
    innovationCount = 0;
}


JointKalmanFilter::State_t JointKalmanFilter::Update(float _measuredPosition, float _controlInput)
{
    // =================== Prediction Step ===================
    // x_pred = A * x + B * u
    // A = [1  dt  0.5*dt^2]    B = [0     ]
    //     [0  1   dt      ]        [0     ]
    //     [0  0   1       ]        [B_acc ]

    float x_pred[3];
    x_pred[0] = state.position + state.velocity * dt + state.acceleration * dt2_half;
    x_pred[1] = state.velocity + state.acceleration * dt;
    x_pred[2] = state.acceleration + B_acc * _controlInput;

    // P_pred = A * P * A^T + Q
    // Expanding for 3x3 symmetric storage:
    // Let's denote old P as p, new predicted P as pp
    float p00 = P[0], p01 = P[1], p02 = P[2];
    float p11 = P[3], p12 = P[4], p22 = P[5];

    // A*P row 0: [p00+dt*p01+h*p02,  p01+dt*p11+h*p12,  p02+dt*p12+h*p22]
    // where h = dt2_half
    float h = dt2_half;
    float ap00 = p00 + dt * p01 + h * p02;
    float ap01 = p01 + dt * p11 + h * p12;
    float ap02 = p02 + dt * p12 + h * p22;

    // A*P row 1: [p01+dt*p02,  p11+dt*p12,  p12+dt*p22]  (skipping the dt*p01 etc)
    float ap10 = p01 + dt * p02;
    float ap11 = p11 + dt * p12;
    float ap12_v = p12 + dt * p22;

    // A*P row 2: [p02, p12, p22]  (A row 2 is [0,0,1])
    float ap20 = p02;
    float ap21 = p12;
    float ap22 = p22;

    // P_pred = (A*P) * A^T + Q
    // A^T column 0: [1, dt, h]^T
    // A^T column 1: [0, 1, dt]^T
    // A^T column 2: [0, 0, 1]^T
    float pp00 = ap00 * 1 + ap01 * dt + ap02 * h + config.q_pos;
    float pp01 = ap00 * 0 + ap01 * 1  + ap02 * dt;
    float pp02 = ap00 * 0 + ap01 * 0  + ap02 * 1;
    float pp11 = ap10 * 0 + ap11 * 1  + ap12_v * dt + config.q_vel;
    float pp12 = ap10 * 0 + ap11 * 0  + ap12_v * 1;
    float pp22 = ap20 * 0 + ap21 * 0  + ap22 * 1 + config.q_acc;

    // =================== Update Step ===================
    // C = [1, 0, 0]
    // Innovation: y - C * x_pred = measured - x_pred[0]
    innovation = _measuredPosition - x_pred[0];

    // S = C * P_pred * C^T + R = pp00 + R
    float S = pp00 + config.r_pos;

    // Avoid division by zero
    if (fabsf(S) < 1e-10f) S = 1e-10f;
    float S_inv = 1.0f / S;

    // K = P_pred * C^T * S^{-1} = [pp00, pp01, pp02]^T / S
    K[0] = pp00 * S_inv;
    K[1] = pp01 * S_inv;
    K[2] = pp02 * S_inv;

    // x_hat = x_pred + K * innovation
    state.position     = x_pred[0] + K[0] * innovation;
    state.velocity     = x_pred[1] + K[1] * innovation;
    state.acceleration = x_pred[2] + K[2] * innovation;

    // P = (I - K*C) * P_pred
    // K*C = [K0, 0, 0; K1, 0, 0; K2, 0, 0]
    // (I-K*C) = [1-K0, 0, 0; -K1, 1, 0; -K2, 0, 1]
    float ik0 = 1.0f - K[0];
    P[0] = ik0 * pp00;                           // (1-K0)*pp00
    P[1] = ik0 * pp01;                           // (1-K0)*pp01
    P[2] = ik0 * pp02;                           // (1-K0)*pp02
    P[3] = -K[1] * pp01 + pp11;                  // -K1*pp01 + pp11
    P[4] = -K[1] * pp02 + pp12;                  // -K1*pp02 + pp12
    P[5] = -K[2] * pp02 + pp22;                  // -K2*pp02 + pp22

    // Update innovation statistics for adaptive tuning
    innovationSqSum += innovation * innovation;
    innovationCount += 1.0f;

    return state;
}


void JointKalmanFilter::Predict(float _controlInput)
{
    // Prediction only (no measurement update)
    state.position += state.velocity * dt + state.acceleration * dt2_half;
    state.velocity += state.acceleration * dt;
    state.acceleration += B_acc * _controlInput;

    float p00 = P[0], p01 = P[1], p02 = P[2];
    float p11 = P[3], p12 = P[4], p22 = P[5];
    float h = dt2_half;

    float ap00 = p00 + dt * p01 + h * p02;
    float ap01 = p01 + dt * p11 + h * p12;
    float ap02 = p02 + dt * p12 + h * p22;
    float ap10 = p01 + dt * p02;
    float ap11 = p11 + dt * p12;
    float ap12_v = p12 + dt * p22;
    float ap22 = p22;

    P[0] = ap00 + ap01 * dt + ap02 * h + config.q_pos;
    P[1] = ap01 + ap02 * dt;
    P[2] = ap02;
    P[3] = ap11 + ap12_v * dt + config.q_vel;
    P[4] = ap12_v;
    P[5] = ap22 + config.q_acc;
}


void JointKalmanFilter::AdaptiveNoiseUpdate()
{
    // Adaptive R and Q based on innovation sequence
    // If normalized innovation squared (NIS) >> 1, increase Q
    // If NIS << 1, decrease Q (or increase R)
    if (innovationCount < INNOVATION_WINDOW) return;

    float avgInnovSq = innovationSqSum / innovationCount;
    // Expected innovation variance = C*P*C^T + R = P[0] + R
    float expectedVar = P[0] + config.r_pos;

    float nis = avgInnovSq / (expectedVar + 1e-10f);

    // Adjust Q if NIS deviates from 1.0
    if (nis > 2.0f)
    {
        // Innovations too large: increase process noise (trust measurement more)
        config.q_vel *= 1.05f;
        config.q_acc *= 1.05f;
    }
    else if (nis < 0.5f)
    {
        // Innovations too small: decrease process noise (trust model more)
        config.q_vel *= 0.95f;
        config.q_acc *= 0.95f;
    }

    // Clamp Q values
    if (config.q_vel < 1.0f) config.q_vel = 1.0f;
    if (config.q_vel > 100000.0f) config.q_vel = 100000.0f;
    if (config.q_acc < 100.0f) config.q_acc = 100.0f;
    if (config.q_acc > 1e8f) config.q_acc = 1e8f;

    // Reset statistics
    innovationSqSum = 0;
    innovationCount = 0;
}


// ========================== KalmanSmoother ==========================

void KalmanSmoother::RecordStep(const JointKalmanFilter::State_t &_filtState,
                                 const JointKalmanFilter::State_t &_predState,
                                 const float _P_filt[6],
                                 const float _P_pred[6])
{
    if (count >= MAX_SAMPLES) return;

    records[count].filtState = _filtState;
    records[count].predState = _predState;
    memcpy(records[count].P_filt, _P_filt, 6 * sizeof(float));
    memcpy(records[count].P_pred, _P_pred, 6 * sizeof(float));
    count++;
}


int KalmanSmoother::RunSmoother(float _dt)
{
    if (count < 2) return 0;

    float h = 0.5f * _dt * _dt;

    // Initialize last smoothed state = last filtered state
    smoothed[count - 1].position = records[count - 1].filtState.position;
    smoothed[count - 1].velocity = records[count - 1].filtState.velocity;
    smoothed[count - 1].acceleration = records[count - 1].filtState.acceleration;

    // Backward pass: RTS smoother
    for (int k = count - 2; k >= 0; k--)
    {
        // G_k = P_filt_k * A^T * P_pred_{k+1}^{-1}
        // For scalar-like approximation (diagonal dominant):
        // G ≈ P_filt / P_pred for each state component

        float *Pf = records[k].P_filt;
        float *Pp = records[k + 1].P_pred;

        // Smoother gain (simplified 3x3 computation)
        // G = P_filt * A^T * inv(P_pred)
        // A^T = [1, 0, 0; dt, 1, 0; h, dt, 1]
        // For efficiency, compute G column-by-column using P_pred diagonal approx

        // Full computation: P_filt * A^T
        // Row 0: [Pf00, Pf01*dt + Pf00, Pf02*h + Pf01*dt + Pf00] ... too complex
        // Use diagonal approximation for real-time:
        float g00 = (Pp[0] > 1e-10f) ? (Pf[0] + Pf[1] * _dt + Pf[2] * h) / Pp[0] : 0;
        float g10 = (Pp[3] > 1e-10f) ? (Pf[1] + Pf[3] * _dt + Pf[4] * h) / Pp[3] : 0;
        float g20 = (Pp[5] > 1e-10f) ? (Pf[2] + Pf[4] * _dt + Pf[5] * h) / Pp[5] : 0;

        // Clamp gains to prevent divergence
        if (g00 > 1.0f) g00 = 1.0f; if (g00 < -1.0f) g00 = -1.0f;
        if (g10 > 1.0f) g10 = 1.0f; if (g10 < -1.0f) g10 = -1.0f;
        if (g20 > 1.0f) g20 = 1.0f; if (g20 < -1.0f) g20 = -1.0f;

        // x_smooth_k = x_filt_k + G * (x_smooth_{k+1} - x_pred_{k+1})
        float dp = smoothed[k + 1].position - records[k + 1].predState.position;
        float dv = smoothed[k + 1].velocity - records[k + 1].predState.velocity;
        float da = smoothed[k + 1].acceleration - records[k + 1].predState.acceleration;

        smoothed[k].position     = records[k].filtState.position     + g00 * dp;
        smoothed[k].velocity     = records[k].filtState.velocity     + g10 * dv;
        smoothed[k].acceleration = records[k].filtState.acceleration + g20 * da;
    }

    return count;
}
