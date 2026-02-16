#include "dmp.h"
#include <cmath>
#include <cstring>


// ========================== DMP (Single DOF) ==========================

DMP::DMP()
{
    memset(&config, 0, sizeof(Config_t));
    memset(&learned, 0, sizeof(LearnedDMP_t));
    memset(&state, 0, sizeof(State_t));
}


void DMP::Init(const Config_t &_config)
{
    config = _config;
    if (config.betaZ <= 0)
        config.betaZ = config.alphaZ / 4.0f;
    SetupBasisFunctions();
}


void DMP::InitDefault(float _dt)
{
    config.numBasis = 25;
    config.alphaZ = 25.0f;
    config.betaZ = config.alphaZ / 4.0f; // Critically damped
    config.alphaX = 1.0f;
    config.tau = 1.0f;
    config.dt = _dt;
    SetupBasisFunctions();
}


void DMP::SetupBasisFunctions()
{
    int N = config.numBasis;
    if (N > MAX_BASIS) N = MAX_BASIS;
    if (N < 1) N = 1;

    learned.numBasis = N;

    // Distribute centers evenly in phase space x in [1, ~0]
    // x decays exponentially: x(t) = exp(-alpha_x * t / tau)
    // So centers in time: c_i = exp(-alpha_x * t_i / tau)
    // where t_i are evenly spaced in [0, tau]

    for (int i = 0; i < N; i++)
    {
        float t_ratio = (float)i / (float)(N - 1); // 0 to 1
        learned.centers[i] = expf(-config.alphaX * t_ratio);

        // Width: overlap between adjacent bases
        if (i < N - 1)
        {
            float nextCenter = expf(-config.alphaX * (float)(i + 1) / (float)(N - 1));
            float diff = learned.centers[i] - nextCenter;
            learned.widths[i] = 1.0f / (diff * diff + 1e-6f) * 0.5f;
        }
        else
        {
            learned.widths[i] = learned.widths[i > 0 ? i - 1 : 0];
        }
    }
}


float DMP::GaussianBasis(float _x, float _center, float _width)
{
    float diff = _x - _center;
    return expf(-_width * diff * diff);
}


float DMP::ComputeForcing(float _x) const
{
    float numerator = 0;
    float denominator = 0;

    for (int i = 0; i < learned.numBasis; i++)
    {
        float psi = GaussianBasis(_x, learned.centers[i], learned.widths[i]);
        numerator += psi * learned.weights[i] * _x;
        denominator += psi;
    }

    if (denominator < 1e-10f) denominator = 1e-10f;

    float goalDiff = learned.goal - learned.y0;
    if (fabsf(goalDiff) < 1e-6f) goalDiff = 1e-6f;

    return (numerator / denominator) * goalDiff;
}


bool DMP::LearnFromDemo(const float *_trajectory, int _numPoints, float _duration)
{
    if (_numPoints < 2 || !_trajectory) return false;

    float dt_demo = _duration / (float)(_numPoints - 1);
    learned.y0 = _trajectory[0];
    learned.goal = _trajectory[_numPoints - 1];
    learned.tau = _duration;
    config.tau = _duration;

    // Compute velocity and acceleration via finite differences
    float *vel = new float[_numPoints];
    float *acc = new float[_numPoints];

    // Central differences (forward/backward at endpoints)
    vel[0] = (_trajectory[1] - _trajectory[0]) / dt_demo;
    vel[_numPoints - 1] = (_trajectory[_numPoints - 1] - _trajectory[_numPoints - 2]) / dt_demo;
    for (int i = 1; i < _numPoints - 1; i++)
        vel[i] = (_trajectory[i + 1] - _trajectory[i - 1]) / (2.0f * dt_demo);

    acc[0] = (vel[1] - vel[0]) / dt_demo;
    acc[_numPoints - 1] = (vel[_numPoints - 1] - vel[_numPoints - 2]) / dt_demo;
    for (int i = 1; i < _numPoints - 1; i++)
        acc[i] = (vel[i + 1] - vel[i - 1]) / (2.0f * dt_demo);

    bool result = LearnFromDemoFull(_trajectory, vel, acc, _numPoints, _duration);

    delete[] vel;
    delete[] acc;
    return result;
}


bool DMP::LearnFromDemoFull(const float *_pos, const float *_vel, const float *_acc,
                             int _numPoints, float _duration)
{
    if (_numPoints < 2) return false;

    float dt_demo = _duration / (float)(_numPoints - 1);
    learned.y0 = _pos[0];
    learned.goal = _pos[_numPoints - 1];
    learned.tau = _duration;
    config.tau = _duration;

    float goalDiff = learned.goal - learned.y0;
    if (fabsf(goalDiff) < 1e-6f) goalDiff = 1e-6f;

    // Compute desired forcing function from demonstration:
    // f_target(t) = tau^2 * ddq - alpha_z * (beta_z * (g - q) - tau * dq)
    // Then solve: f_target = sum(psi_i * w_i * x) / sum(psi_i) * (g - y0)

    // Locally Weighted Regression (LWR) to find weights:
    // w_i = (Phi_i^T * S_i * f_target) / (Phi_i^T * S_i * Phi_i)
    // where S_i = diag(psi_i(x_k)) and Phi_i = x_k * (g - y0)

    int N = learned.numBasis;
    float tau = config.tau;

    for (int i = 0; i < N; i++)
    {
        float sPhiF = 0; // sum(psi * phi * f_target)
        float sPhiPhi = 0; // sum(psi * phi * phi)

        for (int k = 0; k < _numPoints; k++)
        {
            float t = (float)k * dt_demo;
            float x = expf(-config.alphaX * t / tau);

            // Desired forcing function
            float f_target = tau * tau * _acc[k]
                           - config.alphaZ * (config.betaZ * (learned.goal - _pos[k]) - tau * _vel[k]);

            float psi = GaussianBasis(x, learned.centers[i], learned.widths[i]);
            float phi = x * goalDiff;

            sPhiF += psi * phi * f_target;
            sPhiPhi += psi * phi * phi;
        }

        learned.weights[i] = (fabsf(sPhiPhi) > 1e-10f) ? (sPhiF / sPhiPhi) : 0;
    }

    learned.learned = true;
    return true;
}


void DMP::Reset(float _y0, float _goal, float _tau)
{
    state.y = _y0;
    state.z = 0;
    state.x = 1.0f;
    state.f = 0;
    state.dy = 0;
    state.ddy = 0;

    if (_tau > 0)
        config.tau = _tau;
    else
        config.tau = learned.tau;

    // Update goal for spatial scaling
    learned.goal = _goal;
    // Keep original y0 for weight scaling, but update current start
    learned.y0 = _y0;
}


DMP::State_t DMP::Step()
{
    return StepWithGoal(learned.goal);
}


DMP::State_t DMP::StepWithGoal(float _newGoal)
{
    if (!learned.learned) return state;

    float dt = config.dt;
    float tau = config.tau;
    float goal = _newGoal;

    // Save goal for forcing function computation
    float origGoal = learned.goal;
    learned.goal = goal;

    // Canonical system: tau * dx = -alpha_x * x
    float dx = -config.alphaX * state.x / tau;
    state.x += dx * dt;
    if (state.x < 1e-8f) state.x = 1e-8f;

    // Forcing function
    state.f = ComputeForcing(state.x);

    // Transformation system:
    // tau * dz = alpha_z * (beta_z * (g - y) - z) + f
    // tau * dy = z
    float dz = (config.alphaZ * (config.betaZ * (goal - state.y) - state.z) + state.f) / tau;
    state.z += dz * dt;

    state.dy = state.z / tau;
    state.y += state.dy * dt;
    state.ddy = dz / tau;

    learned.goal = origGoal; // Restore for consistent forcing function

    return state;
}


void DMP::SetLearnedDMP(const LearnedDMP_t &_dmp)
{
    memcpy(&learned, &_dmp, sizeof(LearnedDMP_t));
}


// ========================== MultiDOF_DMP ==========================

MultiDOF_DMP::MultiDOF_DMP()
{
    memset(&multiState, 0, sizeof(MultiState_t));
    sharedPhase = 1.0f;
    sharedAlphaX = 1.0f;
    sharedTau = 1.0f;
    sharedDt = 0.001f;
    recordCount = 0;
}


void MultiDOF_DMP::Init(float _dt, int _numBasis)
{
    sharedDt = _dt;
    for (int d = 0; d < NUM_DOF; d++)
    {
        DMP::Config_t cfg;
        cfg.numBasis = _numBasis;
        cfg.alphaZ = 25.0f;
        cfg.betaZ = cfg.alphaZ / 4.0f;
        cfg.alphaX = 1.0f;
        cfg.tau = 1.0f;
        cfg.dt = _dt;
        dmpDof[d].Init(cfg);
    }
    recordCount = 0;
}


bool MultiDOF_DMP::LearnFromDemo(const float _trajectories[][NUM_DOF],
                                   int _numPoints, float _duration)
{
    if (_numPoints < 2) return false;

    bool allOk = true;
    for (int d = 0; d < NUM_DOF; d++)
    {
        // Extract single DOF trajectory
        float *singleDof = new float[_numPoints];
        for (int i = 0; i < _numPoints; i++)
            singleDof[i] = _trajectories[i][d];

        if (!dmpDof[d].LearnFromDemo(singleDof, _numPoints, _duration))
            allOk = false;

        delete[] singleDof;
    }

    sharedTau = _duration;
    sharedPhase = 1.0f;
    return allOk;
}


void MultiDOF_DMP::Reset(const float _start[NUM_DOF], const float _goal[NUM_DOF],
                           float _tau)
{
    for (int d = 0; d < NUM_DOF; d++)
        dmpDof[d].Reset(_start[d], _goal[d], _tau);

    sharedPhase = 1.0f;
    if (_tau > 0) sharedTau = _tau;
    multiState.complete = false;
}


MultiDOF_DMP::MultiState_t MultiDOF_DMP::Step()
{
    for (int d = 0; d < NUM_DOF; d++)
    {
        DMP::State_t s = dmpDof[d].Step();
        multiState.y[d] = s.y;
        multiState.dy[d] = s.dy;
        multiState.ddy[d] = s.ddy;
    }

    // Use first DOF's phase as shared phase
    multiState.x = dmpDof[0].GetState().x;
    multiState.complete = (multiState.x < 0.01f);

    return multiState;
}


MultiDOF_DMP::MultiState_t MultiDOF_DMP::StepWithGoal(const float _newGoal[NUM_DOF])
{
    for (int d = 0; d < NUM_DOF; d++)
    {
        DMP::State_t s = dmpDof[d].StepWithGoal(_newGoal[d]);
        multiState.y[d] = s.y;
        multiState.dy[d] = s.dy;
        multiState.ddy[d] = s.ddy;
    }

    multiState.x = dmpDof[0].GetState().x;
    multiState.complete = (multiState.x < 0.01f);

    return multiState;
}


bool MultiDOF_DMP::IsComplete() const
{
    return multiState.complete;
}


void MultiDOF_DMP::RecordPoint(const float _joints[NUM_DOF])
{
    if (recordCount >= DMP::MAX_DEMO_POINTS) return;
    memcpy(recordBuffer[recordCount], _joints, NUM_DOF * sizeof(float));
    recordCount++;
}


bool MultiDOF_DMP::FinishRecording(float _duration)
{
    if (recordCount < 2) return false;

    bool ok = LearnFromDemo(recordBuffer, recordCount, _duration);
    recordCount = 0;
    return ok;
}
