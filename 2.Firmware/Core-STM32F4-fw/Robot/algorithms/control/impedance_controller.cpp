#include "impedance_controller.h"
#include <cmath>
#include <cstring>


ImpedanceController::ImpedanceController()
{
    memset(&config, 0, sizeof(Config_t));
    memset(&cartImp, 0, sizeof(CartesianImpedance_t));
    memset(&jointImp, 0, sizeof(JointImpedance_t));
    memset(&state, 0, sizeof(State_t));
    memset(Kd_max, 0, sizeof(Kd_max));
    memset(Kd_min, 0, sizeof(Kd_min));
    memset(Kd_current, 0, sizeof(Kd_current));
    alpha_var = 0.01f;
}


void ImpedanceController::Init(float _dt, Mode_t _mode)
{
    config.dt = _dt;
    config.mode = _mode;
    config.forceDeadzone = 0.2f;       // 0.2 N deadzone
    config.maxComplianceDisp = 50.0f;  // 50mm or 50deg max displacement
    config.dampingRatio = 0.7f;        // Slightly underdamped for responsiveness
    config.passivityEnforced = true;

    // Default Cartesian impedance (compliant but stable)
    for (int i = 0; i < 3; i++)
    {
        cartImp.Md[i] = 2.0f;     // 2 kg virtual mass (translational)
        cartImp.Kd[i] = 200.0f;   // 200 N/m stiffness
    }
    for (int i = 3; i < DOF; i++)
    {
        cartImp.Md[i] = 0.1f;     // 0.1 kg*m^2 virtual inertia (rotational)
        cartImp.Kd[i] = 5.0f;     // 5 N*m/rad rotational stiffness
    }
    ComputeDamping();

    // Default joint impedance
    for (int i = 0; i < DOF; i++)
    {
        jointImp.Md[i] = 0.5f;
        jointImp.Kd[i] = 50.0f;
    }

    // Variable impedance defaults
    for (int i = 0; i < DOF; i++)
    {
        Kd_max[i] = cartImp.Kd[i] * 2.0f;
        Kd_min[i] = cartImp.Kd[i] * 0.1f;
        Kd_current[i] = cartImp.Kd[i];
    }
    alpha_var = 0.01f;

    Reset();
}


void ImpedanceController::SetCartesianImpedance(const CartesianImpedance_t &_imp)
{
    memcpy(&cartImp, &_imp, sizeof(CartesianImpedance_t));
    ComputeDamping();
}


void ImpedanceController::SetJointImpedance(const JointImpedance_t &_imp)
{
    memcpy(&jointImp, &_imp, sizeof(JointImpedance_t));
}


void ImpedanceController::SetUniformStiffness(float _stiffness)
{
    for (int i = 0; i < DOF; i++)
    {
        cartImp.Kd[i] = _stiffness;
        Kd_current[i] = _stiffness;
    }
    ComputeDamping();
}


void ImpedanceController::SetDampingRatio(float _zeta)
{
    config.dampingRatio = _zeta;
    ComputeDamping();
}


void ImpedanceController::ComputeDamping()
{
    // B = 2 * zeta * sqrt(K * M)  (critical damping formula)
    for (int i = 0; i < DOF; i++)
    {
        float km = cartImp.Kd[i] * cartImp.Md[i];
        cartImp.Bd[i] = 2.0f * config.dampingRatio * sqrtf(fabsf(km));

        float jkm = jointImp.Kd[i] * jointImp.Md[i];
        jointImp.Bd[i] = 2.0f * config.dampingRatio * sqrtf(fabsf(jkm));
    }
}


void ImpedanceController::Reset()
{
    for (int i = 0; i < DOF; i++)
    {
        state.complianceDisp[i] = 0;
        state.complianceVel[i] = 0;
        state.estimatedForce[i] = 0;
    }
    state.virtualEnergy = 0;
}


void ImpedanceController::AdmittanceTick(const float _forceExt[DOF],
                                          const float _posRef[DOF],
                                          float _posCmd[DOF])
{
    float dt = config.dt;

    for (int i = 0; i < DOF; i++)
    {
        state.estimatedForce[i] = _forceExt[i];

        // Apply force deadzone
        float F = _forceExt[i];
        if (fabsf(F) < config.forceDeadzone) F = 0;

        // Admittance dynamics: M*ddx_c + B*dx_c + K*x_c = F
        // Discrete: ddx_c = (F - B*dx_c - K*x_c) / M
        float M = cartImp.Md[i];
        float B = cartImp.Bd[i];
        float K = (config.mode == MODE_VARIABLE) ? Kd_current[i] : cartImp.Kd[i];

        if (M < 0.01f) M = 0.01f; // Prevent division by zero

        float acc = (F - B * state.complianceVel[i] - K * state.complianceDisp[i]) / M;

        // Semi-implicit Euler integration (more stable than explicit)
        state.complianceVel[i] += acc * dt;
        state.complianceDisp[i] += state.complianceVel[i] * dt;

        // Clamp displacement for safety
        if (state.complianceDisp[i] > config.maxComplianceDisp)
        {
            state.complianceDisp[i] = config.maxComplianceDisp;
            if (state.complianceVel[i] > 0) state.complianceVel[i] = 0;
        }
        else if (state.complianceDisp[i] < -config.maxComplianceDisp)
        {
            state.complianceDisp[i] = -config.maxComplianceDisp;
            if (state.complianceVel[i] < 0) state.complianceVel[i] = 0;
        }

        // Output: reference + compliance
        _posCmd[i] = _posRef[i] + state.complianceDisp[i];
    }

    // Passivity enforcement
    if (config.passivityEnforced)
        EnforcePassivity(_forceExt, state.complianceVel);
}


void ImpedanceController::ImpedanceTick(const float _posRef[DOF],
                                          const float _posCurrent[DOF],
                                          const float _velCurrent[DOF],
                                          const float _accCurrent[DOF],
                                          const float _gravTorque[DOF],
                                          float _tauCmd[DOF])
{
    for (int i = 0; i < DOF; i++)
    {
        float posErr = _posRef[i] - _posCurrent[i];
        float velErr = 0.0f - _velCurrent[i]; // Reference velocity = 0 for static target
        float accErr = 0.0f - _accCurrent[i];

        float K = jointImp.Kd[i];
        float B = jointImp.Bd[i];
        float M = jointImp.Md[i];

        // tau = K*(x_d - x) + B*(dx_d - dx) + M*(ddx_d - ddx) + G(q)
        _tauCmd[i] = K * posErr + B * velErr + M * accErr + _gravTorque[i];
    }
}


void ImpedanceController::VariableImpedanceTick(const float _forceExt[DOF],
                                                  const float _posRef[DOF],
                                                  float _posCmd[DOF])
{
    // Update variable stiffness: K(F) = K_min + (K_max - K_min) * exp(-alpha * |F|)
    for (int i = 0; i < DOF; i++)
    {
        float absF = fabsf(_forceExt[i]);
        float decay = expf(-alpha_var * absF);
        Kd_current[i] = Kd_min[i] + (Kd_max[i] - Kd_min[i]) * decay;

        // Update damping accordingly: B = 2*zeta*sqrt(K*M)
        float km = Kd_current[i] * cartImp.Md[i];
        cartImp.Bd[i] = 2.0f * config.dampingRatio * sqrtf(fabsf(km));
    }

    // Run admittance with updated stiffness
    AdmittanceTick(_forceExt, _posRef, _posCmd);
}


void ImpedanceController::EnforcePassivity(const float _forceExt[DOF],
                                            const float _vel[DOF])
{
    // Tank-based passivity (Ferraguti et al., pedestrians / pedestrians / pedestrians / pedestrians / pedestrians / pedestrians / Automatica 2013)
    // Energy tank: dT/dt = F_ext^T * dx_c - beta * T
    // If T < 0 (energy deficit), scale down compliance velocity

    float power = 0;
    for (int i = 0; i < DOF; i++)
        power += _forceExt[i] * _vel[i];

    // Update virtual energy (tank level)
    float beta = 0.1f; // Energy dissipation rate
    state.virtualEnergy += (power - beta * state.virtualEnergy) * config.dt;

    // Clamp energy tank
    float maxEnergy = 10.0f; // Maximum stored energy (Joules)
    if (state.virtualEnergy > maxEnergy) state.virtualEnergy = maxEnergy;

    // If energy becomes negative, attenuate compliance to ensure passivity
    if (state.virtualEnergy < 0)
    {
        float attenuation = 0.5f; // Reduce compliance velocity by 50%
        for (int i = 0; i < DOF; i++)
        {
            state.complianceVel[i] *= attenuation;
        }
        state.virtualEnergy = 0;
    }
}


void ImpedanceController::GetEffectiveStiffness(float _K[DOF]) const
{
    if (config.mode == MODE_VARIABLE)
        memcpy(_K, Kd_current, DOF * sizeof(float));
    else
        memcpy(_K, cartImp.Kd, DOF * sizeof(float));
}
