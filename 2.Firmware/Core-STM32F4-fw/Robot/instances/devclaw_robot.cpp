#include "communication.hpp"
#include "devclaw_robot.h"

inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (abs(_joints.a[i]) > max)
        {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


DevClawRobot::DevClawRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, true, 50, -170, 170);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 30, -73, 90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true, 30, 35, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, false, 24, -180, 180);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 30, -120, 120);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true, 50, -720, 720);
    hand = new DummyHand(_hcan, 7);

    dof6Solver = new DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);

    // Phase 1 Optimizations: Dynamics + Trajectory Planning
    dynamicsSolver = new DOF6Dynamics(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);
    cartPlanner = new CartesianPlanner();
    cartPlanner->AttachKinematicSolver(dof6Solver);
    multiAxisPlanner = new MultiAxisSCurvePlanner();

    // Phase 2: Robust Control & Identification
    multiDOB = new MultiJointDOB();
    float nominalInertias[6] = {0.05f, 0.08f, 0.06f, 0.02f, 0.015f, 0.01f};
    multiDOB->Init(50.0f, 1000.0f, nominalInertias); // 50Hz cutoff, 1kHz control
    autoTuner = new AutoTuner();
    paramIdent = new ParamIdentifier();

    // Phase 3: Safe Interaction & Learning
    impedanceCtrl = new ImpedanceController();
    impedanceCtrl->Init(0.001f, ImpedanceController::MODE_ADMITTANCE);
    collisionDetector = new CollisionDetector();
    collisionDetector->InitDefault(0.001f);
    dmpLearner = new MultiDOF_DMP();
    dmpLearner->Init(0.001f, 25);

    // Phase 4: Model-Based Control & Force Sensing
    forceEstimator = new ForceEstimator();
    forceEstimator->InitDefault(0.001f);
    ctcController = new ComputedTorqueController();
    ctcController->Init(0.001f, 50.0f); // 50 rad/s natural frequency
    frictionComp = new FrictionCompensator();
    frictionComp->InitDefault(0.001f);

    // Phase 5: Teaching, Assembly & Trajectory Optimization
    teachMode = new TeachMode();
    teachMode->InitDefault(0.001f);
    hybridCtrl = new HybridForcePositionController();
    hybridCtrl->InitDefault(0.001f);
    trajOptimizer = new TrajectoryOptimizer();

    // Phase 6: Safety, State Machine & Telemetry
    safetyMonitor = new SafetyMonitor();
    safetyMonitor->InitDefault(0.001f);
    stateMachine = new RobotStateMachine();
    stateMachine->Init();
    stateMachine->SetDynamicsAvailable(true);
    telemetry = new TelemetryStreamer();
    telemetry->Init(TelemetryStreamer::CH_JOINT_POS | TelemetryStreamer::CH_JOINT_VEL
                    | TelemetryStreamer::CH_EXT_FORCE | TelemetryStreamer::CH_SAFETY_STATUS, 100);

    // Phase 7: Calibration & Workspace Analysis
    workspaceAnalyzer = new WorkspaceAnalyzer();
    workspaceAnalyzer->InitDefault();
    gravCalibrator = new GravityCalibrator();
    float defaultKt[6] = {0.4f, 0.4f, 0.4f, 0.4f, 0.4f, 0.4f};
    float defaultGear[6] = {50, 30, 30, 24, 30, 50};
    gravCalibrator->Init(defaultKt, defaultGear);
    kinCalibrator = new KinematicCalibrator();
}


DevClawRobot::~DevClawRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
    delete dynamicsSolver;
    delete cartPlanner;
    delete multiAxisPlanner;
    delete multiDOB;
    delete autoTuner;
    delete paramIdent;
    delete impedanceCtrl;
    delete collisionDetector;
    delete dmpLearner;
    delete forceEstimator;
    delete ctcController;
    delete frictionComp;
    delete teachMode;
    delete hybridCtrl;
    delete trajOptimizer;
    delete safetyMonitor;
    delete stateMachine;
    delete telemetry;
    delete workspaceAnalyzer;
    delete gravCalibrator;
    delete kinCalibrator;
}


void DevClawRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
}


void DevClawRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
}


void DevClawRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
}


bool DevClawRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);
        float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        for (int j = 1; j <= 6; j++)
        {
            dynamicJointSpeeds.a[j - 1] =
                abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f); //0~10r/s
        }

        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}


bool DevClawRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

void DevClawRobot::UpdateJointAngles()
{
    motorJ[ALL]->UpdateAngle();
}


void DevClawRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}


void DevClawRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)_speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}


void DevClawRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)_acc = 0;
    else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]);
}


void DevClawRobot::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting();
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}


void DevClawRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DevClawRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DevClawRobot::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}


void DevClawRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


bool DevClawRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}


bool DevClawRobot::IsEnabled()
{
    return isEnabled;
}


void DevClawRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_MOTOR_TUNING)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3;
            break;
        case COMMAND_MOTOR_TUNING:
            break;
    }
}


DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint8_t
_id) :
    nodeID(_id), hcan(_hcan)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}


void DummyHand::SetAngle(float _angle)
{
    if (_angle > 30)_angle = 30;
    if (_angle < 0)_angle = 0;

    uint8_t mode = 0x02;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_angle;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetMaxCurrent(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    uint8_t mode = 0x01;
    txHeader.StdId = 7 << 7 | mode;

    // Float to Bytes
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetEnable(bool _enable)
{
    if (_enable)
        SetMaxCurrent(maxCurrent);
    else
        SetMaxCurrent(0);
}


uint32_t DevClawRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}


void DevClawRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    context->MoveJoints(context->targetJoints);
    context->isEnabled = false;
    ClearFifo();
}


std::string DevClawRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);

    return std::string{strBuffer};
}


uint32_t DevClawRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


uint32_t DevClawRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_CONTINUES_TRAJECTORY:
            if (_cmd[0] == '>')
            {
                float joints[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                context->MoveJoints(context->targetJoints);

                while (context->IsMoving() && context->IsEnabled())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                context->MoveJoints(context->targetJoints);

                while (context->IsMoving())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }

            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>')
            {
                float joints[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;
    }

    return osMessageQueueGetSpace(commandFifo);
}


void DevClawRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}


void DevClawRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DevClawRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


void DevClawRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}


// ==================== Phase 1: Advanced Motion Implementations ====================

bool DevClawRobot::MoveJ_SCurve(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,
                               float _velScale, float _accScale)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);

    // Validate joint limits
    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            return false;
    }

    // Clamp scale factors
    if (_velScale < 1) _velScale = 1;
    if (_velScale > 100) _velScale = 100;
    if (_accScale < 1) _accScale = 1;
    if (_accScale > 100) _accScale = 100;

    float velFactor = _velScale / 100.0f;
    float accFactor = _accScale / 100.0f;

    // Set up per-axis constraints
    float pStart[6], pEnd[6];
    MultiAxisSCurvePlanner::AxisConstraints_t axisConstraints[6];

    for (int i = 0; i < 6; i++)
    {
        pStart[i] = currentJoints.a[i];
        pEnd[i] = targetJointsTmp.a[i];
        axisConstraints[i].v_max = JOINT_VMAX[i] * velFactor;
        axisConstraints[i].a_max = JOINT_AMAX[i] * accFactor;
        axisConstraints[i].j_max = JOINT_JMAX[i] * accFactor;
    }

    // Plan synchronized multi-axis S-Curve trajectory
    bool ok = multiAxisPlanner->PlanSynchronized(6, pStart, pEnd, axisConstraints);
    if (!ok) return false;

    // Activate S-Curve trajectory execution
    targetJoints = targetJointsTmp;
    jointsStateFlag = 0;
    scurveActive = true;
    scurveElapsedTime = 0;

    return true;
}


bool DevClawRobot::MoveL_Cartesian(float _x, float _y, float _z, float _a, float _b, float _c,
                                  float _linearSpeed)
{
    if (!cartPlanner) return false;

    DOF6Kinematic::Pose6D_t targetPose(_x, _y, _z, _a, _b, _c);

    bool ok = cartPlanner->PlanLinearPath(currentJoints, targetPose,
                                           _linearSpeed, CARTESIAN_ACC, CARTESIAN_JERK, 50);
    if (!ok) return false;

    // Execute the Cartesian path using the trajectory tick mechanism
    scurveActive = true;
    scurveElapsedTime = 0;
    jointsStateFlag = 0;

    return true;
}


bool DevClawRobot::MoveC_Cartesian(float _vx, float _vy, float _vz, float _va, float _vb, float _vc,
                                  float _ex, float _ey, float _ez, float _ea, float _eb, float _ec,
                                  float _linearSpeed)
{
    if (!cartPlanner) return false;

    DOF6Kinematic::Pose6D_t viaPose(_vx, _vy, _vz, _va, _vb, _vc);
    DOF6Kinematic::Pose6D_t endPose(_ex, _ey, _ez, _ea, _eb, _ec);

    bool ok = cartPlanner->PlanCircularPath(currentJoints, viaPose, endPose,
                                             _linearSpeed, CARTESIAN_ACC, CARTESIAN_JERK, 80);
    if (!ok) return false;

    scurveActive = true;
    scurveElapsedTime = 0;
    jointsStateFlag = 0;

    return true;
}


void DevClawRobot::ApplyGravityCompensation()
{
    if (!dynamicsSolver || !dynamicsEnabled) return;

    float tau_g[6];
    float q[6];
    for (int i = 0; i < 6; i++)
        q[i] = currentJoints.a[i];

    dynamicsSolver->SolveGravityCompensation(q, tau_g);

    // Convert torques to motor current commands and apply as feedforward
    // tau = Kt * I * reduction_ratio  =>  I = tau / (Kt * reduction)
    // For stepper motors, we use the current set point as a torque proxy
    for (int j = 1; j <= 6; j++)
    {
        // Approximate: current(A) ≈ tau(Nm) / (torque_constant * reduction_ratio)
        // For NEMA17/42 steppers, Kt ≈ 0.3~0.5 Nm/A
        float Kt = 0.4f; // Approximate torque constant
        float currentFF = tau_g[j - 1] / (Kt * (float)motorJ[j]->reduction);
        // Clamp to safe range and add to existing control
        if (currentFF > 1.5f) currentFF = 1.5f;
        if (currentFF < -1.5f) currentFF = -1.5f;
        motorJ[j]->SetCurrentSetPoint(currentFF);
    }
}


void DevClawRobot::ApplyDynamicsFeedforward(const DOF6Kinematic::Joint6D_t &_targetJoints,
                                           const float _jointVel[6],
                                           const float _jointAcc[6])
{
    if (!dynamicsSolver || !dynamicsEnabled) return;

    float tau[6];
    float q[6], dq[6], ddq[6];
    for (int i = 0; i < 6; i++)
    {
        q[i] = _targetJoints.a[i];
        dq[i] = _jointVel[i];
        ddq[i] = _jointAcc[i];
    }

    dynamicsSolver->SolveInverseDynamics(q, dq, ddq, tau);

    // Apply feedforward torques via current commands
    for (int j = 1; j <= 6; j++)
    {
        float Kt = 0.4f;
        float currentFF = tau[j - 1] / (Kt * (float)motorJ[j]->reduction);
        if (currentFF > 2.0f) currentFF = 2.0f;
        if (currentFF < -2.0f) currentFF = -2.0f;
        motorJ[j]->SetCurrentSetPoint(currentFF);
    }
}


float DevClawRobot::GetManipulability()
{
    if (!dynamicsSolver) return 0;

    float q[6];
    for (int i = 0; i < 6; i++)
        q[i] = currentJoints.a[i];

    return dynamicsSolver->ComputeManipulability(q);
}


void DevClawRobot::SetDynamicsEnabled(bool _enable)
{
    dynamicsEnabled = _enable;
}


void DevClawRobot::SCurveTrajectoryTick(uint32_t _timeMillis)
{
    if (!scurveActive) return;

    scurveElapsedTime += (float)_timeMillis / 1000.0f;

    // Check if Cartesian path is active
    if (cartPlanner && cartPlanner->IsValid() && cartPlanner->GetTotalTime() > 0)
    {
        DOF6Kinematic::Joint6D_t joints{};
        SCurvePlanner::State_t cartState{};

        if (cartPlanner->Evaluate(scurveElapsedTime, joints, cartState))
        {
            // Apply dynamics feedforward if enabled
            if (dynamicsEnabled && dynamicsSolver)
            {
                // Estimate velocity and acceleration from cartesian state
                // (simplified: use finite difference from waypoint data)
                float vel[6] = {0}, acc[6] = {0};
                ApplyDynamicsFeedforward(joints, vel, acc);
            }

            // Send joint commands
            for (int j = 1; j <= 6; j++)
            {
                motorJ[j]->SetAngleWithVelocityLimit(
                    joints.a[j - 1] - initPose.a[j - 1],
                    JOINT_VMAX[j - 1] * (float)motorJ[j]->reduction / 360.0f);
            }
            targetJoints = joints;
        }

        if (scurveElapsedTime >= cartPlanner->GetTotalTime())
        {
            scurveActive = false;
            jointsStateFlag = 0b1111110;
        }
    }
    // Multi-axis S-Curve (MoveJ_SCurve)
    else if (multiAxisPlanner && multiAxisPlanner->IsValid() && multiAxisPlanner->GetTotalTime() > 0)
    {
        SCurvePlanner::State_t states[6];
        multiAxisPlanner->Evaluate(scurveElapsedTime, states);

        DOF6Kinematic::Joint6D_t cmdJoints{};
        float vel[6], acc[6];
        for (int i = 0; i < 6; i++)
        {
            cmdJoints.a[i] = states[i].position;
            vel[i] = states[i].velocity;
            acc[i] = states[i].acceleration;
        }

        // Apply dynamics feedforward if enabled
        if (dynamicsEnabled && dynamicsSolver)
        {
            ApplyDynamicsFeedforward(cmdJoints, vel, acc);
        }

        // Send joint position commands with velocity limit
        for (int j = 1; j <= 6; j++)
        {
            float velLimit = fabsf(vel[j - 1]) * (float)motorJ[j]->reduction / 360.0f * 1.2f;
            if (velLimit < 0.1f) velLimit = 0.1f;
            motorJ[j]->SetAngleWithVelocityLimit(
                cmdJoints.a[j - 1] - initPose.a[j - 1], velLimit);
        }
        targetJoints = cmdJoints;

        if (scurveElapsedTime >= multiAxisPlanner->GetTotalTime())
        {
            scurveActive = false;
            jointsStateFlag = 0b1111110;
        }
    }
    else
    {
        scurveActive = false;
    }
}


// ==================== Phase 2: Robust Control & Identification ====================

void DevClawRobot::SetDOBEnabled(bool _enable)
{
    dobEnabled = _enable;
    if (multiDOB)
        multiDOB->SetEnabled(_enable);
}


void DevClawRobot::SetDOBCutoff(float _freqHz)
{
    if (!multiDOB) return;
    // Re-initialize with new cutoff
    float nominalInertias[6] = {0.05f, 0.08f, 0.06f, 0.02f, 0.015f, 0.01f};
    multiDOB->Init(_freqHz, 1000.0f, nominalInertias);
    multiDOB->SetEnabled(dobEnabled);
}


void DevClawRobot::StartAutoTune(uint32_t _jointIdx, float _amplitude)
{
    if (!autoTuner || _jointIdx < 1 || _jointIdx > 6) return;

    AutoTuner::RelayConfig_t cfg;
    cfg.relayAmplitude = _amplitude;
    cfg.hysteresis = 5.0f;    // 5 encoder counts hysteresis
    cfg.maxCycles = 5;
    cfg.samplePeriod = 0.001f; // 1ms (1kHz control loop)

    autoTuner->StartRelayTuning(cfg);
    autoTuneJoint = _jointIdx;
}


bool DevClawRobot::CheckAutoTuneComplete()
{
    if (!autoTuner || autoTuneJoint == 0) return false;

    if (autoTuner->IsRelayTuningDone())
    {
        const AutoTuner::RelayResult_t &result = autoTuner->GetRelayResult();
        if (result.valid)
        {
            // Use Tyreus-Luyben (conservative) gains
            AutoTuner::PIDGains_t gains = autoTuner->ComputeTyreusLuybenGains();

            // Apply to the motor's DCE controller via CAN
            // Scale gains to match the motor driver's integer format
            float kpScaled = gains.kp * 10.0f; // DCE kp scaling
            float kvScaled = gains.kv * 10.0f;
            float kiScaled = gains.ki * 10.0f;
            float kdScaled = gains.kd * 10.0f;

            motorJ[autoTuneJoint]->SetDceKp(kpScaled);
            motorJ[autoTuneJoint]->SetDceKv(kvScaled);
            motorJ[autoTuneJoint]->SetDceKi(kiScaled);
            motorJ[autoTuneJoint]->SetDceKd(kdScaled);
        }
        autoTuneJoint = 0;
        return true;
    }
    return false;
}


void DevClawRobot::StartIdentification()
{
    if (!paramIdent) return;

    paramIdent->ResetSamples();
    identRunning = true;
}


bool DevClawRobot::RunIdentification()
{
    if (!paramIdent || !dynamicsSolver) return false;

    identRunning = false;

    ParamIdentifier::IdentResult_t result = paramIdent->Identify();
    if (!result.valid) return false;

    // Update dynamics model with identified parameters
    for (int j = 0; j < 6; j++)
    {
        DOF6Dynamics::LinkParam_t linkParam;
        // Preserve existing defaults, update identified values
        linkParam.mass = fabsf(result.gravityParam[j]) / 0.1f; // Approximate mass from m*lc
        linkParam.com[0] = 0; linkParam.com[1] = 0;
        linkParam.com[2] = (result.gravityParam[j] > 0) ? 0.05f : -0.05f;
        // Set diagonal inertia from reflected inertia
        float I = fabsf(result.reflectedInertia[j]);
        float inertia[9] = {I, 0, 0,  0, I, 0,  0, 0, I * 0.5f};
        memcpy(linkParam.inertia, inertia, 9 * sizeof(float));
        linkParam.friction_v = result.viscousFriction[j];
        linkParam.friction_c = result.coulombFriction[j];

        dynamicsSolver->SetSingleLinkParam(j, linkParam);
    }

    // Also update DOB nominal inertias
    if (multiDOB)
    {
        float inertias[6];
        for (int j = 0; j < 6; j++)
            inertias[j] = fabsf(result.reflectedInertia[j]);
        multiDOB->Init(50.0f, 1000.0f, inertias);
        multiDOB->SetEnabled(dobEnabled);
    }

    return true;
}


void DevClawRobot::RobustControlTick(uint32_t _timeMillis)
{
    float DEG2RAD = 0.01745329251994f;

    // 1. Collect data for identification if running
    if (identRunning && paramIdent)
    {
        float q[6], dq[6], ddq[6], tau[6];
        for (int i = 0; i < 6; i++)
        {
            q[i] = currentJoints.a[i] * DEG2RAD;
            // Approximate velocities and accelerations from joint state
            // (In production, these come from KF on the motor driver side)
            dq[i] = 0; ddq[i] = 0; tau[i] = 0;
        }
        paramIdent->RecordSample(q, dq, ddq, tau);
    }

    // 2. Auto-tune tick
    if (autoTuner && autoTuneJoint > 0 && !autoTuner->IsRelayTuningDone())
    {
        float pos = currentJoints.a[autoTuneJoint - 1];
        float setpoint = targetJoints.a[autoTuneJoint - 1];
        float output = autoTuner->RelayTuningTick(pos, setpoint);

        // Apply relay output as current command to the motor
        motorJ[autoTuneJoint]->SetCurrentSetPoint(output);
    }
    else if (autoTuneJoint > 0)
    {
        CheckAutoTuneComplete();
    }

    // 3. DOB compensation
    if (dobEnabled && multiDOB && dynamicsSolver)
    {
        float tauCmd[6] = {0};
        float velocity[6] = {0};
        float acceleration[6] = {0};
        float tauComp[6] = {0};

        // Read KF-estimated state from motor drivers (via CAN feedback)
        for (int i = 0; i < 6; i++)
        {
            velocity[i] = motorJ[i + 1]->estVelocity * DEG2RAD;
            acceleration[i] = motorJ[i + 1]->estAcceleration * DEG2RAD;
        }

        multiDOB->Update(tauCmd, velocity, acceleration, tauComp);

        // Apply DOB compensation torques
        for (int j = 1; j <= 6; j++)
        {
            float Kt = 0.4f;
            float compCurrent = tauComp[j - 1] / (Kt * (float)motorJ[j]->reduction);
            if (compCurrent > 1.0f) compCurrent = 1.0f;
            if (compCurrent < -1.0f) compCurrent = -1.0f;
            // Add to existing current command (feedforward)
            motorJ[j]->SetCurrentSetPoint(compCurrent);
        }
    }
}


// ==================== Phase 3: Safe Interaction & Learning ====================

void DevClawRobot::SetImpedanceMode(uint32_t _mode)
{
    if (!impedanceCtrl) return;

    switch (_mode)
    {
    case 0:
        impedanceActive = false;
        impedanceCtrl->Reset();
        break;
    case 1:
        impedanceActive = true;
        impedanceCtrl->Init(0.001f, ImpedanceController::MODE_ADMITTANCE);
        break;
    case 2:
        impedanceActive = true;
        impedanceCtrl->Init(0.001f, ImpedanceController::MODE_IMPEDANCE);
        break;
    case 3:
        impedanceActive = true;
        impedanceCtrl->Init(0.001f, ImpedanceController::MODE_VARIABLE);
        break;
    default:
        impedanceActive = false;
        break;
    }
}


void DevClawRobot::SetStiffness(float _stiffness)
{
    if (!impedanceCtrl) return;
    if (_stiffness < 0) _stiffness = 0;
    if (_stiffness > 5000) _stiffness = 5000;
    impedanceCtrl->SetUniformStiffness(_stiffness);
}


void DevClawRobot::SetCollisionReaction(uint32_t _strategy)
{
    if (!collisionDetector) return;

    collisionDetEnabled = true;
    CollisionDetector::ReactionStrategy_t strategy;
    switch (_strategy)
    {
    case 0: strategy = CollisionDetector::REACTION_STOP; break;
    case 1: strategy = CollisionDetector::REACTION_RETRACT; break;
    case 2: strategy = CollisionDetector::REACTION_FLOAT; break;
    case 3: strategy = CollisionDetector::REACTION_REFLEX; break;
    case 4: strategy = CollisionDetector::REACTION_COMPLY; break;
    default: strategy = CollisionDetector::REACTION_STOP; break;
    }
    collisionDetector->SetReactionStrategy(strategy);
}


void DevClawRobot::ResetCollision()
{
    if (!collisionDetector) return;
    collisionDetector->Reset();
    collisionDetEnabled = true;
}


void DevClawRobot::StartDMPRecord()
{
    if (!dmpLearner) return;
    dmpRecording = true;
    dmpExecuting = false;
    // Recording happens in SafeInteractionTick via RecordPoint
}


bool DevClawRobot::StopDMPRecord(float _duration)
{
    if (!dmpLearner || !dmpRecording) return false;
    dmpRecording = false;

    if (_duration <= 0)
        _duration = (float)dmpLearner->GetRecordCount() * 0.001f; // Assume 1kHz

    return dmpLearner->FinishRecording(_duration);
}


bool DevClawRobot::ExecuteDMP(float _j1, float _j2, float _j3,
                              float _j4, float _j5, float _j6)
{
    if (!dmpLearner) return false;

    float start[6], goal[6];
    for (int i = 0; i < 6; i++)
        start[i] = currentJoints.a[i];

    goal[0] = _j1; goal[1] = _j2; goal[2] = _j3;
    goal[3] = _j4; goal[4] = _j5; goal[5] = _j6;

    // Validate goal joint limits
    for (int j = 0; j < 6; j++)
    {
        if (goal[j] > motorJ[j + 1]->angleLimitMax ||
            goal[j] < motorJ[j + 1]->angleLimitMin)
            return false;
    }

    dmpLearner->Reset(start, goal);
    dmpExecuting = true;
    dmpRecording = false;

    return true;
}


void DevClawRobot::SafeInteractionTick(uint32_t _timeMillis)
{
    float DEG2RAD = 0.01745329251994f;

    // 1. Record DMP data if in teach mode
    if (dmpRecording && dmpLearner)
    {
        float joints[6];
        for (int i = 0; i < 6; i++)
            joints[i] = currentJoints.a[i];
        dmpLearner->RecordPoint(joints);
    }

    // 2. Collision detection
    if (collisionDetEnabled && collisionDetector && dynamicsSolver)
    {
        float q[6], dq[6], tauCmd[6], gravTau[6], corTau[6], massDiag[6];

        for (int i = 0; i < 6; i++)
        {
            q[i] = currentJoints.a[i];
            dq[i] = motorJ[i + 1]->estVelocity * DEG2RAD;
            tauCmd[i] = motorJ[i + 1]->motorCurrent * 0.4f * (float)motorJ[i + 1]->reduction;
        }

        dynamicsSolver->SolveGravityCompensation(q, gravTau);
        float zeroDq[6] = {0};
        dynamicsSolver->SolveCoriolisTorques(q, zeroDq, corTau);

        float M[36];
        dynamicsSolver->SolveMassMatrix(q, M);
        for (int i = 0; i < 6; i++)
            massDiag[i] = M[i * 6 + i];

        CollisionDetector::CollisionInfo_t colInfo =
            collisionDetector->Update(tauCmd, q, dq, gravTau, corTau, massDiag);

        if (colInfo.detected)
        {
            // Collision detected! Execute reaction
            float reactionJoints[6];
            if (collisionDetector->GetReactionCommand(q, reactionJoints))
            {
                // If reaction is COMPLY, switch to admittance mode
                if (colInfo.state == CollisionDetector::STATE_REACTING)
                {
                    // Apply reaction joint targets
                    MoveJ(reactionJoints[0], reactionJoints[1], reactionJoints[2],
                          reactionJoints[3], reactionJoints[4], reactionJoints[5]);
                }
            }
        }
    }

    // 3. Impedance/admittance control
    if (impedanceActive && impedanceCtrl)
    {
        // Get estimated external force from DOB
        float forceExt[6] = {0};
        if (dobEnabled && multiDOB)
        {
            multiDOB->GetDisturbances(forceExt);
            // Negate: DOB disturbance is what opposes motion = external force
            for (int i = 0; i < 6; i++)
                forceExt[i] = -forceExt[i];
        }

        float posRef[6], posCmd[6];
        for (int i = 0; i < 6; i++)
            posRef[i] = targetJoints.a[i];

        impedanceCtrl->AdmittanceTick(forceExt, posRef, posCmd);

        // Apply compliant position as actual command
        for (int j = 1; j <= 6; j++)
        {
            float clamped = posCmd[j - 1];
            if (clamped > motorJ[j]->angleLimitMax) clamped = motorJ[j]->angleLimitMax;
            if (clamped < motorJ[j]->angleLimitMin) clamped = motorJ[j]->angleLimitMin;
            motorJ[j]->SetAngleWithVelocityLimit(clamped, 30.0f);
        }
    }
}


void DevClawRobot::DMPTick()
{
    if (!dmpExecuting || !dmpLearner) return;

    MultiDOF_DMP::MultiState_t dmpState = dmpLearner->Step();

    // Send DMP positions to motors
    for (int j = 1; j <= 6; j++)
    {
        float angle = dmpState.y[j - 1];
        if (angle > motorJ[j]->angleLimitMax) angle = motorJ[j]->angleLimitMax;
        if (angle < motorJ[j]->angleLimitMin) angle = motorJ[j]->angleLimitMin;

        float velLimit = fabsf(dmpState.dy[j - 1]) * (float)motorJ[j]->reduction * 0.15f;
        if (velLimit < 1.0f) velLimit = 1.0f;
        if (velLimit > 30.0f) velLimit = 30.0f;

        motorJ[j]->SetAngleWithVelocityLimit(angle, velLimit);
    }

    // Update current joint targets
    for (int i = 0; i < 6; i++)
        targetJoints.a[i] = dmpState.y[i];

    if (dmpLearner->IsComplete())
    {
        dmpExecuting = false;
        jointsStateFlag = 0b1111110;
    }
}


// ==================== Phase 4: Model-Based Control & Force Sensing ====================

void DevClawRobot::SetCTCEnabled(bool _enable)
{
    ctcEnabled = _enable;
    if (ctcController && _enable)
        ctcController->Reset();
}


void DevClawRobot::SetCTCFrequency(float _freqHz)
{
    if (!ctcController) return;
    float wn = 2.0f * 3.14159265f * _freqHz;
    if (wn < 1.0f) wn = 1.0f;
    if (wn > 500.0f) wn = 500.0f;
    ctcController->SetNaturalFrequency(wn);
}


void DevClawRobot::SetFrictionCompEnabled(bool _enable)
{
    frictionCompEnabled = _enable;
    if (frictionComp && _enable)
        frictionComp->Reset();
}


void DevClawRobot::CalibrateForceEstimator()
{
    if (!forceEstimator || !dynamicsSolver) return;

    float DEG2RAD = 0.01745329251994f;
    float q[6], current[6], tauRNEA[6];

    for (int i = 0; i < 6; i++)
    {
        q[i] = currentJoints.a[i] * DEG2RAD;
        current[i] = motorJ[i + 1]->motorCurrent;
    }

    // Get model torque at current static pose (zero velocity/acceleration)
    float zeroDq[6] = {0}, zeroDdq[6] = {0};
    dynamicsSolver->SolveInverseDynamics(q, zeroDq, zeroDdq, tauRNEA);

    forceEstimator->CalibrateDeadzone(current, tauRNEA);
}


void DevClawRobot::RequestMotorFeedback()
{
    for (int j = 1; j <= 6; j++)
        motorJ[j]->RequestKFState();
}


float DevClawRobot::GetExternalForce()
{
    if (!forceEstimator) return 0;
    return forceEstimator->GetCartesianForce().magnitude;
}


void DevClawRobot::ModelBasedControlTick(uint32_t _timeMillis)
{
    if (!dynamicsSolver) return;

    float DEG2RAD = 0.01745329251994f;
    float q[6], dq[6], ddq[6], current[6];

    // Read state from motor KF feedback
    for (int i = 0; i < 6; i++)
    {
        q[i] = currentJoints.a[i] * DEG2RAD;
        dq[i] = motorJ[i + 1]->estVelocity * DEG2RAD;
        ddq[i] = motorJ[i + 1]->estAcceleration * DEG2RAD;
        current[i] = motorJ[i + 1]->motorCurrent;
    }

    // 1. RNEA inverse dynamics: compute model torque
    float tauRNEA[6];
    dynamicsSolver->SolveInverseDynamics(q, dq, ddq, tauRNEA);

    // 2. Force estimation: compare motor torque vs model
    if (forceEstimator)
    {
        forceEstimator->Update(current, q, dq, ddq, tauRNEA);

        // Map to Cartesian if Jacobian is available
        float J[36];
        dynamicsSolver->SolveJacobian(q, J);
        forceEstimator->MapToCartesian(J);

        // Feed force estimate to impedance controller
        if (impedanceActive && impedanceCtrl)
        {
            const ForceEstimator::CartesianForce_t &F = forceEstimator->GetCartesianForce();
            float forceExt[6] = {F.Fx, F.Fy, F.Fz, F.Tx, F.Ty, F.Tz};
            float posRef[6], posCmd[6];
            for (int i = 0; i < 6; i++)
                posRef[i] = targetJoints.a[i];

            impedanceCtrl->AdmittanceTick(forceExt, posRef, posCmd);

            for (int j = 1; j <= 6; j++)
            {
                float clamped = posCmd[j - 1];
                if (clamped > motorJ[j]->angleLimitMax) clamped = motorJ[j]->angleLimitMax;
                if (clamped < motorJ[j]->angleLimitMin) clamped = motorJ[j]->angleLimitMin;
                motorJ[j]->SetAngleWithVelocityLimit(clamped, 30.0f);
            }
        }
    }

    // 3. Friction compensation
    float tauFrictionComp[6] = {0};
    if (frictionCompEnabled && frictionComp)
    {
        frictionComp->Compensate(dq, tauFrictionComp);
    }

    // 4. Computed torque control (if enabled)
    if (ctcEnabled && ctcController)
    {
        ComputedTorqueController::Reference_t ref;
        for (int i = 0; i < 6; i++)
        {
            ref.q_d[i] = targetJoints.a[i] * DEG2RAD;
            ref.dq_d[i] = 0; // Static target
            ref.ddq_d[i] = 0;
        }

        // Nonlinear terms: h = C*dq + g
        float gravTau[6], corTau[6], hTorque[6];
        dynamicsSolver->SolveGravityCompensation(q, gravTau);
        dynamicsSolver->SolveCoriolisTorques(q, dq, corTau);
        for (int i = 0; i < 6; i++)
            hTorque[i] = gravTau[i] + corTau[i];

        // Mass matrix
        float M[36];
        dynamicsSolver->SolveMassMatrix(q, M);

        // DOB compensation (if available)
        float tauDOB[6] = {0};
        if (dobEnabled && multiDOB)
        {
            float tauCompDOB[6];
            multiDOB->Update(tauRNEA, dq, ddq, tauCompDOB);
            for (int i = 0; i < 6; i++)
                tauDOB[i] = tauCompDOB[i];
        }

        float tauCTC[6];
        ctcController->Compute(ref, q, dq, M, hTorque, tauDOB, tauCTC);

        // Add friction compensation
        for (int i = 0; i < 6; i++)
            tauCTC[i] += tauFrictionComp[i];

        // Convert torque to motor current and send
        for (int j = 1; j <= 6; j++)
        {
            float Kt = 0.4f;
            float cmdCurrent = tauCTC[j - 1] / (Kt * (float)motorJ[j]->reduction);
            if (cmdCurrent > 2.0f) cmdCurrent = 2.0f;
            if (cmdCurrent < -2.0f) cmdCurrent = -2.0f;
            motorJ[j]->SetCurrentSetPoint(cmdCurrent);
        }
    }
}


// ==================== Phase 5: Teaching, Assembly & Trajectory Optimization ====================

void DevClawRobot::SetTeachMode(bool _enable)
{
    if (!teachMode) return;

    teachActive = _enable;
    if (_enable)
        teachMode->Start();
    else
        teachMode->Stop();
}


bool DevClawRobot::SaveTeachWaypoint()
{
    if (!teachMode || !teachActive) return false;

    float joints[6];
    for (int i = 0; i < 6; i++)
        joints[i] = currentJoints.a[i];

    return teachMode->SaveWaypoint(joints);
}


void DevClawRobot::SetTeachRecording(bool _record)
{
    if (!teachMode || !teachActive) return;

    if (_record)
        teachMode->StartRecording();
    else
    {
        int nPoints = teachMode->StopRecording();

        // Auto-encode into DMP if enough points recorded
        if (nPoints > 10 && dmpLearner)
        {
            const float (*traj)[6] = teachMode->GetTrajectory();
            float duration = (float)nPoints * 0.01f; // 100Hz recording
            dmpLearner->LearnFromDemo(traj, nPoints, duration);
        }
    }
}


bool DevClawRobot::MoveJ_MinJerk(float _j1, float _j2, float _j3,
                                 float _j4, float _j5, float _j6, float _duration)
{
    if (!trajOptimizer) return false;

    // Validate joint limits
    float goal[6] = {_j1, _j2, _j3, _j4, _j5, _j6};
    for (int j = 0; j < 6; j++)
    {
        if (goal[j] > motorJ[j + 1]->angleLimitMax ||
            goal[j] < motorJ[j + 1]->angleLimitMin)
            return false;
    }

    TrajectoryOptimizer::MinJerkConfig_t cfg;
    for (int i = 0; i < 6; i++)
    {
        cfg.q0[i] = currentJoints.a[i];
        cfg.qf[i] = goal[i];
        cfg.v0[i] = 0; cfg.vf[i] = 0;
        cfg.a0[i] = 0; cfg.af[i] = 0;
    }
    cfg.duration = (_duration > 0) ? _duration : 2.0f;

    trajOptimizer->PlanMinJerk(cfg);
    minJerkActive = true;
    minJerkTimer = 0;
    jointsStateFlag = 0;

    return true;
}


void DevClawRobot::SetHybridAxis(uint32_t _axis, bool _forceMode)
{
    if (!hybridCtrl || _axis > 5) return;

    float sel[6];
    hybridCtrl->GetState(); // Ensure initialized
    // Read current selection, modify one axis
    // For simplicity, maintain a local copy
    static float selMatrix[6] = {0, 0, 0, 0, 0, 0};
    selMatrix[_axis] = _forceMode ? 1.0f : 0.0f;
    hybridCtrl->SetSelectionMatrix(selMatrix);
    hybridActive = true;
}


void DevClawRobot::SetForceRef(float _fx, float _fy, float _fz)
{
    if (!hybridCtrl) return;
    float fRef[6] = {_fx, _fy, _fz, 0, 0, 0};
    hybridCtrl->SetForceReference(fRef);
}


void DevClawRobot::TeachAndForceTick(uint32_t _timeMillis)
{
    float DEG2RAD = 0.01745329251994f;

    // 1. Teach mode tick
    if (teachActive && teachMode && dynamicsSolver)
    {
        float q[6], dq[6], gravTau[6], fricComp[6];

        for (int i = 0; i < 6; i++)
        {
            q[i] = currentJoints.a[i];
            dq[i] = motorJ[i + 1]->estVelocity;
        }

        // Get gravity compensation from RNEA
        float qRad[6];
        for (int i = 0; i < 6; i++) qRad[i] = q[i] * DEG2RAD;
        dynamicsSolver->SolveGravityCompensation(qRad, gravTau);

        // Get friction compensation
        float dqRad[6];
        for (int i = 0; i < 6; i++) dqRad[i] = dq[i] * DEG2RAD;

        if (frictionCompEnabled && frictionComp)
            frictionComp->Compensate(dqRad, fricComp);
        else
            memset(fricComp, 0, sizeof(fricComp));

        float tauCmd[6];
        teachMode->Tick(q, dq, gravTau, fricComp, tauCmd);

        // Convert torque to motor current
        for (int j = 1; j <= 6; j++)
        {
            float Kt = 0.4f;
            float cmdCurrent = tauCmd[j - 1] / (Kt * (float)motorJ[j]->reduction);
            if (cmdCurrent > 2.0f) cmdCurrent = 2.0f;
            if (cmdCurrent < -2.0f) cmdCurrent = -2.0f;
            motorJ[j]->SetCurrentSetPoint(cmdCurrent);
        }
    }

    // 2. Minimum-jerk trajectory execution
    if (minJerkActive && trajOptimizer)
    {
        minJerkTimer += (float)_timeMillis * 0.001f;

        TrajectoryOptimizer::TrajectoryPoint_t pt =
            trajOptimizer->EvalMinJerk(minJerkTimer);

        for (int j = 1; j <= 6; j++)
        {
            float angle = pt.q[j - 1];
            if (angle > motorJ[j]->angleLimitMax) angle = motorJ[j]->angleLimitMax;
            if (angle < motorJ[j]->angleLimitMin) angle = motorJ[j]->angleLimitMin;

            float velLimit = fabsf(pt.dq[j - 1]) * (float)motorJ[j]->reduction * 0.15f;
            if (velLimit < 1.0f) velLimit = 1.0f;
            if (velLimit > 30.0f) velLimit = 30.0f;

            motorJ[j]->SetAngleWithVelocityLimit(angle, velLimit);
        }

        for (int i = 0; i < 6; i++)
            targetJoints.a[i] = pt.q[i];

        if (minJerkTimer >= trajOptimizer->GetDuration())
        {
            minJerkActive = false;
            jointsStateFlag = 0b1111110;
        }
    }

    // 3. Hybrid force/position control
    if (hybridActive && hybridCtrl && forceEstimator && dynamicsSolver)
    {
        float posCurrent[6], velCurrent[6], forceMeas[6];

        // Current Cartesian pose (from FK)
        posCurrent[0] = currentPose6D.X; posCurrent[1] = currentPose6D.Y;
        posCurrent[2] = currentPose6D.Z;
        posCurrent[3] = currentPose6D.A; posCurrent[4] = currentPose6D.B;
        posCurrent[5] = currentPose6D.C;

        // Velocity approximation (from motor KF)
        for (int i = 0; i < 6; i++)
            velCurrent[i] = motorJ[i + 1]->estVelocity * DEG2RAD;

        // Force estimate
        const ForceEstimator::CartesianForce_t &F = forceEstimator->GetCartesianForce();
        forceMeas[0] = F.Fx; forceMeas[1] = F.Fy; forceMeas[2] = F.Fz;
        forceMeas[3] = F.Tx; forceMeas[4] = F.Ty; forceMeas[5] = F.Tz;

        // Jacobian and gravity
        float qRad[6], J[36], gravTau[6];
        for (int i = 0; i < 6; i++) qRad[i] = currentJoints.a[i] * DEG2RAD;
        dynamicsSolver->SolveJacobian(qRad, J);
        dynamicsSolver->SolveGravityCompensation(qRad, gravTau);

        float tauOut[6];
        hybridCtrl->Compute(posCurrent, velCurrent, forceMeas, J, gravTau, tauOut);

        for (int j = 1; j <= 6; j++)
        {
            float Kt = 0.4f;
            float cmdCurrent = tauOut[j - 1] / (Kt * (float)motorJ[j]->reduction);
            if (cmdCurrent > 2.0f) cmdCurrent = 2.0f;
            if (cmdCurrent < -2.0f) cmdCurrent = -2.0f;
            motorJ[j]->SetCurrentSetPoint(cmdCurrent);
        }
    }
}


// ==================== Phase 6: Safety, State Machine & Telemetry ====================

uint32_t DevClawRobot::SafetyCheck()
{
    if (!safetyMonitor) return 0;

    float q[6], dq[6], current[6];
    for (int i = 0; i < 6; i++)
    {
        q[i] = currentJoints.a[i];
        dq[i] = motorJ[i + 1]->estVelocity;
        current[i] = motorJ[i + 1]->motorCurrent;
    }

    // TCP speed: approximate from joint velocities (simplified)
    float tcpSpeed = 0;
    for (int i = 0; i < 3; i++)
        tcpSpeed += dq[i] * dq[i];
    tcpSpeed = sqrtf(tcpSpeed) * 2.0f; // Rough mm/s estimate

    float extForce = 0;
    if (forceEstimator)
        extForce = forceEstimator->GetCartesianForce().magnitude;

    float loopTimeMs = 1.0f; // Nominal 1ms at 1kHz

    SafetyMonitor::SafetyLevel_t level =
        safetyMonitor->Check(q, dq, current, tcpSpeed, extForce, loopTimeMs);

    // React to safety events
    if (level >= SafetyMonitor::SAFETY_ESTOP && stateMachine)
        stateMachine->ProcessEvent(RobotStateMachine::EVT_ESTOP);
    else if (level >= SafetyMonitor::SAFETY_STOP && stateMachine)
        stateMachine->ProcessEvent(RobotStateMachine::EVT_STOP_REQUEST);

    return (uint32_t)level;
}


bool DevClawRobot::ResetEmergencyStop()
{
    bool ok = true;
    if (safetyMonitor) ok &= safetyMonitor->ResetEstop();
    if (stateMachine) ok &= stateMachine->ProcessEvent(RobotStateMachine::EVT_ESTOP_RESET);
    return ok;
}


void DevClawRobot::RequestMode(uint32_t _mode)
{
    if (!stateMachine) return;

    RobotStateMachine::TransitionEvent_t evt;
    switch (_mode)
    {
    case 0: evt = RobotStateMachine::EVT_STOP_REQUEST; break;
    case 1: evt = RobotStateMachine::EVT_REQUEST_POSITION; break;
    case 2: evt = RobotStateMachine::EVT_REQUEST_CTC; break;
    case 3: evt = RobotStateMachine::EVT_REQUEST_IMPEDANCE; break;
    case 4: evt = RobotStateMachine::EVT_REQUEST_TEACH; break;
    case 5: evt = RobotStateMachine::EVT_REQUEST_DMP; break;
    case 6: evt = RobotStateMachine::EVT_REQUEST_HYBRID; break;
    case 7: evt = RobotStateMachine::EVT_REQUEST_MINJERK; break;
    default: return;
    }
    stateMachine->ProcessEvent(evt);
}


void DevClawRobot::SetTCPSpeedLimit(float _speed)
{
    if (safetyMonitor)
        safetyMonitor->SetTCPSpeedLimit(_speed);
}


void DevClawRobot::ConfigTelemetry(uint32_t _channels, float _rateHz)
{
    if (!telemetry) return;
    telemetry->SetChannels((uint16_t)_channels);
    if (_rateHz > 0)
        telemetry->SetDecimation((uint16_t)(1000.0f / _rateHz)); // 1kHz / rate
}


uint32_t DevClawRobot::GetRobotState()
{
    if (!stateMachine) return 0;
    return (uint32_t)stateMachine->GetState();
}


void DevClawRobot::MasterControlTick(uint32_t _timeMillis)
{
    float DEG2RAD = 0.01745329251994f;

    // 1. Request motor feedback (KF state via CAN)
    RequestMotorFeedback();

    // 2. Update state machine
    if (stateMachine)
        stateMachine->Tick(_timeMillis);

    // 3. Safety check
    uint32_t safetyLevel = SafetyCheck();

    // 4. If E-stopped, disable all motors and return
    if (safetyLevel >= SafetyMonitor::SAFETY_ESTOP)
    {
        for (int j = 1; j <= 6; j++)
            motorJ[j]->SetCurrentSetPoint(0);
        return;
    }

    // 5. Apply speed scale from safety monitor
    float speedScale = safetyMonitor ? safetyMonitor->GetSpeedScale() : 1.0f;

    // 6. Dispatch to active control mode
    if (stateMachine)
    {
        switch (stateMachine->GetState())
        {
        case RobotStateMachine::STATE_POSITION:
            // Standard position control handled by motor driver DCE
            break;

        case RobotStateMachine::STATE_CTC_TORQUE:
            ModelBasedControlTick(_timeMillis);
            break;

        case RobotStateMachine::STATE_IMPEDANCE:
            SafeInteractionTick(_timeMillis);
            break;

        case RobotStateMachine::STATE_TEACH:
            TeachAndForceTick(_timeMillis);
            break;

        case RobotStateMachine::STATE_DMP_EXEC:
            DMPTick();
            break;

        case RobotStateMachine::STATE_HYBRID_FORCE:
            TeachAndForceTick(_timeMillis);
            break;

        case RobotStateMachine::STATE_MINJERK_EXEC:
            TeachAndForceTick(_timeMillis);
            break;

        case RobotStateMachine::STATE_STOPPING:
            // Decelerate: zero current commands
            for (int j = 1; j <= 6; j++)
                motorJ[j]->SetCurrentSetPoint(0);
            break;

        default:
            break;
        }
    }

    // 7. DOB compensation (runs in parallel with active mode)
    if (dobEnabled && stateMachine &&
        stateMachine->GetState() != RobotStateMachine::STATE_CTC_TORQUE)
    {
        RobustControlTick(_timeMillis);
    }

    // 8. Update telemetry
    if (telemetry)
    {
        TelemetryStreamer::TelemetryData_t td;
        for (int i = 0; i < 6; i++)
        {
            td.jointPos[i] = currentJoints.a[i];
            td.jointVel[i] = motorJ[i + 1]->estVelocity;
            td.jointAcc[i] = motorJ[i + 1]->estAcceleration;
            td.motorCurrent[i] = motorJ[i + 1]->motorCurrent;
        }
        td.cartPose[0] = currentPose6D.X; td.cartPose[1] = currentPose6D.Y;
        td.cartPose[2] = currentPose6D.Z; td.cartPose[3] = currentPose6D.A;
        td.cartPose[4] = currentPose6D.B; td.cartPose[5] = currentPose6D.C;

        if (forceEstimator)
        {
            const ForceEstimator::CartesianForce_t &F = forceEstimator->GetCartesianForce();
            td.extForce[0] = F.Fx; td.extForce[1] = F.Fy; td.extForce[2] = F.Fz;
            td.extForce[3] = F.Tx; td.extForce[4] = F.Ty; td.extForce[5] = F.Tz;
        }

        if (safetyMonitor)
        {
            const SafetyMonitor::SafetyStatus_t &ss = safetyMonitor->GetStatus();
            for (int i = 0; i < 6; i++)
                td.temperature[i] = ss.estTemperature[i];
            td.safetyLevel = (uint32_t)ss.level;
            td.safetyViolation = (uint32_t)ss.violation;
            td.safetyJoint = ss.violationJoint;
            td.safetyCount = ss.stopCount;
        }

        if (stateMachine)
        {
            td.stateCurrent = (uint32_t)stateMachine->GetState();
            td.statePrevious = (uint32_t)stateMachine->GetInfo().previous;
        }

        td.timestampMs = _timeMillis;
        td.loopTimeMs = (float)(_timeMillis - prevTickTime);
        td.jitterMs = fabsf(td.loopTimeMs - 1.0f);
        prevTickTime = _timeMillis;

        telemetry->Update(td);
    }
}


// ==================== Phase 7: Calibration & Workspace Analysis ====================

float DevClawRobot::AnalyzeWorkspace()
{
    if (!workspaceAnalyzer || !dynamicsSolver) return 0;

    float DEG2RAD = 0.01745329251994f;
    float qRad[6], J[36];
    for (int i = 0; i < 6; i++)
        qRad[i] = currentJoints.a[i] * DEG2RAD;

    dynamicsSolver->SolveJacobian(qRad, J);

    float q[6];
    for (int i = 0; i < 6; i++) q[i] = currentJoints.a[i];

    WorkspaceAnalyzer::AnalysisResult_t res = workspaceAnalyzer->Analyze(q, J);
    return res.manipulability;
}


bool DevClawRobot::IsNearSingularity()
{
    if (!workspaceAnalyzer || !dynamicsSolver) return false;

    float DEG2RAD = 0.01745329251994f;
    float qRad[6], J[36];
    for (int i = 0; i < 6; i++)
        qRad[i] = currentJoints.a[i] * DEG2RAD;

    dynamicsSolver->SolveJacobian(qRad, J);
    return workspaceAnalyzer->CheckSingularity(J);
}


bool DevClawRobot::RecordGravitySample()
{
    if (!gravCalibrator) return false;

    float q[6], current[6];
    for (int i = 0; i < 6; i++)
    {
        q[i] = currentJoints.a[i];
        current[i] = motorJ[i + 1]->motorCurrent;
    }

    int idx = gravCalibrator->RecordSample(q, current);
    return idx >= 0;
}


float DevClawRobot::RunGravityCalibration()
{
    if (!gravCalibrator) return -1;

    GravityCalibrator::CalibResult_t res = gravCalibrator->Calibrate();
    return res.valid ? res.residualNorm : -1.0f;
}


bool DevClawRobot::RecordKinematicSample(float _measX, float _measY, float _measZ)
{
    if (!kinCalibrator) return false;

    float q[6];
    for (int i = 0; i < 6; i++)
        q[i] = currentJoints.a[i];

    float measPos[3] = {_measX, _measY, _measZ};
    int idx = kinCalibrator->RecordSample(q, measPos);
    return idx >= 0;
}


float DevClawRobot::RunKinematicCalibration()
{
    if (!kinCalibrator) return -1;

    KinematicCalibrator::CalibResult_t res = kinCalibrator->CalibrateOffsets();
    return res.valid ? res.residualRMS : -1.0f;
}
