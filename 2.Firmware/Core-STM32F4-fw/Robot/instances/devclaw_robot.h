#ifndef REF_STM32F4_FW_DEVCLAW_ROBOT_H
#define REF_STM32F4_FW_DEVCLAW_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "algorithms/dynamics/6dof_dynamics.h"
#include "algorithms/trajectory/s_curve_planner.h"
#include "algorithms/trajectory/cartesian_planner.h"
#include "algorithms/control/disturbance_observer.h"
#include "algorithms/control/auto_tuner.h"
#include "algorithms/control/impedance_controller.h"
#include "algorithms/control/collision_detector.h"
#include "algorithms/control/force_estimator.h"
#include "algorithms/control/computed_torque.h"
#include "algorithms/control/friction_compensator.h"
#include "algorithms/control/teach_mode.h"
#include "algorithms/control/hybrid_force_position.h"
#include "algorithms/safety/safety_monitor.h"
#include "algorithms/safety/robot_state_machine.h"
#include "algorithms/safety/telemetry_streamer.h"
#include "algorithms/identification/param_identifier.h"
#include "algorithms/identification/gravity_calibrator.h"
#include "algorithms/kinematic/workspace_analyzer.h"
#include "algorithms/kinematic/kinematic_calibrator.h"
#include "algorithms/learning/dmp.h"
#include "algorithms/trajectory/trajectory_optimizer.h"
#include "actuators/ctrl_step/ctrl_step.hpp"

#define ALL 0

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 2               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 2               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */


class DummyHand
{
public:
    uint8_t nodeID = 7;
    float maxCurrent = 0.7;


    DummyHand(CAN_HandleTypeDef* _hcan, uint8_t _id);


    void SetAngle(float _angle);
    void SetMaxCurrent(float _val);
    void SetEnable(bool _enable);


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("set_angle", *this, &DummyHand::SetAngle, "angle"),
            make_protocol_function("set_enable", *this, &DummyHand::SetEnable, "enable"),
            make_protocol_function("set_current_limit", *this, &DummyHand::SetMaxCurrent, "current")
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8];
    CAN_TxHeaderTypeDef txHeader;
    float minAngle = 0;
    float maxAngle = 45;
};


class DevClawRobot
{
public:
    explicit DevClawRobot(CAN_HandleTypeDef* _hcan);
    ~DevClawRobot();


    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DevClawRobot* _context) : context(_context)
        {
        }

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);


        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }


    private:
        DevClawRobot* context;
        float time = 0;
        uint8_t tuningFlag = 0;
        float frequency = 1;
        float amplitude = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);


    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -73, 180, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED = 30;  // degree/s
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
    const float DEFAULT_JOINT_ACCELERATION_LOW = 30;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;  // 0~100
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;


    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;
    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor* motorJ[7] = {nullptr};
    DummyHand* hand = {nullptr};


    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    void UpdateJointPose6D();
    void Reboot();
    void SetEnable(bool _enable);
    void CalibrateHomeOffset();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);

    // ==================== Advanced Motion API (Phase 1 Optimizations) ====================

    /**
     * @brief S-Curve based MoveJ with jerk-limited smooth profile
     * @param _j1~_j6  Target joint angles (deg)
     * @param _velScale  Velocity scale factor (0~100)
     * @param _accScale  Acceleration scale factor (0~100)
     * @return true if motion is valid
     */
    bool MoveJ_SCurve(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6,
                      float _velScale = 50, float _accScale = 50);

    /**
     * @brief True Cartesian linear motion with S-Curve + SLERP interpolation
     * @param _x,_y,_z   Target position (mm)
     * @param _a,_b,_c   Target orientation (deg)
     * @param _linearSpeed  Cartesian speed (mm/s)
     * @return true if path is valid (all IK solutions exist)
     */
    bool MoveL_Cartesian(float _x, float _y, float _z, float _a, float _b, float _c,
                         float _linearSpeed = 100);

    /**
     * @brief Cartesian circular arc motion through via point
     */
    bool MoveC_Cartesian(float _vx, float _vy, float _vz, float _va, float _vb, float _vc,
                         float _ex, float _ey, float _ez, float _ea, float _eb, float _ec,
                         float _linearSpeed = 100);

    /**
     * @brief Compute and apply gravity compensation torques (call periodically)
     */
    void ApplyGravityCompensation();

    /**
     * @brief Compute full inverse dynamics feedforward torques
     * @param _targetJoints   Target joint angles
     * @param _jointVel       Target joint velocities (deg/s)
     * @param _jointAcc       Target joint accelerations (deg/s^2)
     */
    void ApplyDynamicsFeedforward(const DOF6Kinematic::Joint6D_t &_targetJoints,
                                  const float _jointVel[6],
                                  const float _jointAcc[6]);

    /**
     * @brief Get current manipulability index
     */
    float GetManipulability();

    /**
     * @brief Enable/disable dynamics feedforward compensation
     */
    void SetDynamicsEnabled(bool _enable);

    /**
     * @brief Execute a planned S-Curve trajectory tick (called from control loop)
     * @param _timeMillis  Time step in milliseconds
     */
    void SCurveTrajectoryTick(uint32_t _timeMillis);

    // ==================== Phase 2: Robust Control & Identification ====================

    /**
     * @brief Enable/disable disturbance observer compensation
     */
    void SetDOBEnabled(bool _enable);

    /**
     * @brief Set DOB cutoff frequency (Hz) for all joints
     */
    void SetDOBCutoff(float _freqHz);

    /**
     * @brief Start relay feedback auto-tuning for a specific joint
     * @param _jointIdx  Joint index (1-6)
     * @param _amplitude  Relay excitation amplitude
     */
    void StartAutoTune(uint32_t _jointIdx, float _amplitude);

    /**
     * @brief Check if auto-tuning is complete and apply gains
     */
    bool CheckAutoTuneComplete();

    /**
     * @brief Start dynamic parameter identification experiment
     */
    void StartIdentification();

    /**
     * @brief Run identification after data collection
     */
    bool RunIdentification();

    /**
     * @brief Control tick for DOB + adaptive control (call from main loop)
     */
    void RobustControlTick(uint32_t _timeMillis);

    // ==================== Phase 3: Safe Interaction & Learning ====================

    /**
     * @brief Set impedance control mode
     * @param _mode  0=off, 1=admittance, 2=impedance, 3=variable
     */
    void SetImpedanceMode(uint32_t _mode);

    /**
     * @brief Set Cartesian stiffness for all axes (N/m or N*m/rad)
     */
    void SetStiffness(float _stiffness);

    /**
     * @brief Set collision detection enabled and reaction strategy
     * @param _strategy  0=stop, 1=retract, 2=float, 3=reflex, 4=comply
     */
    void SetCollisionReaction(uint32_t _strategy);

    /**
     * @brief Reset collision detector after event
     */
    void ResetCollision();

    /**
     * @brief Start DMP recording (teach mode)
     */
    void StartDMPRecord();

    /**
     * @brief Stop DMP recording and learn
     * @param _duration  Recording duration in seconds
     */
    bool StopDMPRecord(float _duration);

    /**
     * @brief Execute learned DMP to new goal
     */
    bool ExecuteDMP(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);

    /**
     * @brief Safe interaction control tick (impedance + collision)
     */
    void SafeInteractionTick(uint32_t _timeMillis);

    /**
     * @brief DMP execution tick
     */
    void DMPTick();

    // ==================== Phase 4: Model-Based Control & Force Sensing ====================

    /**
     * @brief Enable/disable computed torque control mode
     *        (replaces default DCE with full model-based control)
     */
    void SetCTCEnabled(bool _enable);

    /**
     * @brief Set CTC natural frequency for all joints (Hz)
     */
    void SetCTCFrequency(float _freqHz);

    /**
     * @brief Enable/disable LuGre friction compensation
     */
    void SetFrictionCompEnabled(bool _enable);

    /**
     * @brief Calibrate force estimator deadzone (robot must be stationary)
     */
    void CalibrateForceEstimator();

    /**
     * @brief Request KF state from all motor drivers (call periodically)
     */
    void RequestMotorFeedback();

    /**
     * @brief Full model-based control tick (CTC + friction + force estimation)
     */
    void ModelBasedControlTick(uint32_t _timeMillis);

    /**
     * @brief Get estimated Cartesian force at end-effector
     */
    float GetExternalForce();

    // ==================== Phase 5: Teaching, Assembly & Trajectory Optimization ====================

    /**
     * @brief Enable/disable teach (lead-through) mode
     */
    void SetTeachMode(bool _enable);

    /**
     * @brief Save current position as teach waypoint
     */
    bool SaveTeachWaypoint();

    /**
     * @brief Start/stop teach trajectory recording
     */
    void SetTeachRecording(bool _record);

    /**
     * @brief Execute minimum-jerk trajectory to target
     */
    bool MoveJ_MinJerk(float _j1, float _j2, float _j3,
                        float _j4, float _j5, float _j6, float _duration);

    /**
     * @brief Set force/position hybrid control selection
     * @param _axis  0-5 (x,y,z,rx,ry,rz)
     * @param _forceMode  true=force control, false=position control
     */
    void SetHybridAxis(uint32_t _axis, bool _forceMode);

    /**
     * @brief Set force reference for hybrid controller
     */
    void SetForceRef(float _fx, float _fy, float _fz);

    /**
     * @brief Teach mode + force control tick
     */
    void TeachAndForceTick(uint32_t _timeMillis);

    // ==================== Phase 6: Safety, State Machine & Telemetry ====================

    /**
     * @brief Run safety check (call every control cycle)
     * @return Safety level (0=OK, 4=ESTOP)
     */
    uint32_t SafetyCheck();

    /**
     * @brief Reset emergency stop
     */
    bool ResetEmergencyStop();

    /**
     * @brief Request control mode transition
     * @param _mode  0=idle,1=position,2=ctc,3=impedance,4=teach,5=dmp,6=hybrid,7=minjerk
     */
    void RequestMode(uint32_t _mode);

    /**
     * @brief Set TCP speed limit for collaborative mode (mm/s)
     */
    void SetTCPSpeedLimit(float _speed);

    /**
     * @brief Configure telemetry channels and rate
     * @param _channels  Bitmask of channels
     * @param _rateHz    Streaming rate (Hz)
     */
    void ConfigTelemetry(uint32_t _channels, float _rateHz);

    /**
     * @brief Get current state machine state (as uint32)
     */
    uint32_t GetRobotState();

    /**
     * @brief Master control tick — orchestrates safety, state machine, and all subsystems
     */
    void MasterControlTick(uint32_t _timeMillis);

    // ==================== Phase 7: Calibration & Workspace Analysis ====================

    /**
     * @brief Run workspace analysis at current configuration
     * @return Manipulability index
     */
    float AnalyzeWorkspace();

    /**
     * @brief Record gravity calibration sample at current static pose
     */
    bool RecordGravitySample();

    /**
     * @brief Run gravity calibration on recorded samples
     * @return Residual RMS error
     */
    float RunGravityCalibration();

    /**
     * @brief Record kinematic calibration sample
     * @param _measX, _measY, _measZ  Measured TCP position (mm)
     */
    bool RecordKinematicSample(float _measX, float _measY, float _measZ);

    /**
     * @brief Run kinematic (joint offset) calibration
     * @return Residual RMS error (mm)
     */
    float RunKinematicCalibration();

    /**
     * @brief Check if near singularity
     */
    bool IsNearSingularity();


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DevClawRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DevClawRobot::Homing),
            make_protocol_function("resting", *this, &DevClawRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot", *this, &DevClawRobot::Reboot),
            make_protocol_function("set_enable", *this, &DevClawRobot::SetEnable, "enable"),
            make_protocol_function("move_j", *this, &DevClawRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6"),
            make_protocol_function("move_l", *this, &DevClawRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("set_joint_speed", *this, &DevClawRobot::SetJointSpeed, "speed"),
            make_protocol_function("set_joint_acc", *this, &DevClawRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DevClawRobot::SetCommandMode, "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions()),
            // Phase 1: Advanced Motion API
            make_protocol_function("move_j_scurve", *this, &DevClawRobot::MoveJ_SCurve,
                                   "j1", "j2", "j3", "j4", "j5", "j6", "vel_scale", "acc_scale"),
            make_protocol_function("move_l_cart", *this, &DevClawRobot::MoveL_Cartesian,
                                   "x", "y", "z", "a", "b", "c", "speed"),
            make_protocol_function("set_dynamics_enabled", *this, &DevClawRobot::SetDynamicsEnabled, "enable"),
            make_protocol_function("get_manipulability", *this, &DevClawRobot::GetManipulability),
            // Phase 2: Robust Control & Identification API
            make_protocol_function("set_dob_enabled", *this, &DevClawRobot::SetDOBEnabled, "enable"),
            make_protocol_function("set_dob_cutoff", *this, &DevClawRobot::SetDOBCutoff, "freq"),
            make_protocol_function("start_auto_tune", *this, &DevClawRobot::StartAutoTune, "joint", "amplitude"),
            make_protocol_function("start_identification", *this, &DevClawRobot::StartIdentification),
            make_protocol_function("run_identification", *this, &DevClawRobot::RunIdentification),
            // Phase 3: Safe Interaction & Learning API
            make_protocol_function("set_impedance_mode", *this, &DevClawRobot::SetImpedanceMode, "mode"),
            make_protocol_function("set_stiffness", *this, &DevClawRobot::SetStiffness, "stiffness"),
            make_protocol_function("set_collision_reaction", *this, &DevClawRobot::SetCollisionReaction, "strategy"),
            make_protocol_function("reset_collision", *this, &DevClawRobot::ResetCollision),
            make_protocol_function("start_dmp_record", *this, &DevClawRobot::StartDMPRecord),
            make_protocol_function("stop_dmp_record", *this, &DevClawRobot::StopDMPRecord, "duration"),
            make_protocol_function("execute_dmp", *this, &DevClawRobot::ExecuteDMP,
                                   "j1", "j2", "j3", "j4", "j5", "j6"),
            // Phase 4: Model-Based Control API
            make_protocol_function("set_ctc_enabled", *this, &DevClawRobot::SetCTCEnabled, "enable"),
            make_protocol_function("set_ctc_frequency", *this, &DevClawRobot::SetCTCFrequency, "freq"),
            make_protocol_function("set_friction_comp", *this, &DevClawRobot::SetFrictionCompEnabled, "enable"),
            make_protocol_function("calibrate_force", *this, &DevClawRobot::CalibrateForceEstimator),
            make_protocol_function("get_ext_force", *this, &DevClawRobot::GetExternalForce),
            // Phase 5: Teaching & Assembly API
            make_protocol_function("set_teach_mode", *this, &DevClawRobot::SetTeachMode, "enable"),
            make_protocol_function("save_waypoint", *this, &DevClawRobot::SaveTeachWaypoint),
            make_protocol_function("set_teach_recording", *this, &DevClawRobot::SetTeachRecording, "record"),
            make_protocol_function("move_j_minjerk", *this, &DevClawRobot::MoveJ_MinJerk,
                                   "j1", "j2", "j3", "j4", "j5", "j6", "duration"),
            make_protocol_function("set_hybrid_axis", *this, &DevClawRobot::SetHybridAxis, "axis", "force_mode"),
            make_protocol_function("set_force_ref", *this, &DevClawRobot::SetForceRef, "fx", "fy", "fz"),
            // Phase 6: Safety & Telemetry API
            make_protocol_function("safety_check", *this, &DevClawRobot::SafetyCheck),
            make_protocol_function("reset_estop", *this, &DevClawRobot::ResetEmergencyStop),
            make_protocol_function("request_mode", *this, &DevClawRobot::RequestMode, "mode"),
            make_protocol_function("set_tcp_speed_limit", *this, &DevClawRobot::SetTCPSpeedLimit, "speed"),
            make_protocol_function("config_telemetry", *this, &DevClawRobot::ConfigTelemetry, "channels", "rate"),
            make_protocol_function("get_robot_state", *this, &DevClawRobot::GetRobotState),
            // Phase 7: Calibration & Workspace API
            make_protocol_function("analyze_workspace", *this, &DevClawRobot::AnalyzeWorkspace),
            make_protocol_function("record_grav_sample", *this, &DevClawRobot::RecordGravitySample),
            make_protocol_function("run_grav_calib", *this, &DevClawRobot::RunGravityCalibration),
            make_protocol_function("record_kin_sample", *this, &DevClawRobot::RecordKinematicSample,
                                   "x", "y", "z"),
            make_protocol_function("run_kin_calib", *this, &DevClawRobot::RunKinematicCalibration),
            make_protocol_function("is_near_singular", *this, &DevClawRobot::IsNearSingularity)
        );
    }


    class CommandHandler
    {
    public:
        explicit CommandHandler(DevClawRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }

        uint32_t Push(const std::string &_cmd);
        std::string Pop(uint32_t timeout);
        uint32_t ParseCommand(const std::string &_cmd);
        uint32_t GetSpace();
        void ClearFifo();
        void EmergencyStop();


    private:
        DevClawRobot* context;
        osMessageQueueId_t commandFifo;
        char strBuffer[64]{};
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic* dof6Solver;
    DOF6Dynamics* dynamicsSolver = nullptr;
    CartesianPlanner* cartPlanner = nullptr;
    MultiAxisSCurvePlanner* multiAxisPlanner = nullptr;
    bool isEnabled = false;
    bool dynamicsEnabled = false;
    bool scurveActive = false;
    float scurveElapsedTime = 0;

    // Per-axis S-Curve constraints
    static constexpr float JOINT_VMAX[6] = {60, 60, 60, 90, 90, 120};   // deg/s
    static constexpr float JOINT_AMAX[6] = {200, 150, 200, 300, 300, 400}; // deg/s^2
    static constexpr float JOINT_JMAX[6] = {1000, 800, 1000, 1500, 1500, 2000}; // deg/s^3
    static constexpr float CARTESIAN_ACC = 500;   // mm/s^2
    static constexpr float CARTESIAN_JERK = 5000; // mm/s^3

    // Phase 2: Robust control & identification modules
    MultiJointDOB* multiDOB = nullptr;
    AutoTuner* autoTuner = nullptr;
    ParamIdentifier* paramIdent = nullptr;
    bool dobEnabled = false;
    bool identRunning = false;
    uint32_t autoTuneJoint = 0; // 0 = none

    // Phase 3: Safe interaction & learning modules
    ImpedanceController* impedanceCtrl = nullptr;
    CollisionDetector* collisionDetector = nullptr;
    MultiDOF_DMP* dmpLearner = nullptr;
    bool impedanceActive = false;
    bool collisionDetEnabled = false;
    bool dmpRecording = false;
    bool dmpExecuting = false;

    // Phase 4: Model-based control & force sensing modules
    ForceEstimator* forceEstimator = nullptr;
    ComputedTorqueController* ctcController = nullptr;
    FrictionCompensator* frictionComp = nullptr;
    bool ctcEnabled = false;
    bool frictionCompEnabled = false;

    // Phase 5: Teaching, assembly & trajectory optimization modules
    TeachMode* teachMode = nullptr;
    HybridForcePositionController* hybridCtrl = nullptr;
    TrajectoryOptimizer* trajOptimizer = nullptr;
    bool teachActive = false;
    bool hybridActive = false;
    bool minJerkActive = false;
    float minJerkTimer = 0;

    // Phase 6: Safety, state machine & telemetry modules
    SafetyMonitor* safetyMonitor = nullptr;
    RobotStateMachine* stateMachine = nullptr;
    TelemetryStreamer* telemetry = nullptr;
    uint32_t prevTickTime = 0;

    // Phase 7: Calibration & workspace analysis modules
    WorkspaceAnalyzer* workspaceAnalyzer = nullptr;
    GravityCalibrator* gravCalibrator = nullptr;
    KinematicCalibrator* kinCalibrator = nullptr;
};


#endif //REF_STM32F4_FW_DEVCLAW_ROBOT_H
