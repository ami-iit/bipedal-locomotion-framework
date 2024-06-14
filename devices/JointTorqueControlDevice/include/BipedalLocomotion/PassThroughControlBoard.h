/**
 * @file PassThroughControlBoard.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_PASS_THROUGH_CONTROL_BOARD_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_PASS_THROUGH_CONTROL_BOARD_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAmplifierControl.h>
#include <yarp/dev/IControlCalibration.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

namespace BipedalLocomotion
{
class PassThroughControlBoard;
}

/**
 *
 * \brief PassThroughControlBoard: base class for devices can be used to modify the behavior of a
 * controlboard device.
 *
 */
class BipedalLocomotion::PassThroughControlBoard : public yarp::dev::DeviceDriver,
                                                   public yarp::dev::IEncodersTimed,
                                                   public yarp::dev::IPositionControl,
                                                   public yarp::dev::IVelocityControl,
                                                   public yarp::dev::IPositionDirect,
                                                   public yarp::dev::IControlMode,
                                                   public yarp::dev::ITorqueControl,
                                                   public yarp::dev::IPWMControl,
                                                   public yarp::dev::IInteractionMode,
                                                   public yarp::dev::IPidControl,
                                                   public yarp::dev::ICurrentControl,
                                                   public yarp::dev::IMotorEncoders,
                                                   public yarp::dev::IAxisInfo,
                                                   public yarp::dev::IMultipleWrapper,
                                                   public yarp::dev::IAmplifierControl,
                                                   public yarp::dev::IControlCalibration,
                                                   public yarp::dev::IControlLimits
{
protected:
    yarp::dev::IEncodersTimed* proxyIEncodersTimed;
    yarp::dev::IPositionControl* proxyIPositionControl;
    yarp::dev::IVelocityControl* proxyIVelocityControl;
    yarp::dev::IPositionDirect* proxyIPositionDirect;
    yarp::dev::IControlMode* proxyIControlMode;
    yarp::dev::ITorqueControl* proxyITorqueControl;
    yarp::dev::IPWMControl* proxyIOpenLoopControl;
    yarp::dev::IInteractionMode* proxyIInteractionMode;
    yarp::dev::IPidControl* proxyIPidControl;
    yarp::dev::ICurrentControl* proxyICurrentControl;
    yarp::dev::IMotorEncoders* proxyIMotorEncoders;
    yarp::dev::IAxisInfo* proxyIAxisInfo;
    yarp::dev::IAmplifierControl* proxyIAmplifierControl;
    yarp::dev::IControlCalibration* proxyIControlCalibration;
    yarp::dev::IControlLimits* proxyIControlLimits;
    void proxyIMotorEncoder(const double* vals);
    void resetPointers();

public:
    // CONSTRUCTOR
    PassThroughControlBoard();
    ~PassThroughControlBoard();

    // DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    // IMULTIPLEWRAPPER
    virtual bool attachAll(const yarp::dev::PolyDriverList& p);
    virtual bool detachAll();

    // ENCODERS
    virtual bool getAxes(int* ax);
    virtual bool resetEncoder(int j);
    virtual bool resetEncoders();
    virtual bool setEncoder(int j, double val);
    virtual bool setEncoders(const double* vals);
    virtual bool getEncoder(int j, double* v);
    virtual bool getEncoders(double* encs);
    virtual bool getEncoderSpeed(int j, double* sp);
    virtual bool getEncoderSpeeds(double* spds);
    virtual bool getEncoderAcceleration(int j, double* spds);
    virtual bool getEncoderAccelerations(double* accs);

    // ENCODERS TIMED
    virtual bool getEncodersTimed(double* encs, double* time);
    virtual bool getEncoderTimed(int j, double* encs, double* time);

    // MOTOR ENCODERS
    virtual bool getNumberOfMotorEncoders(int* num);
    virtual bool resetMotorEncoder(int m);
    virtual bool resetMotorEncoders();
    virtual bool setMotorEncoderCountsPerRevolution(int m, const double cpr);
    virtual bool getMotorEncoderCountsPerRevolution(int m, double* cpr);
    virtual bool setMotorEncoder(int m, const double val);
    virtual bool setMotorEncoders(const double* vals);
    virtual bool getMotorEncoder(int m, double* v);
    virtual bool getMotorEncoders(double* encs);
    virtual bool getMotorEncodersTimed(double* encs, double* time);
    virtual bool getMotorEncoderTimed(int m, double* encs, double* time);
    virtual bool getMotorEncoderSpeed(int m, double* sp);
    virtual bool getMotorEncoderSpeeds(double* spds);
    virtual bool getMotorEncoderAcceleration(int m, double* acc);
    virtual bool getMotorEncoderAccelerations(double* accs);

    // POSITION CONTROL
    virtual bool positionMove(int j, double ref);
    virtual bool positionMove(const double* refs);
    virtual bool relativeMove(int j, double delta);
    virtual bool relativeMove(const double* deltas);
    virtual bool checkMotionDone(int j, bool* flag);
    virtual bool checkMotionDone(bool* flag);
    virtual bool setRefSpeed(int j, double sp);
    virtual bool setRefSpeeds(const double* spds);
    virtual bool setRefAcceleration(int j, double acc);
    virtual bool setRefAccelerations(const double* accs);
    virtual bool getRefSpeed(int j, double* ref);
    virtual bool getRefSpeeds(double* spds);
    virtual bool getRefAcceleration(int j, double* acc);
    virtual bool getRefAccelerations(double* accs);
    virtual bool stop(int j);
    virtual bool stop();

    // POSITION CONTROL
    virtual bool positionMove(const int n_joint, const int* joints, const double* refs);
    virtual bool relativeMove(const int n_joint, const int* joints, const double* deltas);
    virtual bool checkMotionDone(const int n_joint, const int* joints, bool* flags);
    virtual bool setRefSpeeds(const int n_joint, const int* joints, const double* spds);
    virtual bool setRefAccelerations(const int n_joint, const int* joints, const double* accs);
    virtual bool getRefSpeeds(const int n_joint, const int* joints, double* spds);
    virtual bool getRefAccelerations(const int n_joint, const int* joints, double* accs);
    virtual bool stop(const int n_joint, const int* joints);
    virtual bool getTargetPosition(const int joint, double* ref);
    virtual bool getTargetPositions(double* refs);
    virtual bool getTargetPositions(const int n_joint, const int* joints, double* refs);

    // VELOCITY CONTROL
    virtual bool velocityMove(int j, double sp);
    virtual bool velocityMove(const double* sp);

    // VELOCITY CONTROL
    virtual bool velocityMove(const int n_joint, const int* joints, const double* spds);
    virtual bool getRefVelocity(const int joint, double* vel);
    virtual bool getRefVelocities(double* vels);
    virtual bool getRefVelocities(const int n_joint, const int* joints, double* vels);

    // POSITION DIRECT CONTROL
    virtual bool setPosition(int j, double ref);
    virtual bool setPositions(const int n_joint, const int* joints, const double* refs);
    virtual bool setPositions(const double* refs);
    virtual bool getRefPosition(const int joint, double* ref);
    virtual bool getRefPositions(double* refs);
    virtual bool getRefPositions(const int n_joint, const int* joints, double* refs);

    // CONTROL MODE
    virtual bool getControlMode(int j, int* mode);
    virtual bool getControlModes(int* modes);

    // CONTROL MODE
    virtual bool getControlModes(const int n_joint, const int* joints, int* modes);
    virtual bool setControlMode(const int j, const int mode);
    virtual bool setControlModes(const int n_joint, const int* joints, int* modes);
    virtual bool setControlModes(int* modes);

    // TORQUE CONTROL
    virtual bool getRefTorques(double* t);
    virtual bool getRefTorque(int j, double* t);
    virtual bool setRefTorques(const double* t);
    virtual bool setRefTorque(int j, double t);
    virtual bool getTorque(int j, double* t);
    virtual bool getTorques(double* t);
    virtual bool getTorqueRange(int j, double* min, double* max);
    virtual bool getTorqueRanges(double* min, double* max);

    // OPEN LOOP CONTROL
    virtual bool getNumberOfMotors(int* n);
    virtual bool setRefDutyCycle(int m, double ref);
    virtual bool setRefDutyCycles(const double* refs);
    virtual bool getRefDutyCycle(int m, double* ref);
    virtual bool getRefDutyCycles(double* refs);
    virtual bool getDutyCycle(int m, double* val);
    virtual bool getDutyCycles(double* vals);

    // CURRENT CONTROL
    virtual bool getCurrent(int m, double* curr);
    virtual bool getCurrents(double* currs);
    virtual bool getCurrentRange(int m, double* min, double* max);
    virtual bool getCurrentRanges(double* min, double* max);
    virtual bool setRefCurrents(const double* currs);
    virtual bool setRefCurrent(int m, double curr);
    virtual bool setRefCurrents(const int n_motor, const int* motors, const double* currs);
    virtual bool getRefCurrents(double* currs);
    virtual bool getRefCurrent(int m, double* curr);

    // AMPLIFIER CONTROL
    virtual bool enableAmp(int j);
    virtual bool disableAmp(int j);
    virtual bool getAmpStatus(int* st);
    virtual bool getAmpStatus(int j, int* v);
    virtual bool getMaxCurrent(int j, double* v);
    virtual bool setMaxCurrent(int j, double v);
    virtual bool getNominalCurrent(int m, double* val);
    virtual bool setNominalCurrent(int m, const double val);
    virtual bool getPeakCurrent(int m, double* val);
    virtual bool setPeakCurrent(int m, const double val);
    virtual bool getPWM(int j, double* val);
    virtual bool getPWMLimit(int j, double* val);
    virtual bool setPWMLimit(int j, const double val);
    virtual bool getPowerSupplyVoltage(int j, double* val);

    // INTERACTION MODE
    virtual bool getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode);
    virtual bool
    getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool getInteractionModes(yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode);
    virtual bool
    setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes);
    virtual bool setInteractionModes(yarp::dev::InteractionModeEnum* modes);

    // PID CONTROL INTERFACE METHODS
    virtual bool
    setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, const yarp::dev::Pid& pid);
    virtual bool setPids(const yarp::dev::PidControlTypeEnum& pidtype, const yarp::dev::Pid* pids);
    virtual bool setPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double ref);
    virtual bool setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, const double* refs);
    virtual bool
    setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double limit);
    virtual bool
    setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, const double* limits);
    virtual bool getPidError(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* err);
    virtual bool getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype, double* errs);
    virtual bool getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* out);
    virtual bool getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype, double* outs);
    virtual bool getPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::Pid* pid);
    virtual bool getPids(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid* pids);
    virtual bool getPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* ref);
    virtual bool getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, double* refs);
    virtual bool
    getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* limit);
    virtual bool getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, double* limits);
    virtual bool resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    virtual bool disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    virtual bool enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j);
    virtual bool setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v);
    virtual bool isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled);

    // AXIS INFO
    virtual bool getAxisName(int axis, std::string& name);
    virtual bool getJointType(int axis, yarp::dev::JointTypeEnum& type);

    // CONTROL CALIBRATION
    virtual bool
    calibrateAxisWithParams(int axis, unsigned int type, double p1, double p2, double p3);
    virtual bool setCalibrationParameters(int axis, const yarp::dev::CalibrationParameters& params);
    virtual bool calibrationDone(int j);
    virtual bool setCalibrator(yarp::dev::ICalibrator* c);
    virtual bool calibrateRobot();
    virtual bool park(bool wait = true);
    virtual bool abortCalibration();
    virtual bool abortPark();

    // CONTROL LIMITS
    virtual bool setLimits(int axis, double min, double max);
    virtual bool getLimits(int axis, double* min, double* max);
    virtual bool setVelLimits(int axis, double min, double max);
    virtual bool getVelLimits(int axis, double* min, double* max);
};

#endif /* CODYCO_PASS_THROUGHT_CONTROL_BOARD_H */
