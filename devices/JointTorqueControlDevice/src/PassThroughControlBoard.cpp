/**
 * @file PassThroughControlBoard.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/PassThroughControlBoard.h>
#include <iostream>
#include <yarp/os/Property.h>

using namespace yarp::dev;
using namespace BipedalLocomotion;

// CONSTRUCTOR
PassThroughControlBoard::PassThroughControlBoard()
    : proxyIEncodersTimed(0)
    , proxyIPositionControl(0)
    , proxyIVelocityControl(0)
    , proxyIPositionDirect(0)
    , proxyIControlMode(0)
    , proxyITorqueControl(0)
    , proxyIOpenLoopControl(0)
    , proxyIInteractionMode(0)
    , proxyIPidControl(0)
    , proxyICurrentControl(0)
    , proxyIMotorEncoders(0)
    , proxyIAxisInfo(0)
    , proxyIAmplifierControl(0)
    , proxyIControlCalibration(0)
    , proxyIControlLimits(0)
    , proxyIMotor(0)
{
}

PassThroughControlBoard::~PassThroughControlBoard()
{
    this->close();
}

void PassThroughControlBoard::resetPointers()
{
    proxyIEncodersTimed = nullptr;
    proxyIPositionControl = nullptr;
    proxyIControlMode = nullptr;
    proxyITorqueControl = nullptr;
    proxyIOpenLoopControl = nullptr;
    proxyIVelocityControl = nullptr;
    proxyIPositionDirect = nullptr;
    proxyIInteractionMode = nullptr;
    proxyIPidControl = nullptr;
    proxyICurrentControl = nullptr;
    proxyIMotorEncoders = nullptr;
    proxyIAxisInfo = nullptr;
    proxyIAmplifierControl = nullptr;
    proxyIControlCalibration = nullptr;
    proxyIControlLimits = nullptr;
    proxyIMotor = nullptr;
}

// DEVICE DRIVER
bool PassThroughControlBoard::open(yarp::os::Searchable& config)
{
    bool legacyOption = false;
    legacyOption = legacyOption || config.check("proxy_remote");
    legacyOption = legacyOption || config.check("proxy_local");

    if (legacyOption)
    {
        yError("PassThroughControlBoard error: legacy option proxy_remote and proxy_local are not "
               "supported anymore. Please use the yarprobotinterface to compose devices.\n");
        return false;
    }

    // Do nothing, the actual proxies are initialized in the attachAll method,
    // that is called by usercode or by the yarprobotinterface
    return true;
}

bool PassThroughControlBoard::close()
{
    resetPointers();
    return true;
}

bool PassThroughControlBoard::attachAll(const PolyDriverList& p)
{
    if (p.size() != 1)
    {
        yError("PassThroughControlBoard error: %d devices passed to attachAll, but only 1 is "
               "supported. If you want to connect several device to a single "
               "PassThroughControlBoard, please use a controlboardremapper device in the middle.\n",
               p.size());
        return false;
    }

    PolyDriver* proxyDevice = p[0]->poly;
    proxyDevice->view(proxyIEncodersTimed);
    proxyDevice->view(proxyIPositionControl);
    proxyDevice->view(proxyIControlMode);
    proxyDevice->view(proxyITorqueControl);
    proxyDevice->view(proxyIOpenLoopControl);
    proxyDevice->view(proxyIVelocityControl);
    proxyDevice->view(proxyIPositionDirect);
    proxyDevice->view(proxyIInteractionMode);
    proxyDevice->view(proxyIPidControl);
    proxyDevice->view(proxyICurrentControl);
    proxyDevice->view(proxyIMotorEncoders);
    proxyDevice->view(proxyIAxisInfo);
    proxyDevice->view(proxyIAmplifierControl);
    proxyDevice->view(proxyIControlCalibration);
    proxyDevice->view(proxyIControlLimits);
    proxyDevice->view(proxyIMotor);

    return true;
}

bool PassThroughControlBoard::detachAll()
{
    resetPointers();
    return true;
}

// ENCODERS
bool PassThroughControlBoard::getAxes(int* ax)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getAxes(ax);
}

bool PassThroughControlBoard::resetEncoder(int j)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->resetEncoder(j);
}

bool PassThroughControlBoard::resetEncoders()
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->resetEncoders();
}

bool PassThroughControlBoard::setEncoder(int j, double val)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->setEncoder(j, val);
}

bool PassThroughControlBoard::setEncoders(const double* vals)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->setEncoders(vals);
}

bool PassThroughControlBoard::getEncoder(int j, double* v)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoder(j, v);
}

bool PassThroughControlBoard::getEncoders(double* encs)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoders(encs);
}

bool PassThroughControlBoard::getEncoderSpeed(int j, double* sp)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderSpeed(j, sp);
}

bool PassThroughControlBoard::getEncoderSpeeds(double* spds)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderSpeeds(spds);
}

bool PassThroughControlBoard::getEncoderAcceleration(int j, double* spds)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderAcceleration(j, spds);
}

bool PassThroughControlBoard::getEncoderAccelerations(double* accs)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderAccelerations(accs);
}

// ENCODERS TIMED
bool PassThroughControlBoard::getEncodersTimed(double* encs, double* time)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getEncodersTimed(encs, time);
}

bool PassThroughControlBoard::getEncoderTimed(int j, double* encs, double* time)
{
    if (!proxyIEncodersTimed)
    {
        return false;
    }
    return proxyIEncodersTimed->getEncoderTimed(j, encs, time);
}

// MOTOR ENCODERS
bool PassThroughControlBoard::getNumberOfMotorEncoders(int* num)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getNumberOfMotorEncoders(num);
}

bool PassThroughControlBoard::resetMotorEncoder(int m)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->resetMotorEncoder(m);
}

bool PassThroughControlBoard::resetMotorEncoders()
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->resetMotorEncoders();
}

bool PassThroughControlBoard::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->setMotorEncoderCountsPerRevolution(m, cpr);
}

bool PassThroughControlBoard::getMotorEncoderCountsPerRevolution(int m, double* cpr)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncoderCountsPerRevolution(m, cpr);
}

bool PassThroughControlBoard::setMotorEncoder(int m, const double val)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->setMotorEncoder(m, val);
}

bool PassThroughControlBoard::setMotorEncoders(const double* vals)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->setMotorEncoders(vals);
}

bool PassThroughControlBoard::getMotorEncoder(int m, double* v)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncoder(m, v);
}

bool PassThroughControlBoard::getMotorEncoders(double* encs)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncoders(encs);
}

bool PassThroughControlBoard::getMotorEncodersTimed(double* encs, double* time)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncodersTimed(encs, time);
}

bool PassThroughControlBoard::getMotorEncoderTimed(int m, double* encs, double* time)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncoderTimed(m, encs, time);
}

bool PassThroughControlBoard::getMotorEncoderSpeed(int m, double* sp)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncoderSpeed(m, sp);
}

bool PassThroughControlBoard::getMotorEncoderSpeeds(double* spds)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncoderSpeeds(spds);
}

bool PassThroughControlBoard::getMotorEncoderAcceleration(int m, double* acc)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncoderAcceleration(m, acc);
}

bool PassThroughControlBoard::getMotorEncoderAccelerations(double* accs)
{
    if (!proxyIMotorEncoders)
    {
        return false;
    }
    return proxyIMotorEncoders->getMotorEncoderAccelerations(accs);
}

// POSITION CONTROL
bool PassThroughControlBoard::positionMove(int j, double ref)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->positionMove(j, ref);
}

bool PassThroughControlBoard::positionMove(const double* refs)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->positionMove(refs);
}

bool PassThroughControlBoard::relativeMove(int j, double delta)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->relativeMove(j, delta);
}

bool PassThroughControlBoard::relativeMove(const double* deltas)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->relativeMove(deltas);
}

bool PassThroughControlBoard::checkMotionDone(int j, bool* flag)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->checkMotionDone(j, flag);
}

bool PassThroughControlBoard::checkMotionDone(bool* flag)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->checkMotionDone(flag);
}

bool PassThroughControlBoard::setRefSpeed(int j, double sp)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->setRefSpeed(j, sp);
}

bool PassThroughControlBoard::setRefSpeeds(const double* spds)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->setRefSpeeds(spds);
}

bool PassThroughControlBoard::setRefAcceleration(int j, double acc)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->setRefAcceleration(j, acc);
}

bool PassThroughControlBoard::setRefAccelerations(const double* accs)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->setRefAccelerations(accs);
}

bool PassThroughControlBoard::getRefSpeed(int j, double* ref)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getRefSpeed(j, ref);
}

bool PassThroughControlBoard::getRefSpeeds(double* spds)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getRefSpeeds(spds);
}

bool PassThroughControlBoard::getRefAcceleration(int j, double* acc)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getRefAcceleration(j, acc);
}

bool PassThroughControlBoard::getRefAccelerations(double* accs)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getRefAccelerations(accs);
}

bool PassThroughControlBoard::stop(int j)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->stop(j);
}

bool PassThroughControlBoard::stop()
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->stop();
}

// POSITION CONTROL
bool PassThroughControlBoard::positionMove(const int n_joint, const int* joints, const double* refs)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->positionMove(n_joint, joints, refs);
}

bool PassThroughControlBoard::relativeMove(const int n_joint,
                                           const int* joints,
                                           const double* deltas)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->relativeMove(n_joint, joints, deltas);
}

bool PassThroughControlBoard::checkMotionDone(const int n_joint, const int* joints, bool* flags)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->checkMotionDone(n_joint, joints, flags);
}

bool PassThroughControlBoard::setRefSpeeds(const int n_joint, const int* joints, const double* spds)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->setRefSpeeds(n_joint, joints, spds);
}

bool PassThroughControlBoard::setRefAccelerations(const int n_joint,
                                                  const int* joints,
                                                  const double* accs)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->setRefAccelerations(n_joint, joints, accs);
}

bool PassThroughControlBoard::getRefSpeeds(const int n_joint, const int* joints, double* spds)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getRefSpeeds(n_joint, joints, spds);
}

bool PassThroughControlBoard::getRefAccelerations(const int n_joint,
                                                  const int* joints,
                                                  double* accs)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getRefAccelerations(n_joint, joints, accs);
}

bool PassThroughControlBoard::stop(const int n_joint, const int* joints)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->stop(n_joint, joints);
}

bool PassThroughControlBoard::getTargetPosition(const int joint, double* ref)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getTargetPosition(joint, ref);
}

bool PassThroughControlBoard::getTargetPositions(double* refs)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getTargetPositions(refs);
}

bool PassThroughControlBoard::getTargetPositions(const int n_joint, const int* joints, double* refs)
{
    if (!proxyIPositionControl)
    {
        return false;
    }
    return proxyIPositionControl->getTargetPositions(n_joint, joints, refs);
}

// VELOCITY CONTROL
bool PassThroughControlBoard::velocityMove(int j, double sp)
{
    if (!proxyIVelocityControl)
    {
        return false;
    }
    return proxyIVelocityControl->velocityMove(j, sp);
}

bool PassThroughControlBoard::velocityMove(const double* sp)
{
    if (!proxyIVelocityControl)
    {
        return false;
    }
    return proxyIVelocityControl->velocityMove(sp);
}

// VELOCITY CONTROL
bool PassThroughControlBoard::velocityMove(const int n_joint, const int* joints, const double* spds)
{
    if (!proxyIVelocityControl)
    {
        return false;
    }
    return proxyIVelocityControl->velocityMove(n_joint, joints, spds);
}

bool PassThroughControlBoard::getRefVelocity(const int joint, double* vel)
{
    if (!proxyIVelocityControl)
    {
        return false;
    }
    return proxyIVelocityControl->getRefVelocity(joint, vel);
}

bool PassThroughControlBoard::getRefVelocities(double* vels)
{
    if (!proxyIVelocityControl)
    {
        return false;
    }
    return proxyIVelocityControl->getRefVelocities(vels);
}

bool PassThroughControlBoard::getRefVelocities(const int n_joint, const int* joints, double* vels)
{
    if (!proxyIVelocityControl)
    {
        return false;
    }
    return proxyIVelocityControl->getRefVelocities(n_joint, joints, vels);
}

// POSITION DIRECT CONTROL
bool PassThroughControlBoard::setPosition(int j, double ref)
{
    if (!proxyIPositionDirect)
    {
        return false;
    }
    return proxyIPositionDirect->setPosition(j, ref);
}

bool PassThroughControlBoard::setPositions(const int n_joint, const int* joints, const double* refs)
{
    if (!proxyIPositionDirect)
    {
        return false;
    }
    return proxyIPositionDirect->setPositions(n_joint, joints, refs);
}

bool PassThroughControlBoard::setPositions(const double* refs)
{
    if (!proxyIPositionDirect)
    {
        return false;
    }
    return proxyIPositionDirect->setPositions(refs);
}

bool PassThroughControlBoard::getRefPosition(const int joint, double* ref)
{
    if (!proxyIPositionDirect)
    {
        return false;
    }
    return proxyIPositionDirect->getRefPosition(joint, ref);
}

bool PassThroughControlBoard::getRefPositions(double* refs)
{
    if (!proxyIPositionDirect)
    {
        return false;
    }
    return proxyIPositionDirect->getRefPositions(refs);
}

bool PassThroughControlBoard::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    if (!proxyIPositionDirect)
    {
        return false;
    }
    return proxyIPositionDirect->getRefPositions(n_joint, joints, refs);
}

// CONTROL MODE
bool PassThroughControlBoard::getControlMode(int j, int* mode)
{
    if (!proxyIControlMode)
    {
        return false;
    }
    return proxyIControlMode->getControlMode(j, mode);
}

bool PassThroughControlBoard::getControlModes(int* modes)
{
    if (!proxyIControlMode)
    {
        return false;
    }
    return proxyIControlMode->getControlModes(modes);
}

// CONTROL MODE
bool PassThroughControlBoard::getControlModes(const int n_joint, const int* joints, int* modes)
{
    if (!proxyIControlMode)
    {
        return false;
    }
    return proxyIControlMode->getControlModes(n_joint, joints, modes);
}

bool PassThroughControlBoard::setControlMode(const int j, const int mode)
{
    if (!proxyIControlMode)
    {
        return false;
    }
    return proxyIControlMode->setControlMode(j, mode);
}

bool PassThroughControlBoard::setControlModes(const int n_joint, const int* joints, int* modes)
{
    if (!proxyIControlMode)
    {
        return false;
    }
    return proxyIControlMode->setControlModes(n_joint, joints, modes);
}

bool PassThroughControlBoard::setControlModes(int* modes)
{
    if (!proxyIControlMode)
    {
        return false;
    }
    return proxyIControlMode->setControlModes(modes);
}

// TORQUE CONTROL
bool PassThroughControlBoard::getRefTorques(double* t)
{
    if (!proxyITorqueControl)
    {
        return false;
    }
    return proxyITorqueControl->getRefTorques(t);
}

bool PassThroughControlBoard::getRefTorque(int j, double* t)
{
    if (!proxyITorqueControl)
    {
        return false;
    }
    return proxyITorqueControl->getRefTorque(j, t);
}

bool PassThroughControlBoard::setRefTorques(const double* t)
{
    if (!proxyITorqueControl)
    {
        return false;
    }
    return proxyITorqueControl->setRefTorques(t);
}

bool PassThroughControlBoard::setRefTorque(int j, double t)
{
    if (!proxyITorqueControl)
    {
        return false;
    }
    return proxyITorqueControl->setRefTorque(j, t);
}

bool PassThroughControlBoard::getTorque(int j, double* t)
{
    if (!proxyITorqueControl)
    {
        return false;
    }
    return proxyITorqueControl->getTorque(j, t);
}

bool PassThroughControlBoard::getTorques(double* t)
{
    if (!proxyITorqueControl)
    {
        return false;
    }
    return proxyITorqueControl->getTorques(t);
}

bool PassThroughControlBoard::getTorqueRange(int j, double* min, double* max)
{
    if (!proxyITorqueControl)
    {
        return false;
    }
    return proxyITorqueControl->getTorqueRange(j, min, max);
}

bool PassThroughControlBoard::getTorqueRanges(double* min, double* max)
{
    if (!proxyITorqueControl)
    {
        return false;
    }
    return proxyITorqueControl->getTorqueRanges(min, max);
}

// OPEN LOOP
bool PassThroughControlBoard::getNumberOfMotors(int* n)
{
    if (!proxyIOpenLoopControl)
    {
        return false;
    }
    return proxyIOpenLoopControl->getNumberOfMotors(n);
}

bool PassThroughControlBoard::setRefDutyCycle(int m, double ref)
{
    if (!proxyIOpenLoopControl)
    {
        return false;
    }
    return proxyIOpenLoopControl->setRefDutyCycle(m, ref);
}

bool PassThroughControlBoard::setRefDutyCycles(const double* refs)
{
    if (!proxyIOpenLoopControl)
    {
        return false;
    }
    return proxyIOpenLoopControl->setRefDutyCycles(refs);
}

bool PassThroughControlBoard::getRefDutyCycle(int m, double* ref)
{
    if (!proxyIOpenLoopControl)
    {
        return false;
    }
    return proxyIOpenLoopControl->getRefDutyCycle(m, ref);
}

bool PassThroughControlBoard::getRefDutyCycles(double* refs)
{
    if (!proxyIOpenLoopControl)
    {
        return false;
    }
    return proxyIOpenLoopControl->getRefDutyCycles(refs);
}

bool PassThroughControlBoard::getDutyCycle(int m, double* val)
{
    if (!proxyIOpenLoopControl)
    {
        return false;
    }
    return proxyIOpenLoopControl->getDutyCycle(m, val);
}

bool PassThroughControlBoard::getDutyCycles(double* vals)
{
    if (!proxyIOpenLoopControl)
    {
        return false;
    }
    return proxyIOpenLoopControl->getDutyCycles(vals);
}

// CURRENT CONTROL
bool PassThroughControlBoard::getCurrent(int m, double* curr)
{
    // std::cerr << "Pass Through GetCurrent" << std::endl;
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->getCurrent(m, curr);
}

bool PassThroughControlBoard::getCurrents(double* currs)
{
    // std::cerr << "Pass Through GetCurrents" << std::endl;
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->getCurrents(currs);
}

bool PassThroughControlBoard::getCurrentRange(int m, double* min, double* max)
{
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->getCurrentRange(m, min, max);
}

bool PassThroughControlBoard::getCurrentRanges(double* min, double* max)
{
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->getCurrentRanges(min, max);
}

bool PassThroughControlBoard::getGearboxRatio(int m, double *val)
{
    if (!proxyIMotor)
    {
        return false;
    }
    return proxyIMotor->getGearboxRatio(m, val);
}

bool PassThroughControlBoard::getRefCurrents(double* currs)
{
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->getRefCurrents(currs);
}

bool PassThroughControlBoard::setRefCurrent(int m, double curr)
{
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->setRefCurrent(m, curr);
}

bool PassThroughControlBoard::setRefCurrents(const int n_motor,
                                             const int* motors,
                                             const double* currs)
{
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->setRefCurrents(n_motor, motors, currs);
}

bool PassThroughControlBoard::setRefCurrents(const double* currs)
{
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->setRefCurrents(currs);
}

bool PassThroughControlBoard::getRefCurrent(int m, double* curr)
{
    if (!proxyICurrentControl)
    {
        return false;
    }
    return proxyICurrentControl->getRefCurrent(m, curr);
}

// INTERACTION MODE
bool PassThroughControlBoard::getInteractionMode(int axis, yarp::dev::InteractionModeEnum* mode)
{
    if (!proxyIInteractionMode)
    {
        return false;
    }
    return proxyIInteractionMode->getInteractionMode(axis, mode);
}

bool PassThroughControlBoard::getInteractionModes(int n_joints,
                                                  int* joints,
                                                  yarp::dev::InteractionModeEnum* modes)
{
    if (!proxyIInteractionMode)
    {
        return false;
    }
    return proxyIInteractionMode->getInteractionModes(n_joints, joints, modes);
}

bool PassThroughControlBoard::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (!proxyIInteractionMode)
    {
        return false;
    }
    return proxyIInteractionMode->getInteractionModes(modes);
}

bool PassThroughControlBoard::setInteractionMode(int axis, yarp::dev::InteractionModeEnum mode)
{
    if (!proxyIInteractionMode)
    {
        return false;
    }
    return proxyIInteractionMode->setInteractionMode(axis, mode);
}

bool PassThroughControlBoard::setInteractionModes(int n_joints,
                                                  int* joints,
                                                  yarp::dev::InteractionModeEnum* modes)
{
    if (!proxyIInteractionMode)
    {
        return false;
    }
    return proxyIInteractionMode->setInteractionModes(n_joints, joints, modes);
}

bool PassThroughControlBoard::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    if (!proxyIInteractionMode)
    {
        return false;
    }
    return proxyIInteractionMode->setInteractionModes(modes);
}

// PID CONTROL INTERFACE METHODS
bool PassThroughControlBoard::setPid(const PidControlTypeEnum& pidtype, int j, const Pid& pid)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->setPid(pidtype, j, pid);
}

bool PassThroughControlBoard::setPids(const PidControlTypeEnum& pidtype, const Pid* pids)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->setPids(pidtype, pids);
}

bool PassThroughControlBoard::setPidReference(const PidControlTypeEnum& pidtype, int j, double ref)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->setPidReference(pidtype, j, ref);
}

bool PassThroughControlBoard::setPidReferences(const PidControlTypeEnum& pidtype,
                                               const double* refs)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->setPidReferences(pidtype, refs);
}

bool PassThroughControlBoard::setPidErrorLimit(const PidControlTypeEnum& pidtype,
                                               int j,
                                               double limit)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->setPidErrorLimit(pidtype, j, limit);
}

bool PassThroughControlBoard::setPidErrorLimits(const PidControlTypeEnum& pidtype,
                                                const double* limits)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->setPidErrorLimits(pidtype, limits);
}

bool PassThroughControlBoard::getPidError(const PidControlTypeEnum& pidtype, int j, double* err)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPidError(pidtype, j, err);
}

bool PassThroughControlBoard::getPidErrors(const PidControlTypeEnum& pidtype, double* errs)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPidErrors(pidtype, errs);
}

bool PassThroughControlBoard::getPidOutput(const PidControlTypeEnum& pidtype, int j, double* out)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPidOutput(pidtype, j, out);
}

bool PassThroughControlBoard::getPidOutputs(const PidControlTypeEnum& pidtype, double* outs)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPidOutputs(pidtype, outs);
}

bool PassThroughControlBoard::getPid(const PidControlTypeEnum& pidtype, int j, Pid* pid)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPid(pidtype, j, pid);
}

bool PassThroughControlBoard::getPids(const PidControlTypeEnum& pidtype, Pid* pids)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPids(pidtype, pids);
}

bool PassThroughControlBoard::getPidReference(const PidControlTypeEnum& pidtype, int j, double* ref)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPidReference(pidtype, j, ref);
}

bool PassThroughControlBoard::getPidReferences(const PidControlTypeEnum& pidtype, double* refs)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPidReferences(pidtype, refs);
}

bool PassThroughControlBoard::getPidErrorLimit(const PidControlTypeEnum& pidtype,
                                               int j,
                                               double* limit)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPidErrorLimit(pidtype, j, limit);
}

bool PassThroughControlBoard::getPidErrorLimits(const PidControlTypeEnum& pidtype, double* limits)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->getPidErrorLimits(pidtype, limits);
}

bool PassThroughControlBoard::resetPid(const PidControlTypeEnum& pidtype, int j)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->resetPid(pidtype, j);
}

bool PassThroughControlBoard::disablePid(const PidControlTypeEnum& pidtype, int j)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->disablePid(pidtype, j);
}

bool PassThroughControlBoard::enablePid(const PidControlTypeEnum& pidtype, int j)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->enablePid(pidtype, j);
}

bool PassThroughControlBoard::setPidOffset(const PidControlTypeEnum& pidtype, int j, double v)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->setPidOffset(pidtype, j, v);
}

bool PassThroughControlBoard::isPidEnabled(const PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    if (!proxyIPidControl)
    {
        return false;
    }
    return proxyIPidControl->isPidEnabled(pidtype, j, enabled);
}

// AXIS INFO
bool PassThroughControlBoard::getAxisName(int axis, std::string& name)
{
    if (!proxyIAxisInfo)
    {
        return false;
    }
    return proxyIAxisInfo->getAxisName(axis, name);
}
bool PassThroughControlBoard::getJointType(int axis, yarp::dev::JointTypeEnum& type)
{
    if (!proxyIAxisInfo)
    {
        return false;
    }
    return proxyIAxisInfo->getJointType(axis, type);
}

// AMPLIFIER CONTROL
bool PassThroughControlBoard::enableAmp(int j)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->enableAmp(j);
}

bool PassThroughControlBoard::disableAmp(int j)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->disableAmp(j);
}

bool PassThroughControlBoard::getAmpStatus(int* st)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->getAmpStatus(st);
}

bool PassThroughControlBoard::getAmpStatus(int j, int* v)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->getAmpStatus(j, v);
}

bool PassThroughControlBoard::getMaxCurrent(int j, double* v)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->getMaxCurrent(j, v);
}

bool PassThroughControlBoard::setMaxCurrent(int j, double v)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->setMaxCurrent(j, v);
}

bool PassThroughControlBoard::getNominalCurrent(int m, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->getNominalCurrent(m, val);
}

bool PassThroughControlBoard::setNominalCurrent(int m, const double val)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->setNominalCurrent(m, val);
}

bool PassThroughControlBoard::getPeakCurrent(int m, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->getPeakCurrent(m, val);
}

bool PassThroughControlBoard::setPeakCurrent(int m, const double val)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->setPeakCurrent(m, val);
}

bool PassThroughControlBoard::getPWM(int j, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->getPWM(j, val);
}

bool PassThroughControlBoard::getPWMLimit(int j, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->getPWMLimit(j, val);
}

bool PassThroughControlBoard::setPWMLimit(int j, const double val)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->setPWMLimit(j, val);
}

bool PassThroughControlBoard::getPowerSupplyVoltage(int j, double* val)
{
    if (!proxyIAmplifierControl)
    {
        return false;
    }
    return proxyIAmplifierControl->getPowerSupplyVoltage(j, val);
}

// CONTROL CALIBRATION
bool PassThroughControlBoard::calibrateAxisWithParams(
    int axis, unsigned int type, double p1, double p2, double p3)
{
    if (!proxyIControlCalibration)
    {
        return false;
    }
    return proxyIControlCalibration->calibrateAxisWithParams(axis, type, p1, p2, p3);
}

bool PassThroughControlBoard::setCalibrationParameters(int axis,
                                                       const CalibrationParameters& params)
{
    if (!proxyIControlCalibration)
    {
        return false;
    }
    return proxyIControlCalibration->setCalibrationParameters(axis, params);
}

bool PassThroughControlBoard::calibrationDone(int j)
{
    if (!proxyIControlCalibration)
    {
        return false;
    }
    return proxyIControlCalibration->calibrationDone(j);
}

bool PassThroughControlBoard::setCalibrator(ICalibrator* c)
{
    if (!proxyIControlCalibration)
    {
        return false;
    }
    return proxyIControlCalibration->setCalibrator(c);
}

bool PassThroughControlBoard::calibrateRobot()
{
    if (!proxyIControlCalibration)
    {
        return false;
    }
    return proxyIControlCalibration->calibrateRobot();
}

bool PassThroughControlBoard::park(bool wait)
{
    if (!proxyIControlCalibration)
    {
        return false;
    }
    return proxyIControlCalibration->park(wait);
}

bool PassThroughControlBoard::abortCalibration()
{
    if (!proxyIControlCalibration)
    {
        return false;
    }
    return proxyIControlCalibration->abortCalibration();
}

bool PassThroughControlBoard::abortPark()
{
    if (!proxyIControlCalibration)
    {
        return false;
    }
    return proxyIControlCalibration->abortPark();
}

// CONTROL LIMITS
bool PassThroughControlBoard::setLimits(int axis, double min, double max)
{
    if (!proxyIControlLimits)
    {
        return false;
    }
    return proxyIControlLimits->setLimits(axis, min, max);
}

bool PassThroughControlBoard::getLimits(int axis, double* min, double* max)
{
    if (!proxyIControlLimits)
    {
        return false;
    }
    return proxyIControlLimits->getLimits(axis, min, max);
}
bool PassThroughControlBoard::setVelLimits(int axis, double min, double max)
{
    if (!proxyIControlLimits)
    {
        return false;
    }
    return proxyIControlLimits->setVelLimits(axis, min, max);
}
bool PassThroughControlBoard::getVelLimits(int axis, double* min, double* max)
{
    if (!proxyIControlLimits)
    {
        return false;
    }
    return proxyIControlLimits->getVelLimits(axis, min, max);
}
