/**
 * @file JointTorqueControlDevice.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_JOINT_TORQUE_CONTROL_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_JOINT_TORQUE_CONTROL_DEVICE_H

#include <BipedalLocomotion/PassThroughControlBoard.h>

#include <iostream>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Core>

namespace BipedalLocomotion
{
class JointTorqueControlDevice;
}

/**
 * Coupling matrices
 *
 */
struct CouplingMatrices
{
    Eigen::MatrixXd fromJointTorquesToMotorTorques;
    Eigen::MatrixXd fromMotorTorquesToJointTorques;
    Eigen::MatrixXd fromJointVelocitiesToMotorVelocities;
    void reset(int NDOF)
    {
        fromJointTorquesToMotorTorques = Eigen::MatrixXd::Identity(NDOF, NDOF);
        fromMotorTorquesToJointTorques = Eigen::MatrixXd::Identity(NDOF, NDOF);
        fromJointVelocitiesToMotorVelocities = Eigen::MatrixXd::Identity(NDOF, NDOF);
    }
};

/**
 * Parameters for the control of torque through current and for mechanical friction compensation
 *
 */
struct MotorTorqueCurrentParameters
{
    double kt; ///< motor torque to current gain
    double kc; ///< coulomb friction parameter
    double kv; ///< viscous friction parameter
    double kfc; ///< friction compensation weight parameter
    double max_curr;

    void reset()
    {
        kt = kc = kv = kfc = max_curr = 0.0;
    }
};

class BipedalLocomotion::JointTorqueControlDevice
    : public BipedalLocomotion::PassThroughControlBoard,
      public yarp::os::PeriodicThread
{
private:
    yarp::os::Property PropertyConfigOptions;
    int axes;
    std::vector<MotorTorqueCurrentParameters> motorTorqueCurrentParameters;
    yarp::sig::Vector desiredJointTorques;
    yarp::sig::Vector desiredMotorCurrents;
    yarp::sig::Vector measuredJointVelocities;
    yarp::sig::Vector measuredMotorVelocities;
    yarp::sig::Vector measuredJointTorques;

    CouplingMatrices couplingMatrices;

    bool openCalledCorrectly{false};

    // HIJACKING CONTROL
    /**
     *  vector of getAxes() size.
     *  For each axis contains true if we are hijacking the torque control
     *  for this joint, or false otherwise.
     */
    std::vector<bool> hijackingTorqueControl;
    std::vector<int> hijackedMotors;
    void startHijackingTorqueControlIfNecessary(int j);
    void stopHijackingTorqueControlIfNecessary(int j);
    bool isHijackingTorqueControl(int j);

    void computeDesiredCurrents();
    void readStatus();
    bool loadGains(yarp::os::Searchable& config);

    /**
     * Load the coupling matrices from the group whose name
     *      is specified in group_name
     *
     *
     */
    bool loadCouplingMatrix(yarp::os::Searchable& config,
                            CouplingMatrices& coupling_matrices,
                            std::string group_name);

    std::mutex globalMutex; ///< mutex protecting control variables & proxy interface methods

    // Method that actually executes one control loop
    double timeOfLastControlLoop{-1.0};
    void controlLoop();

public:
    // CONSTRUCTOR/DESTRUCTOR
    JointTorqueControlDevice();
    ~JointTorqueControlDevice();

    // DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    // IMULTIPLEWRAPPER
    virtual bool attachAll(const yarp::dev::PolyDriverList& p);
    virtual bool detachAll();

    // CONTROL MODE
    virtual bool getControlMode(int j, int* mode);
    virtual bool getControlModes(int* modes);

    // CONTROL MODE
    virtual bool getControlModes(const int n_joint, const int* joints, int* modes);
    virtual bool setControlMode(const int j, const int mode);
    virtual bool setControlModes(int* modes);
    virtual bool setControlModes(const int n_joint, const int* joints, int* modes);

    // TORQUE CONTROL
    virtual bool getRefTorques(double* t);
    virtual bool getRefTorque(int j, double* t);
    virtual bool setRefTorques(const int n_joints, const int* joints, const double* trqs);
    virtual bool setRefTorques(const double* t);
    virtual bool setRefTorque(int j, double t);

    // CONTROL THREAD
    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
};

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_JOINT_TORQUE_CONTROL_DEVICE_H
