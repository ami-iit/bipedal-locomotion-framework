/**
 * @file JointTorqueControlDevice.cpp
 * @authors Ines Sorrentino
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/JointTorqueControlDevice.h>
#include <BipedalLocomotion/System/Clock.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <mutex>

#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#include <Eigen/Core>
#include <Eigen/LU>

#define M_PI 3.14159265358979323846 /* pi */

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace BipedalLocomotion;

template <class T> bool contains(std::vector<T>& v, T& x)
{
    return !(std::find(v.begin(), v.end(), x) == v.end());
}

template <class T> int findAndReturnIndex(std::vector<T>& v, T& x)
{
    typename std::vector<T>::iterator it = std::find(v.begin(), v.end(), x);
    if (it == v.end())
    {
        return -1;
    } else
    {
        return std::distance(v.begin(), it);
    }
}

// INLINE OPERATIONS
/** Saturate the specified value between the specified bounds. */
inline double saturation(const double x, const double xMax, const double xMin)
{
    return x > xMax ? xMax : (x < xMin ? xMin : x);
}

inline double sign(double x)
{
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

inline Eigen::Map<Eigen::MatrixXd> toEigen(yarp::sig::Vector& vec)
{
    return Eigen::Map<Eigen::MatrixXd>(vec.data(), 1, vec.size());
}

inline Eigen::Map<Eigen::VectorXd> toEigenVector(yarp::sig::Vector& vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
}

/** Check if the vector, corresponding at the parameter with name=name,
 * exists and has the correct lenght (expected_vec_size). */
bool checkVectorExistInConfiguration(yarp::os::Bottle& bot,
                                     const std::string& name,
                                     const int expected_vec_size)
{
    return (bot.check(name) && bot.find(name).isList()
            && bot.find(name).asList()->size() == expected_vec_size);
}

// JOINT TORQUE CONTROL IMPLEMENTATION

// CONSTRUCTOR/DESTRUCTOR
JointTorqueControlDevice::JointTorqueControlDevice()
    : PassThroughControlBoard()
    , PeriodicThread(10)
{
}

JointTorqueControlDevice::~JointTorqueControlDevice()
{
}

bool JointTorqueControlDevice::setKpJtcvc(const std::string& jointName, const double kp)
{
    size_t index = 0;

    do
    {
        if (m_axisNames[index] == jointName)
        {
            std::lock_guard<std::mutex> lock(mutexTorqueControlParam_);
            motorTorqueCurrentParameters[index].kp = kp;

            log()->info("Request for kp des = {}", kp);
            log()->info("Setting value kp = {}", motorTorqueCurrentParameters[index].kp);

            return true;
        }

        index++;
    } while (index < m_axisNames.size());

    return false;
}

double JointTorqueControlDevice::getKpJtcvc(const std::string& jointName)
{
    size_t index = 0;

    double kp = -1;

    do
    {
        if (m_axisNames[index] == jointName)
        {
            std::lock_guard<std::mutex> lock(mutexTorqueControlParam_);

            log()->info("kp value is {}", motorTorqueCurrentParameters[index].kp);

            return motorTorqueCurrentParameters[index].kp;
        }

        index++;
    } while (index < m_axisNames.size());

    return kp;
}

bool JointTorqueControlDevice::setKfcJtcvc(const std::string& jointName, const double kfc)
{
    size_t index = 0;

    do
    {
        if (m_axisNames[index] == jointName)
        {
            std::lock_guard<std::mutex> lock(mutexTorqueControlParam_);
            motorTorqueCurrentParameters[index].kfc = kfc;

            return true;
        }

        index++;
    } while (index < m_axisNames.size());

    return false;
}

double JointTorqueControlDevice::getKfcJtcvc(const std::string& jointName)
{
    size_t index = 0;

    double kfc = -1;

    do
    {
        if (m_axisNames[index] == jointName)
        {
            std::lock_guard<std::mutex> lock(mutexTorqueControlParam_);

            return motorTorqueCurrentParameters[index].kfc;
        }

        index++;
    } while (index < m_axisNames.size());

    return kfc;
}

bool JointTorqueControlDevice::setFrictionModel(const std::string& jointName,
                                                const std::string& model)
{
    size_t index = 0;

    do
    {
        if (m_axisNames[index] == jointName)
        {
            std::lock_guard<std::mutex> lock(mutexTorqueControlParam_);
            motorTorqueCurrentParameters[index].frictionModel = model;

            if (model == "FRICTION_PINN")
            {
                frictionEstimators[index]->resetEstimator();
            }

            return true;
        }

        index++;
    } while (index < m_axisNames.size());

    return false;
}

std::string JointTorqueControlDevice::getFrictionModel(const std::string& jointName)
{
    size_t index = 0;

    std::string model = "none";

    do
    {
        if (m_axisNames[index] == jointName)
        {
            std::lock_guard<std::mutex> lock(mutexTorqueControlParam_);

            return motorTorqueCurrentParameters[index].frictionModel;
        }

        index++;
    } while (index < m_axisNames.size());

    return model;
}

// HIJACKING CONTROL
void JointTorqueControlDevice::startHijackingTorqueControlIfNecessary(int j)
{
    if (!this->hijackingTorqueControl[j])
    {
        desiredJointTorques(j) = measuredJointTorques(j); // TODO check if it is better to
                                                          // initialize the desired joint torque
                                                          // considering the measured current

        if (motorTorqueCurrentParameters[j].frictionModel == "FRICTION_PINN")
        {
            frictionEstimators[j]->resetEstimator();

            this->readStatus();
            for (int i = 0; i < axes; i++)
            {
                m_initialDeltaMotorJointRadians[i]
                    = (measuredMotorPositions[i] - m_gearRatios[i] * measuredJointPositions[i])
                      * M_PI / 180.0;
            }
        }

        this->hijackingTorqueControl[j] = true;
        hijackedMotors.push_back(j);
    }
}

void JointTorqueControlDevice::stopHijackingTorqueControlIfNecessary(int j)
{
    if (this->hijackingTorqueControl[j])
    {
        if (motorTorqueCurrentParameters[j].frictionModel == "FRICTION_PINN")
        {
            frictionEstimators[j]->resetEstimator();
        }
        this->hijackingTorqueControl[j] = false;
    }
    hijackedMotors.erase(std::remove(hijackedMotors.begin(), hijackedMotors.end(), j),
                         hijackedMotors.end());
}

bool JointTorqueControlDevice::isHijackingTorqueControl(int j)
{
    return this->hijackingTorqueControl[j];
}

double JointTorqueControlDevice::computeFrictionTorque(int joint)
{
    double frictionTorque = 0.0;

    if (motorTorqueCurrentParameters[joint].frictionModel == "FRICTION_COULOMB_VISCOUS")
    {
        double velocityRadians = measuredJointVelocities[joint] * M_PI / 180.0;

        frictionTorque = coulombViscousParameters[joint].kc
                             * std::tanh(coulombViscousParameters[joint].ka * velocityRadians)
                         + coulombViscousParameters[joint].kv * velocityRadians;
    } else if (motorTorqueCurrentParameters[joint].frictionModel
               == "FRICTION_COULOMB_VISCOUS_STRIBECK")
    {
        double velocityRadians = measuredJointVelocities[joint] * M_PI / 180.0;

        double tauCoulomb
            = coulombViscousStribeckParameters[joint].kc
              * std::tanh(coulombViscousStribeckParameters[joint].ka * velocityRadians);
        double tauViscous = coulombViscousStribeckParameters[joint].kv * velocityRadians;
        double tauStribeck
            = (coulombViscousStribeckParameters[joint].ks
               - coulombViscousStribeckParameters[joint].kc)
              * std::exp(
                  -std::pow(std::abs(velocityRadians / coulombViscousStribeckParameters[joint].vs),
                            coulombViscousStribeckParameters[joint].alpha))
              * std::tanh(coulombViscousStribeckParameters[joint].ka * velocityRadians);

        frictionTorque = tauCoulomb + tauViscous + tauStribeck;
    } else if (motorTorqueCurrentParameters[joint].frictionModel == "FRICTION_PINN")
    {
        m_tempJointPosRad = measuredJointPositions[joint] * M_PI / 180.0;
        m_tempJointPosMotorSideRad = m_gearRatios[joint] * m_tempJointPosRad;
        m_motorPositionsRadians[joint] = measuredMotorPositions[joint] * M_PI / 180.0;
        m_motorPositionCorrected[joint]
            = m_motorPositionsRadians[joint] - m_initialDeltaMotorJointRadians[joint];
        m_jointVelRadSec = measuredJointVelocities[joint] * M_PI / 180.0;
        m_motorPositionError[joint] = m_tempJointPosMotorSideRad - m_motorPositionCorrected[joint];

        // Test network with inputs position error motor side, joint velocity
        if (!frictionEstimators[joint]->estimate(m_motorPositionError[joint],
                                                 m_jointVelRadSec,
                                                 frictionTorque))
        {
            frictionTorque = 0.0;
        }
    }

    return frictionTorque;
}

void JointTorqueControlDevice::computeDesiredCurrents()
{
    toEigenVector(desiredJointTorques)
        = couplingMatrices.fromJointTorquesToMotorTorques * toEigenVector(desiredJointTorques);

    estimatedFrictionTorques.zero();

    for (int j = 0; j < this->axes; j++)
    {
        std::lock_guard<std::mutex> lock(mutexTorqueControlParam_);

        if (this->hijackingTorqueControl[j])
        {
            if (motorTorqueCurrentParameters[j].kfc > 0.0)
            {
                estimatedFrictionTorques[j] = computeFrictionTorque(j);
            }

            desiredMotorCurrents[j]
                = (desiredJointTorques[j]
                   + motorTorqueCurrentParameters[j].kp
                         * (desiredJointTorques[j] - measuredJointTorques[j])
                   + motorTorqueCurrentParameters[j].kfc * estimatedFrictionTorques[j])
                  / motorTorqueCurrentParameters[j].kt;

            desiredMotorCurrents[j] = desiredMotorCurrents[j] / m_gearRatios[j];

            desiredMotorCurrents[j] = saturation(desiredMotorCurrents[j],
                                                 motorTorqueCurrentParameters[j].maxCurr,
                                                 -motorTorqueCurrentParameters[j].maxCurr);

            {
                std::lock_guard<std::mutex> lockOutput(m_status.mutex);
                m_status.m_frictionLogging[j] = estimatedFrictionTorques[j];
                m_status.m_torqueLogging[j] = desiredJointTorques[j];
                m_status.m_currentLogging[j] = desiredMotorCurrents[j];
            }
        }
    }

    bool isNaNOrInf = false;
    for (int j = 0; j < this->axes; j++)
    {
        if (this->hijackingTorqueControl[j])
        {
            if (isnan(desiredMotorCurrents[j]) || isinf(desiredMotorCurrents[j]))
            { // this is not std c++. Supported in C99 and C++11
                desiredMotorCurrents[j] = 0;
                isNaNOrInf = true;
            }
        }
    }
    if (isNaNOrInf)
    {
        yWarning("Inf or NaN found in control output");
    }
}

void JointTorqueControlDevice::readStatus()
{
    auto logPrefix = "[JointTorqueControlDevice::readStatus]";

    if (!this->PassThroughControlBoard::getEncoderSpeeds(measuredJointVelocities.data()))
    {
        log()->error("{} Failed to get Motor encoders speed", logPrefix);
    }
    if (!this->PassThroughControlBoard::getMotorEncoderSpeeds(measuredMotorVelocities.data()))
    {
        // In case the motor speed can not be directly measured, it tries to estimate it from joint
        // velocity if available
        if (!this->PassThroughControlBoard::getEncoderSpeeds(measuredJointVelocities.data()))
        {
            log()->error("{} Failed to get Motor encoders speed", logPrefix);
        } else
        {
            toEigenVector(measuredMotorVelocities)
                = couplingMatrices.fromJointVelocitiesToMotorVelocities
                  * toEigenVector(measuredJointVelocities);
        }
    }
    if (!this->PassThroughControlBoard::getTorques(measuredJointTorques.data()))
    {
        log()->error("{} Failed to get joint torque", logPrefix);
    }
    if (!this->PassThroughControlBoard::getEncoders(measuredJointPositions.data()))
    {
        log()->error("{} Failed to get joint position", logPrefix);
    }
    if (!this->PassThroughControlBoard::getMotorEncoders(measuredMotorPositions.data()))
    {
        log()->error("{} Failed to get motor position", logPrefix);
    }
}

bool JointTorqueControlDevice::loadCouplingMatrix(Searchable& config,
                                                  CouplingMatrices& coupling_matrix,
                                                  std::string group_name)
{
    auto logPrefix = "[JointTorqueControlDevice::loadCouplingMatrix]";

    if (!config.check(group_name))
    {
        coupling_matrix.reset(this->axes);
        log()->warn("{} COUPLING MATRIX option not found in configuration file", logPrefix);
        return true;
    } else
    {
        yInfo("COUPLING MATRIX found");
        yarp::os::Bottle couplings_bot = config.findGroup(group_name);

        if (!checkVectorExistInConfiguration(couplings_bot, "axesNames", this->axes))
        {
            log()->error("{} Error in loading axesNames parameter", logPrefix);
            return false;
        }
        if (!checkVectorExistInConfiguration(couplings_bot, "motorNames", this->axes))
        {
            log()->error("{} Error in loading motorNames parameter", logPrefix);
            return false;
        }
        std::vector<std::string> motorNames(this->axes);
        std::vector<std::string> axesNames(this->axes);
        for (int j = 0; j < this->axes; j++)
        {
            motorNames[j] = couplings_bot.find("motorNames").asList()->get(j).asString();
            axesNames[j] = couplings_bot.find("axesNames").asList()->get(j).asString();
        }
        for (int axis_id = 0; axis_id < (int)this->axes; axis_id++)
        {
            // search if coupling information is provided for each axis, if not return false
            std::string axis_name = axesNames[axis_id];
            if (!couplings_bot.check(axis_name) || !(couplings_bot.find(axis_name).isList()))
            {
                log()->error("{} {} group found, but no coupling found for joint {}, exiting",
                             logPrefix,
                             group_name,
                             axis_name);
                return false;
            }

            // Check coupling configuration is well formed
            yarp::os::Bottle* axis_coupling_bot = couplings_bot.find(axis_name).asList();
            for (int coupled_motor = 0; coupled_motor < axis_coupling_bot->size(); coupled_motor++)
            {
                if (!(axis_coupling_bot->get(coupled_motor).isList())
                    || !(axis_coupling_bot->get(coupled_motor).asList()->size() == 2)
                    || !(axis_coupling_bot->get(coupled_motor).asList()->get(0).isFloat64())
                    || !(axis_coupling_bot->get(coupled_motor).asList()->get(1).isString()))
                {
                    log()->error("{} {} group found, but coupling for axis {} is malformed",
                             logPrefix,
                             group_name,
                             axis_name);
                    return false;
                }

                std::string motorName
                    = axis_coupling_bot->get(coupled_motor).asList()->get(1).asString().c_str();
                if (!contains(motorNames, motorName))
                {
                    log()->error("{} {} group found, but motor name {} is not part of the motor list",
                             logPrefix,
                             group_name,
                             motorName);
                    return false;
                }
            }

            // Zero the row of the selected axis in velocity coupling matrix
            coupling_matrix.fromJointVelocitiesToMotorVelocities.row(axis_id).setZero();

            // Get non-zero coefficient of the coupling matrices
            for (int coupled_motor = 0; coupled_motor < axis_coupling_bot->size(); coupled_motor++)
            {
                double coeff = axis_coupling_bot->get(coupled_motor).asList()->get(0).asFloat64();
                std::string motorName
                    = axis_coupling_bot->get(coupled_motor).asList()->get(1).asString();
                int motorIndex = findAndReturnIndex(motorNames, motorName);

                if (motorIndex == -1)
                {
                    return false;
                }
                coupling_matrix.fromJointVelocitiesToMotorVelocities(axis_id, motorIndex) = coeff;
            }
        }

        log()->error("{} LoadCouplingMatrix DEBUG: loaded kinematic coupling matrix from group {}", logPrefix, group_name);
        log()->error("{}", coupling_matrix.fromJointVelocitiesToMotorVelocities);

        coupling_matrix.fromJointTorquesToMotorTorques
            = coupling_matrix.fromJointVelocitiesToMotorVelocities.transpose();
        coupling_matrix.fromMotorTorquesToJointTorques
            = coupling_matrix.fromJointTorquesToMotorTorques.inverse();
        coupling_matrix.fromJointVelocitiesToMotorVelocities
            = coupling_matrix.fromJointVelocitiesToMotorVelocities.inverse();
        // Compute the torque coupling matrix

        return true;
    }
}

bool JointTorqueControlDevice::loadFrictionParams(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto logPrefix = "[JointTorqueControlDevice::loadFrictionModel]";

    auto ptr = paramHandler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler", logPrefix);
        return false;
    }

    auto frictionGroup = ptr->getGroup("FRICTION_COULOMB_VISCOUS").lock();
    if (frictionGroup == nullptr)
    {
        log()->info("{} Group `FRICTION_COULOMB_VISCOUS` not found in configuration.", logPrefix);
    } else
    {
        Eigen::VectorXd kc;
        if (!frictionGroup->getParameter("kc", kc))
        {
            log()->error("{} Parameter `kc` not found", logPrefix);
            return false;
        }
        Eigen::VectorXd kv;
        if (!frictionGroup->getParameter("kv", kv))
        {
            log()->error("{} Parameter `kv` not found", logPrefix);
            return false;
        }
        Eigen::VectorXd ka;
        if (!frictionGroup->getParameter("ka", ka))
        {
            log()->error("{} Parameter `ka` not found", logPrefix);
            return false;
        }

        for (int i = 0; i < kc.size(); i++)
        {
            coulombViscousParameters[i].kc = kc(i);
            coulombViscousParameters[i].kv = kv(i);
            coulombViscousParameters[i].ka = ka(i);
        }
    }

    frictionGroup = ptr->getGroup("FRICTION_COULOMB_VISCOUS_STRIBECK").lock();
    if (frictionGroup == nullptr)
    {
        log()->info("{} Group `FRICTION_COULOMB_VISCOUS_STRIBECK` not found in configuration.",
                    logPrefix);
    } else
    {
        Eigen::VectorXd kc;
        if (!frictionGroup->getParameter("kc", kc))
        {
            log()->error("{} Parameter `kc` not found", logPrefix);
            return false;
        }
        Eigen::VectorXd kv;
        if (!frictionGroup->getParameter("kv", kv))
        {
            log()->error("{} Parameter `kv` not found", logPrefix);
            return false;
        }
        Eigen::VectorXd vs;
        if (!frictionGroup->getParameter("vs", vs))
        {
            log()->error("{} Parameter `vs` not found", logPrefix);
            return false;
        }
        Eigen::VectorXd ka;
        if (!frictionGroup->getParameter("ka", ka))
        {
            log()->error("{} Parameter `ka` not found", logPrefix);
            return false;
        }
        Eigen::VectorXd ks;
        if (!frictionGroup->getParameter("ks", ks))
        {
            log()->error("{} Parameter `ks` not found", logPrefix);
            return false;
        }
        Eigen::VectorXd alpha;
        if (!frictionGroup->getParameter("alpha", alpha))
        {
            log()->error("{} Parameter `alpha` not found", logPrefix);
            return false;
        }

        for (int i = 0; i < kc.size(); i++)
        {
            coulombViscousStribeckParameters[i].kc = kc(i);
            coulombViscousStribeckParameters[i].kv = kv(i);
            coulombViscousStribeckParameters[i].vs = vs(i);
            coulombViscousStribeckParameters[i].ka = ka(i);
            coulombViscousStribeckParameters[i].ks = ks(i);
            coulombViscousStribeckParameters[i].alpha = alpha(i);
        }
    }

    frictionGroup = ptr->getGroup("FRICTION_PINN").lock();
    if (frictionGroup == nullptr)
    {
        log()->info("{} Group `FRICTION_PINN` not found in configuration.", logPrefix);
    } else
    {
        std::vector<std::string> models;
        if (!frictionGroup->getParameter("model", models))
        {
            log()->error("{} Parameter `model` not found", logPrefix);
            return false;
        }
        Eigen::VectorXi historySize;
        if (!frictionGroup->getParameter("history_size", historySize))
        {
            log()->error("{} Parameter `history_size` not found", logPrefix);
            return false;
        }
        Eigen::VectorXi inputNumber;
        if (!frictionGroup->getParameter("number_of_inputs", inputNumber))
        {
            log()->error("{} Parameter `number_of_inputs` not found", logPrefix);
            return false;
        }
        int threads;
        if (!frictionGroup->getParameter("thread_number", threads))
        {
            log()->error("{} Parameter `thread_number` not found", logPrefix);
            return false;
        }

        for (int i = 0; i < models.size(); i++)
        {
            pinnParameters[i].modelPath = models[i];
            pinnParameters[i].historyLength = historySize[i];
            pinnParameters[i].inputNumber = inputNumber[i];
            pinnParameters[i].threadNumber = threads;
        }
    }

    return true;
}

// DEVICE DRIVER
bool JointTorqueControlDevice::open(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[JointTorqueControlDevice::open]";

    // Call open of PassThroughControlBoard
    bool ret = PassThroughControlBoard::open(config);

    if (!ret)
    {
        return false;
    }

    PropertyConfigOptions.fromString(config.toString().c_str());
    openCalledCorrectly = ret;

    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);

    int rate = 10;
    if (!params->getParameter("period", rate))
    {
        log()->info("{} Parameter `period` not found", logPrefix);
    }
    this->setPeriod(rate * 0.001);

    auto torqueGroup = params->getGroup("TORQUE_CURRENT_PARAMS").lock();
    if (torqueGroup == nullptr)
    {
        log()->error("{} Group `TORQUE_CURRENT_PARAMS` not found in configuration.", logPrefix);
        return false;
    }

    Eigen::VectorXd kt;
    if (!torqueGroup->getParameter("kt", kt))
    {
        log()->error("{} Parameter `kt` not found", logPrefix);
        return false;
    }
    Eigen::VectorXd kfc;
    if (!torqueGroup->getParameter("kfc", kfc))
    {
        log()->error("{} Parameter `kfc` not found", logPrefix);
        return false;
    }
    Eigen::VectorXd kp;
    if (!torqueGroup->getParameter("kp", kp))
    {
        log()->error("{} Parameter `kp` not found", logPrefix);
        return false;
    }
    Eigen::VectorXd maxCurr;
    if (!torqueGroup->getParameter("max_curr", maxCurr))
    {
        log()->error("{} Parameter `max_curr` not found", logPrefix);
        return false;
    }

    std::vector<std::string> frictionModels;
    if (!torqueGroup->getParameter("friction_model", frictionModels))
    {
        log()->error("{} Parameter `friction_model` not found", logPrefix);
        return false;
    }

    motorTorqueCurrentParameters.resize(kt.size());
    pinnParameters.resize(kt.size());
    coulombViscousParameters.resize(kt.size());
    coulombViscousStribeckParameters.resize(kt.size());
    frictionEstimators.resize(kt.size());
    for (int i = 0; i < kt.size(); i++)
    {
        motorTorqueCurrentParameters[i].kt = kt[i];
        motorTorqueCurrentParameters[i].kfc = kfc[i];
        motorTorqueCurrentParameters[i].kp = kp[i];
        motorTorqueCurrentParameters[i].maxCurr = maxCurr[i];
        motorTorqueCurrentParameters[i].frictionModel = frictionModels[i];
    }

    if (!this->loadFrictionParams(params))
    {
        log()->error("{} Failed to load friction model", logPrefix);
        return false;
    }

    int threadNumber = 1;

    for (int i = 0; i < kt.size(); i++)
    {
        if (motorTorqueCurrentParameters[i].frictionModel == "FRICTION_PINN"
            && motorTorqueCurrentParameters[i].kfc > 0.0)
        {
            frictionEstimators[i] = std::make_unique<PINNFrictionEstimator>();

            if (!frictionEstimators[i]->initialize(pinnParameters[i].modelPath,
                                                   threadNumber,
                                                   threadNumber))
            {
                log()->error("{} Failed to initialize friction estimator", logPrefix);
                return false;
            }
        }
    }

    if (!m_vectorsCollectionServer.initialize(params))
    {
        log()->error("{} Unable to configure the server.", logPrefix);
        return false;
    }

    std::vector<std::string> joint_list;
    if (!params->getParameter("joint_list", joint_list))
    {
        log()->info("{} Parameter `joint_list` not found", logPrefix);
    }

    std::string remote;
    if (!params->getParameter("remote", remote))
    {
        log()->error("{} Parameter `remote` not found", logPrefix);
        return false;
    }

    std::string rpcPortName = remote + "/commands/rpc:i";
    this->yarp().attachAsServer(this->m_rpcPort);
    if (!m_rpcPort.open(rpcPortName))
    {
        log()->error("{} Could not open", logPrefix);
        return false;
    }

    m_vectorsCollectionServer.populateMetadata("motor_currents::desired", joint_list);
    m_vectorsCollectionServer.populateMetadata("joint_torques::desired", joint_list);
    m_vectorsCollectionServer.populateMetadata("friction_torques::estimated", joint_list);
    m_vectorsCollectionServer.finalizeMetadata();

    // run the thread
    m_torqueControlIsRunning = false;
    m_publishEstimationThread = std::thread([this] { this->publishStatus(); });

    return ret;
}

void JointTorqueControlDevice::publishStatus()
{
    constexpr auto logPrefix = "[JointTorqueControlDevice::publishStatus]";

    auto time = BipedalLocomotion::clock().now();
    auto oldTime = time;
    auto wakeUpTime = time;
    const auto publishOutputPeriod = std::chrono::duration<double>(0.01);

    m_torqueControlIsRunning = true;

    while (m_torqueControlIsRunning)
    {
        m_vectorsCollectionServer.prepareData(); // required to prepare the data to be sent
        m_vectorsCollectionServer.clearData(); // optional see the documentation

        // detect if a clock has been reset
        oldTime = time;
        time = BipedalLocomotion::clock().now();
        // if the current time is lower than old time, the timer has been reset.
        if ((time - oldTime).count() < 1e-12)
        {
            wakeUpTime = time;
        }
        wakeUpTime = std::chrono::duration_cast<std::chrono::nanoseconds>(wakeUpTime
                                                                          + publishOutputPeriod);

        {
            std::lock_guard<std::mutex> lockOutput(m_status.mutex);
            m_vectorsCollectionServer.populateData("motor_currents::desired",
                                                   m_status.m_currentLogging);
            m_vectorsCollectionServer.populateData("joint_torques::desired",
                                                   m_status.m_torqueLogging);
            m_vectorsCollectionServer.populateData("friction_torques::estimated",
                                                   m_status.m_frictionLogging);
            m_vectorsCollectionServer.sendData();
        }

        // release the CPU
        BipedalLocomotion::clock().yield();

        // sleep
        BipedalLocomotion::clock().sleepUntil(wakeUpTime);
    }
}

bool JointTorqueControlDevice::attachAll(const PolyDriverList& p)
{
    if (!openCalledCorrectly)
    {
        return false;
    }

    // Call attachAll of PassThroughControlBoard
    bool ret = PassThroughControlBoard::attachAll(p);

    // number of axis is read and stored
    if (ret)
    {
        ret = ret && this->getAxes(&axes);
    }

    // initialization of std::vector
    if (ret)
    {
        hijackedMotors.clear();
        hijackingTorqueControl.assign(axes, false);
        desiredMotorCurrents.resize(axes);
        desiredJointTorques.resize(axes);
        measuredJointVelocities.resize(axes, 0.0);
        measuredMotorVelocities.resize(axes, 0.0);
        measuredJointTorques.resize(axes, 0.0);
        measuredJointPositions.resize(axes, 0.0);
        measuredMotorPositions.resize(axes, 0.0);
        estimatedFrictionTorques.resize(axes, 0.0);
        m_gearRatios.resize(axes, 1);
        m_axisNames.resize(axes);
        m_initialDeltaMotorJointRadians.resize(axes, 1);
        m_motorPositionError.resize(axes, 1);
        m_motorPositionCorrected.resize(axes, 1);
        m_motorPositionsRadians.resize(axes, 1);
        m_status.m_torqueLogging.resize(axes, 1);
        m_status.m_frictionLogging.resize(axes, 1);
        m_status.m_currentLogging.resize(axes, 1);
    }

    // Load coupling matrices
    if (ret)
    {
        couplingMatrices.reset(this->axes);
        ret = ret
              && this->loadCouplingMatrix(PropertyConfigOptions,
                                          couplingMatrices,
                                          "FROM_MOTORS_TO_JOINTS_KINEMATIC_COUPLINGS");
    }

    // Set control thread (set to 10 if not found)
    int rate = PropertyConfigOptions
                   .check("period",
                          yarp::os::Value(10),
                          "update period of the current control thread (ms)")
                   .asInt32();
    this->setPeriod(rate * 0.001);

    if (ret)
    {
        if (!this->start())
        {
            log()->error("Failed to start device");
            ret = false;
        }
    }

    // Get gearratio of motors
    for (int i = 0; i < axes; i++)
    {
        double gearRatio;
        if (!this->getGearboxRatio(i, &gearRatio))
        {
            yError("Failed to get gear ratio for joint %d", i);
            ret = false;
        }
        m_gearRatios[i] = gearRatio;
    }

    for (int i = 0; i < axes; i++)
    {
        std::string jointName;
        if (!this->getAxisName(i, jointName))
        {
            yError("Failed to get the axis name for axis %d", i);
            ret = false;
        }

        m_axisNames[i] = jointName;
    }

    return true;
}

bool JointTorqueControlDevice::detachAll()
{
    m_torqueControlIsRunning = false;
    if (m_publishEstimationThread.joinable())
    {
        m_publishEstimationThread.join();
    }

    m_rpcPort.close();
    this->PeriodicThread::stop();
    openCalledCorrectly = false;
    return PassThroughControlBoard::detachAll();
}

bool JointTorqueControlDevice::close()
{
    return true;
}

// CONTROL MODE
bool JointTorqueControlDevice::getControlMode(int j, int* mode)
{
    if (!proxyIControlMode)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(globalMutex);

    bool ret = proxyIControlMode->getControlMode(j, mode);
    if (isHijackingTorqueControl(j) && (*mode) == VOCAB_CM_CURRENT)
    {
        *mode = VOCAB_CM_TORQUE;
    }
    return ret;
}

bool JointTorqueControlDevice::getControlModes(int* modes)
{
    if (!proxyIControlMode)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(globalMutex);

    bool ret = proxyIControlMode->getControlModes(modes);
    for (int j = 0; j < this->axes; j++)
    {
        if (this->isHijackingTorqueControl(j) && modes[j] == VOCAB_CM_CURRENT)
        {
            modes[j] = VOCAB_CM_TORQUE;
        }
    }
    return ret;
}

// CONTROL MODE
bool JointTorqueControlDevice::getControlModes(const int n_joint, const int* joints, int* modes)
{
    if (!proxyIControlMode)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(globalMutex);

    bool ret = proxyIControlMode->getControlModes(n_joint, joints, modes);
    for (int i = 0; i < n_joint; i++)
    {
        int j = joints[i];
        if (this->isHijackingTorqueControl(j) && modes[i] == VOCAB_CM_CURRENT)
        {
            modes[i] = VOCAB_CM_TORQUE;
        }
    }
    return ret;
}

bool JointTorqueControlDevice::setControlMode(const int j, const int mode)
{
    if (!proxyIControlMode)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(globalMutex);

    int new_mode = mode;
    if (new_mode == VOCAB_CM_TORQUE)
    {
        this->startHijackingTorqueControlIfNecessary(j);
        new_mode = VOCAB_CM_CURRENT;
    } else
    {
        this->stopHijackingTorqueControlIfNecessary(j);
    }

    return proxyIControlMode->setControlMode(j, new_mode);
}

bool JointTorqueControlDevice::setControlModes(int* modes)
{
    if (!proxyIControlMode)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(globalMutex);

    for (int j = 0; j < this->axes; j++)
    {
        if (modes[j] == VOCAB_CM_TORQUE)
        {
            this->startHijackingTorqueControlIfNecessary(j);
            modes[j] = VOCAB_CM_CURRENT;
        } else
        {
            this->stopHijackingTorqueControlIfNecessary(j);
        }
    }

    return proxyIControlMode->setControlModes(modes);
}

bool JointTorqueControlDevice::setControlModes(const int n_joint, const int* joints, int* modes)
{
    if (!proxyIControlMode)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(globalMutex);

    for (int i = 0; i < n_joint; i++)
    {
        int j = joints[i];
        if (modes[i] == VOCAB_CM_TORQUE)
        {
            this->startHijackingTorqueControlIfNecessary(j);
            modes[i] = VOCAB_CM_CURRENT;
        } else
        {
            this->stopHijackingTorqueControlIfNecessary(j);
        }
    }

    return proxyIControlMode->setControlModes(n_joint, joints, modes);
}

bool JointTorqueControlDevice::setRefTorques(const double* trqs)
{
    {
        std::lock_guard<std::mutex>(this->globalMutex);
        memcpy(desiredJointTorques.data(), trqs, this->axes * sizeof(double));

        this->controlLoop();
    }

    return true;
}

bool JointTorqueControlDevice::setRefTorques(const int n_joints,
                                             const int* joints,
                                             const double* trqs)
{
    {
        std::lock_guard<std::mutex>(this->globalMutex);
        for (int i = 0; i < n_joints; i++)
        {
            desiredJointTorques[joints[i]] = trqs[i];
        }

        this->controlLoop();
    }

    return true;
}

bool JointTorqueControlDevice::setRefTorque(int j, double trq)
{
    std::lock_guard<std::mutex>(this->globalMutex);
    desiredJointTorques[j] = trq;

    // If the single joint version is used, we do not call the updateLoop, because probably this
    // means that multiple joints are updated and this will generate a huge number of messages sent
    // to the low level

    return true;
}

bool JointTorqueControlDevice::getRefTorques(double* trqs)
{
    std::lock_guard<std::mutex>(this->globalMutex);
    memcpy(trqs, desiredJointTorques.data(), this->axes * sizeof(double));
    return true;
}

bool JointTorqueControlDevice::getRefTorque(int j, double* trq)
{
    std::lock_guard<std::mutex>(this->globalMutex);
    *trq = desiredJointTorques[j];
    return true;
}

// HIJACKED CONTROL THREAD
bool JointTorqueControlDevice::threadInit()
{
    return true;
}

void JointTorqueControlDevice::threadRelease()
{
    std::lock_guard<std::mutex> guard(globalMutex);
}

void JointTorqueControlDevice::run()
{
    std::chrono::nanoseconds now = BipedalLocomotion::clock().now();

    std::lock_guard<std::mutex> lock(globalMutex);
    if (now.count() - timeOfLastControlLoop.count() >= this->getPeriod())
    {
        this->controlLoop();
    }

    double now_2 = yarp::os::Time::now();
    // std::cout << "Elapsed time: " << now_2 - now << std::endl;
}

void JointTorqueControlDevice::controlLoop()
{

    // Read status from the controlboard
    this->readStatus();

    // update desired  current
    this->computeDesiredCurrents();

    std::vector<double> desiredMotorCurrentsHijackedMotors;
    desiredMotorCurrentsHijackedMotors.clear();

    for (std::vector<int>::iterator it = hijackedMotors.begin(); it != hijackedMotors.end(); ++it)
    {
        desiredMotorCurrentsHijackedMotors.push_back(desiredMotorCurrents[*it]);
    }

    this->setRefCurrents(hijackedMotors.size(),
                         hijackedMotors.data(),
                         desiredMotorCurrentsHijackedMotors.data());

    timeOfLastControlLoop = BipedalLocomotion::clock().now();;
}
