/**
 * @file FloatingBaseEstimator.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <chrono>

#include <iDynTree/Model.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::Estimators;

FloatingBaseEstimator::FloatingBaseEstimator()
{
    m_state.imuOrientation.setIdentity();
}

bool FloatingBaseEstimator::initialize(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    std::shared_ptr<iDynTree::KinDynComputations> kindyn)
{
    if (!m_modelComp.setKinDynObject(kindyn))
    {
        log()->error("[FloatingBaseEstimator::initialize] "
                     "The pointer to KinDynComputations object could not be set.");
        return false;
    }

    if (!initialize(handler))
    {
        return false;
    }

    return true;
}

bool FloatingBaseEstimator::initialize(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("[FloatingBaseEstimator::initialize] "
                     "The parameter handler has expired. Please check its scope.");
        return false;
    }

    if (!handle->getParameter("use_model_info", m_useModelInfo))
    {
        m_useModelInfo = true;
        log()->warn("[FloatingBaseEstimator::initialize] "
                    "The parameter handler could not find \" use_model_info \" in the "
                    "configuration file."
                    "Setting to default value {}",
                    m_useModelInfo);
    }

    if (m_estimatorState != State::NotInitialized)
    {
        log()->error("[FloatingBaseEstimator::initialize] "
                     "The estimator seems to be already initialized.");
        return false;
    }

    // setup sampling period
    if (!handle->getParameter("sampling_period_in_s", m_dt))
    {
        log()->error("[FloatingBaseEstimator::initialize] "
                     "The parameter handler could not find \" sampling_period_in_s \" in the "
                     "configuration file.");
        return false;
    }

    if (m_useModelInfo)
    {
        if (!m_modelComp.isKinDynValid())
        {
            log()->error("[FloatingBaseEstimator::initialize] "
                         "The kindyn object with valid does not seem to be loaded."
                         "Please either call initialize(handler, kindyncomputations) to set the "
                         "kindyn object and re-initialize the estimator,"
                         "or call modelComp()->setKinDynObject(kindyncomputations) to set the "
                         "kindyn object only.");
            return false;
        }

        // setup model related entities
        auto modelHandle = handle->getGroup("ModelInfo");
        if (!setupModelParams(modelHandle))
        {
            log()->error("[FloatingBaseEstimator::initialize] "
                         "Could not load model related parameters.");
            return false;
        }
    }

    if (!customInitialization(handler))
    {
        log()->error("[FloatingBaseEstimator::initialize] "
                     "Could not run custom initialization of the filter.");
        return false;
    }

    m_estimatorState = State::Initialized;

    return true;
}

bool FloatingBaseEstimator::advance()
{
    if (m_estimatorState != State::Initialized && m_estimatorState != State::Running)
    {
        log()->error("[FloatingBaseEstimator::advance] "
                     "Please initialize the estimator before calling advance().");
        return false;
    }

    bool ok{true};
    if (m_estimatorState == State::Initialized)
    {
        m_estimatorState = State::Running;
        m_measPrev = m_meas;
    }

    if (m_useModelInfo && m_options.kinematicsUpdateEnabled)
    {
        // update robot state with previous state estimates
        if ((m_meas.encoders.size() == m_modelComp.nrJoints())
            || (m_meas.encodersSpeed.size() == m_modelComp.nrJoints()))
        {
            if (!m_modelComp.kinDyn()->setRobotState(m_estimatorOut.basePose.transform(),
                                                     m_meas.encoders,
                                                     m_estimatorOut.baseTwist,
                                                     m_meas.encodersSpeed,
                                                     m_options.accelerationDueToGravity))
            {
                log()->error("[FloatingBaseEstimator::advance] "
                             "Failed to set kindyncomputations robot state.");
                return false;
            }
        }
    }

    ok = ok && predictState(m_measPrev, m_dt);
    if (m_options.ekfUpdateEnabled)
    {
        if (m_useModelInfo && m_options.kinematicsUpdateEnabled)
        {
            ok = ok && updateKinematics(m_meas, m_dt);
        }

        if (m_options.staticLandmarksUpdateEnabled)
        {
            ok = ok && updateLandmarkRelativePoses(m_meas, m_dt);
        }
    }

    ok = ok
         && updateBaseStateFromIMUState(m_state,
                                        m_measPrev,
                                        m_estimatorOut.basePose,
                                        m_estimatorOut.baseTwist);

    m_statePrev = m_state;
    m_measPrev = m_meas;

    m_estimatorOut.state = m_state;
    m_estimatorOut.stateStdDev = m_stateStdDev;

    // clear the map of landmark and contact measurements
    if (m_useModelInfo)
    {
        m_meas.stampedContactsStatus.clear();
    }

    if (m_options.staticLandmarksUpdateEnabled)
    {
        m_meas.stampedRelLandmarkPoses.clear();
    }
    return ok;
}

bool FloatingBaseEstimator::ModelComputations::setKinDynObject(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if (kinDyn != nullptr)
    {
        m_kindyn = kinDyn;

        if (m_kindyn->isValid())
        {
            m_nrJoints = m_kindyn->getNrOfDegreesOfFreedom();
            m_validKinDyn = true;
            return true;
        }
    }

    log()->error("[FloatingBaseEstimator::ModelComputations::setKinDynObject] "
                 "Invalid KinDynComputations object.");
    return false;
}

bool FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU(const std::string& baseLink,
                                                                 const std::string& imuFrame)
{
    m_baseLinkIdx = m_kindyn->model().getFrameIndex(baseLink);
    if (m_baseLinkIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] "
                     "Specified base link not available in the loaded URDF Model.");
        return false;
    }

    m_baseImuIdx = m_kindyn->model().getFrameIndex(imuFrame);
    if (m_baseImuIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] "
                     "Specified IMU frame not available in the loaded URDF Model.");
        return false;
    }

    if (m_baseLinkIdx != m_kindyn->model().getFrameLink(m_baseImuIdx))
    {
        log()->error("[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] "
                     "Specified IMU not rigidly attached to the base link. Please specify a base "
                     "link colocated IMU.");
        return false;
    }

    if (m_kindyn->model().getDefaultBaseLink() != m_baseLinkIdx)
    {
        auto model = m_kindyn->model();
        model.setDefaultBaseLink(m_baseLinkIdx);
        m_kindyn->loadRobotModel(model);
    }

    m_baseLink = baseLink;
    m_baseImuFrame = imuFrame;
    m_base_H_imu = Conversions::toManifPose(m_kindyn->model().getFrameTransform(m_baseImuIdx));
    return true;
}

bool FloatingBaseEstimator::ModelComputations::setFeetContactFrames(
    const std::string& lFootContactFrame, const std::string& rFootContactFrame)
{
    m_lFootContactIdx = m_kindyn->model().getFrameIndex(lFootContactFrame);
    if (m_lFootContactIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("[FloatingBaseEstimator::ModelComputations::setFeetContactFrames] "
                     "Specified left foot contact frame not available in the loaded URDF Model.");
        return false;
    }

    m_rFootContactIdx = m_kindyn->model().getFrameIndex(rFootContactFrame);
    if (m_rFootContactIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("[FloatingBaseEstimator::ModelComputations::setFeetContactFrames] "
                     "Specified right foot contact frame not available in the loaded URDF Model.");
        return false;
    }

    m_lFootContactFrame = lFootContactFrame;
    m_rFootContactFrame = rFootContactFrame;

    return true;
}

bool FloatingBaseEstimator::ModelComputations::isModelInfoLoaded()
{
    bool loaded = (m_baseLinkIdx != iDynTree::FRAME_INVALID_INDEX)
                  && (m_baseImuIdx != iDynTree::FRAME_INVALID_INDEX);

    return loaded;
}

bool FloatingBaseEstimator::ModelComputations::getBaseStateFromIMUState(
    const manif::SE3d& A_H_IMU,
    Eigen::Ref<const Eigen::Matrix<double, 6, 1>> v_IMU,
    manif::SE3d& A_H_B,
    Eigen::Ref<Eigen::Matrix<double, 6, 1>> v_B)
{
    if (!isModelInfoLoaded())
    {
        log()->error("[FloatingBaseEstimator::ModelComputations::getBaseStateFromIMUState] "
                     "Please set required model info parameters, before calling "
                     "getBaseStateFromIMUState()");
        return false;
    }

    A_H_B = A_H_IMU * (m_base_H_imu.inverse());

    // transform velocity (mixed-representation)
    auto A_o_BIMU = A_H_B.asSO3().act(m_base_H_imu.translation());
    manif::SE3d X(A_o_BIMU, Eigen::Quaterniond::Identity());

    // coordinate/adjoint transformation of the twist
    v_B = X.adj() * v_IMU;

    return true;
}

bool FloatingBaseEstimator::ModelComputations::getIMU_H_feet(
    Eigen::Ref<const Eigen::VectorXd> encoders,
    manif::SE3d& IMU_H_l_foot,
    manif::SE3d& IMU_H_r_foot)
{
    if (!isModelInfoLoaded() && (encoders.size() != nrJoints()))
    {
        log()->error("[FloatingBaseEstimator::ModelComputations::getIMU_H_feet] "
                     "Please set required model info parameters, before calling getIMU_H_feet()");
        return false;
    }

    if (!m_kindyn->setJointPos(encoders))
    {
        log()->error("[FloatingBaseEstimator::ModelComputations::getIMU_H_feet] "
                     "Failed setting joint positions.");
    }
    IMU_H_l_foot
        = Conversions::toManifPose(m_kindyn->getRelativeTransform(m_baseImuIdx, m_lFootContactIdx));
    IMU_H_r_foot
        = Conversions::toManifPose(m_kindyn->getRelativeTransform(m_baseImuIdx, m_rFootContactIdx));

    return true;
}

bool FloatingBaseEstimator::ModelComputations::getIMU_H_feet(
    const Eigen::Ref<const Eigen::VectorXd> encoders,
    manif::SE3d& IMU_H_l_foot,
    manif::SE3d& IMU_H_r_foot,
    Eigen::Ref<Eigen::MatrixXd> J_IMULF,
    Eigen::Ref<Eigen::MatrixXd> J_IMURF)
{
    if (!getIMU_H_feet(encoders, IMU_H_l_foot, IMU_H_r_foot))
    {
        return false;
    }

    m_kindyn->getRelativeJacobian(m_baseImuIdx, m_lFootContactIdx, J_IMULF);
    m_kindyn->getRelativeJacobian(m_baseImuIdx, m_rFootContactIdx, J_IMURF);

    return true;
}

bool FloatingBaseEstimator::setIMUMeasurement(const Eigen::Vector3d& accMeas,
                                              const Eigen::Vector3d& gyroMeas)
{
    m_meas.acc = accMeas;
    m_meas.gyro = gyroMeas;

    return true;
}

bool FloatingBaseEstimator::setKinematics(const Eigen::VectorXd& encoders,
                                          const Eigen::VectorXd& encoderSpeeds)
{
    if ((encoders.size() != encoderSpeeds.size())
        || (encoders.size() != modelComputations().nrJoints()))
    {
        log()->warn("[FloatingBaseEstimator::setKinematics] "
                    "kinematic measurements size mismatch.");
        return false;
    }

    m_meas.encoders = encoders;
    m_meas.encodersSpeed = encoderSpeeds;
    return true;
}

bool FloatingBaseEstimator::setContacts(const bool& lfInContact, const bool& rfInContact)
{
    m_meas.lfInContact = lfInContact;
    m_meas.rfInContact = rfInContact;
    return true;
}

bool FloatingBaseEstimator::setContactStatus(const std::string& name,
                                             const bool& contactStatus,
                                             const std::chrono::nanoseconds& switchTime,
                                             const std::chrono::nanoseconds& timeNow)
{
    auto idx = m_modelComp.kinDyn()->model().getFrameIndex(name);
    if (!m_modelComp.kinDyn()->model().isValidFrameIndex(idx))
    {
        log()->warn("[FloatingBaseEstimator::setContactStatus] "
                    "Contact frame index: {} not found in loaded model, skipping measurement.",
                    idx);
        return false;
    }

    auto& contacts = m_meas.stampedContactsStatus;

    // operator[] creates a key-value pair if key does not already exist,
    // otherwise just an update is carried out
    contacts[idx].switchTime = switchTime;
    contacts[idx].isActive = contactStatus;
    contacts[idx].lastUpdateTime = timeNow;
    contacts[idx].index = idx;
    contacts[idx].name = name;

    return true;
}

bool FloatingBaseEstimator::setLandmarkRelativePose(const int& landmarkID,
                                                    const Eigen::Quaterniond& quat,
                                                    const Eigen::Vector3d& pos,
                                                    const std::chrono::nanoseconds& timeNow)
{
    auto& poses = m_meas.stampedRelLandmarkPoses;

    // operator[] creates a key-value pair if key does not already exist,
    // otherwise just an update is carried out
    poses[landmarkID].lastUpdateTime = timeNow;
    poses[landmarkID].pose = manif::SE3d(pos, quat);
    return true;
}

FloatingBaseEstimator::ModelComputations& FloatingBaseEstimator::modelComputations()
{
    return m_modelComp;
}

const FloatingBaseEstimators::Output& FloatingBaseEstimator::getOutput() const
{
    return m_estimatorOut;
}

bool FloatingBaseEstimator::setupModelParams(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("[FloatingBaseEstimator::setupModelParams] "
                     "The parameter handler has expired. Please check its scope.");
        return false;
    }

    // setup base link and imu
    std::string baseLink, imu;
    if (!handle->getParameter("base_link", baseLink))
    {
        log()->error("[FloatingBaseEstimator::setupModelParams] "
                     "The parameter handler could not find \" base_link \" in the configuration "
                     "file.");
        return false;
    }

    if (!handle->getParameter("base_link_imu", imu))
    {
        log()->error("[FloatingBaseEstimator::setupModelParams] "
                     "The parameter handler could not find \" base_link_imu \" in the "
                     "configuration file.");
        return false;
    }

    // setup base lfContact and rfContact
    std::string lfContact, rfContact;
    if (!handle->getParameter("left_foot_contact_frame", lfContact))
    {
        log()->error("[FloatingBaseEstimator::setupModelParams] "
                     "The parameter handler could not find \" left_foot_contact_frame \" in the "
                     "configuration file.");
        return false;
    }

    if (!handle->getParameter("right_foot_contact_frame", rfContact))
    {
        log()->error("[FloatingBaseEstimator::setupModelParams] "
                     "The parameter handler could not find \" right_foot_contact_frame \" in the "
                     "configuration file.");
        return false;
    }

    if (!m_modelComp.setBaseLinkAndIMU(baseLink, imu))
    {
        return false;
    }

    if (!m_modelComp.setFeetContactFrames(lfContact, rfContact))
    {
        return false;
    }

    return true;
}

bool FloatingBaseEstimator::setupOptions(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("[FloatingBaseEstimator::setupOptions] "
                     "The parameter handler has expired. Please check its scope.");
        return false;
    }

    if (!handle->getParameter("enable_imu_bias_estimation", m_options.imuBiasEstimationEnabled))
    {
        log()->error("[FloatingBaseEstimator::setupOptions] "
                     "The parameter handler has expired. Please check its scope.");
        std::cout << "[FloatingBaseEstimator::setupOptions] The parameter handler could not find "
                     "\" enable_imu_bias_estimation \" in the configuration file. Setting default "
                     "value to false."
                  << std::endl;
        m_options.imuBiasEstimationEnabled = false;
    }

    if (m_options.imuBiasEstimationEnabled)
    {
        if (!handle->getParameter("enable_static_imu_bias_initialization",
                                  m_options.staticImuBiasInitializationEnabled))
        {
            m_options.staticImuBiasInitializationEnabled = false;
            log()->warn("[FloatingBaseEstimator::setupOptions] "
                        "The parameter handler could not find \" "
                        "enable_static_imu_bias_initialization \"  "
                        "in the configuration file. Setting to default value: {}.",
                        m_options.staticImuBiasInitializationEnabled);
        }

        if (m_options.staticImuBiasInitializationEnabled)
        {
            if (!handle->getParameter("nr_samples_for_imu_bias_initialization",
                                      m_options.nrSamplesForImuBiasInitialization))
            {
                log()->error("[FloatingBaseEstimator::setupOptions] "
                             "The parameter handler could not find \" "
                             "nr_samples_for_imu_bias_initialization \"  "
                             "in the configuration file.  This is a required parameter since "
                             "the static_imu_bias_initialization_enabled is set to true.");
                return false;
            }
        }
    }

    if (!handle->getParameter("enable_ekf_update", m_options.ekfUpdateEnabled))
    {
        m_options.ekfUpdateEnabled = true;
    }

    if (!m_options.ekfUpdateEnabled)
    {
        log()->warn("[FloatingBaseEstimator::setupOptions] "
                    "EKF Measurement updates are disabled. "
                    "This might cause quick divergence of the estimates. "
                    "Please enable the update for expected performance.");
        m_options.kinematicsUpdateEnabled = false;
        m_options.staticLandmarksUpdateEnabled = false;
    } else
    {
        if (!handle->getParameter("use_kinematics_measure", m_options.kinematicsUpdateEnabled))
        {
            m_options.kinematicsUpdateEnabled = true;
        }

        if (!handle->getParameter("use_static_ldmks_pose_measure",
                                  m_options.staticLandmarksUpdateEnabled))
        {
            m_options.staticLandmarksUpdateEnabled = false;
        }
    }

    // This parameter is optional
    handle->getParameter("acceleration_due_to_gravity", //
                         m_options.accelerationDueToGravity);

    return true;
}

bool FloatingBaseEstimator::setupSensorDevs(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("[FloatingBaseEstimator::setupSensorDevs] "
                     "The parameter handler has expired. Please check its scope.");
        return false;
    }

    std::string printPrefix{"setupSensorDevs"};

    std::vector<double> accNoise(3);
    if (!setupFixedVectorParamPrivate("accelerometer_measurement_noise_std_dev",
                                      printPrefix,
                                      handler,
                                      accNoise))
    {
        return false;
    }

    std::vector<double> gyroNoise(3);
    if (!setupFixedVectorParamPrivate("gyroscope_measurement_noise_std_dev",
                                      printPrefix,
                                      handler,
                                      gyroNoise))
    {
        return false;
    }

    if (m_useModelInfo)
    {
        std::vector<double> contactFootLinvelNoise(3);
        if (!setupFixedVectorParamPrivate("contact_foot_linear_velocity_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          contactFootLinvelNoise))
        {
            return false;
        }

        std::vector<double> contactFootAngvelNoise(3);
        if (!setupFixedVectorParamPrivate("contact_foot_angular_velocity_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          contactFootAngvelNoise))
        {
            return false;
        }

        std::vector<double> swingFootLinvelNoise(3);
        if (!setupFixedVectorParamPrivate("swing_foot_linear_velocity_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          swingFootLinvelNoise))
        {
            return false;
        }

        std::vector<double> swingFootAngvelNoise(3);
        if (!setupFixedVectorParamPrivate("swing_foot_angular_velocity_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          swingFootAngvelNoise))
        {
            return false;
        }

        std::vector<double> forwardKinematicsNoise(6);
        if (!setupFixedVectorParamPrivate("forward_kinematic_measurement_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          forwardKinematicsNoise))
        {
            return false;
        }

        std::vector<double> encodersNoise(m_modelComp.nrJoints());
        if (!setupFixedVectorParamPrivate("encoders_measurement_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          encodersNoise))
        {
            return false;
        }

        m_sensorsDev.contactFootLinvelNoise << contactFootLinvelNoise[0], contactFootLinvelNoise[1],
            contactFootLinvelNoise[2];
        m_sensorsDev.contactFootAngvelNoise << contactFootAngvelNoise[0], contactFootAngvelNoise[1],
            contactFootAngvelNoise[2];

        m_sensorsDev.swingFootLinvelNoise << swingFootLinvelNoise[0], swingFootLinvelNoise[1],
            swingFootLinvelNoise[2];
        m_sensorsDev.swingFootAngvelNoise << swingFootAngvelNoise[0], swingFootAngvelNoise[1],
            swingFootAngvelNoise[2];

        m_sensorsDev.forwardKinematicsNoise << forwardKinematicsNoise[0], forwardKinematicsNoise[1],
            forwardKinematicsNoise[2], forwardKinematicsNoise[3], forwardKinematicsNoise[4],
            forwardKinematicsNoise[5];

        m_sensorsDev.encodersNoise.resize(encodersNoise.size());
        m_sensorsDev.encodersNoise
            = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(encodersNoise.data(),
                                                            encodersNoise.size());
    }

    if (m_options.imuBiasEstimationEnabled)
    {
        std::vector<double> accBiasNoise(3);
        if (!setupFixedVectorParamPrivate("accelerometer_measurement_bias_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          accBiasNoise))
        {
            return false;
        }

        std::vector<double> gyroBiasNoise(3);
        if (!setupFixedVectorParamPrivate("gyroscope_measurement_bias_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          gyroBiasNoise))
        {
            return false;
        }

        m_sensorsDev.accelerometerBiasNoise << accBiasNoise[0], accBiasNoise[1], accBiasNoise[2];
        m_sensorsDev.gyroscopeBiasNoise << gyroBiasNoise[0], gyroBiasNoise[1], gyroBiasNoise[2];
    }

    m_sensorsDev.accelerometerNoise << accNoise[0], accNoise[1], accNoise[2];
    m_sensorsDev.gyroscopeNoise << gyroNoise[0], gyroNoise[1], gyroNoise[2];

    if (m_options.staticLandmarksUpdateEnabled)
    {
        std::vector<double> ldmkPredictionNoise(6);
        if (!setupFixedVectorParamPrivate("landmark_prediction_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          ldmkPredictionNoise))
        {
            return false;
        }

        std::vector<double> ldmkMeasurmentNoise(6);
        if (!setupFixedVectorParamPrivate("landmark_measurement_noise_std_dev",
                                          printPrefix,
                                          handler,
                                          ldmkMeasurmentNoise))
        {
            return false;
        }

        m_sensorsDev.landmarkPredictionNoise << ldmkPredictionNoise[0], ldmkPredictionNoise[1],
            ldmkPredictionNoise[2], ldmkPredictionNoise[3], ldmkPredictionNoise[4],
            ldmkPredictionNoise[5];

        m_sensorsDev.landmarkMeasurementNoise << ldmkMeasurmentNoise[0], ldmkMeasurmentNoise[1],
            ldmkMeasurmentNoise[2], ldmkMeasurmentNoise[3], ldmkMeasurmentNoise[4],
            ldmkMeasurmentNoise[5];
    }

    return true;
}

bool FloatingBaseEstimator::setupInitialStates(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("[FloatingBaseEstimator::setupInitialStates] "
                     "The parameter handler has expired. Please check its scope.");
        return false;
    }

    std::string printPrefix{"setupInitialStates"};

    std::vector<double> imuOrientation(4);
    if (!setupFixedVectorParamPrivate("imu_orientation_quaternion_wxyz",
                                      printPrefix,
                                      handler,
                                      imuOrientation))
    {
        return false;
    }

    std::vector<double> imuPosition(3);
    if (!setupFixedVectorParamPrivate("imu_position_xyz", printPrefix, handler, imuPosition))
    {
        return false;
    }

    std::vector<double> imuLinearVelocity(3);
    if (!setupFixedVectorParamPrivate("imu_linear_velocity_xyz",
                                      printPrefix,
                                      handler,
                                      imuLinearVelocity))
    {
        return false;
    }

    if (m_useModelInfo && m_isInvEKF)
    {
        // This conditional block will be soon deprecated
        std::vector<double> lContactFrameOrientation(4);
        if (!setupFixedVectorParamPrivate("l_contact_frame_orientation_quaternion_wxyz",
                                          printPrefix,
                                          handler,
                                          lContactFrameOrientation))
        {
            return false;
        }

        std::vector<double> lContactFramePosition(3);
        if (!setupFixedVectorParamPrivate("l_contact_frame_position_xyz",
                                          printPrefix,
                                          handler,
                                          lContactFramePosition))
        {
            return false;
        }

        std::vector<double> rContactFrameOrientation(4);
        if (!setupFixedVectorParamPrivate("r_contact_frame_orientation_quaternion_wxyz",
                                          printPrefix,
                                          handler,
                                          rContactFrameOrientation))
        {
            return false;
        }

        std::vector<double> rContactFramePosition(3);
        if (!setupFixedVectorParamPrivate("r_contact_frame_position_xyz",
                                          printPrefix,
                                          handler,
                                          rContactFramePosition))
        {
            return false;
        }

        m_statePrev.lContactFrameOrientation
            = Eigen::Quaterniond(lContactFrameOrientation[0],
                                 lContactFrameOrientation[1],
                                 lContactFrameOrientation[2],
                                 lContactFrameOrientation[3]); // here loaded as w x y z
        m_statePrev.lContactFrameOrientation.normalize(); // normalize the user defined quaternion
                                                          // to respect internal tolerances for unit
                                                          // norm constraint
        m_statePrev.lContactFramePosition << lContactFramePosition[0], lContactFramePosition[1],
            lContactFramePosition[2];

        m_statePrev.rContactFrameOrientation
            = Eigen::Quaterniond(rContactFrameOrientation[0],
                                 rContactFrameOrientation[1],
                                 rContactFrameOrientation[2],
                                 rContactFrameOrientation[3]); // here loaded as w x y z
        m_statePrev.rContactFrameOrientation.normalize(); // normalize the user defined quaternion
                                                          // to respect internal tolerances for unit
                                                          // norm constraint
        m_statePrev.rContactFramePosition << rContactFramePosition[0], rContactFramePosition[1],
            rContactFramePosition[2];
    }

    if (m_options.imuBiasEstimationEnabled)
    {
        std::vector<double> accelerometerBias(3);
        if (!setupFixedVectorParamPrivate("accelerometer_bias",
                                          printPrefix,
                                          handler,
                                          accelerometerBias))
        {
            return false;
        }

        std::vector<double> gyroscopeBias(3);
        if (!setupFixedVectorParamPrivate("gyroscope_bias", printPrefix, handler, gyroscopeBias))
        {
            return false;
        }

        m_statePrev.accelerometerBias << accelerometerBias[0], accelerometerBias[1],
            accelerometerBias[2];
        m_statePrev.gyroscopeBias << gyroscopeBias[0], gyroscopeBias[1], gyroscopeBias[2];
    }

    m_statePrev.imuOrientation = Eigen::Quaterniond(imuOrientation[0],
                                                    imuOrientation[1],
                                                    imuOrientation[2],
                                                    imuOrientation[3]); // here loaded as w x y z
    m_statePrev.imuOrientation.normalize(); // normalize the user defined quaternion to respect
                                            // internal tolerances for unit norm constraint
    m_statePrev.imuPosition << imuPosition[0], imuPosition[1], imuPosition[2];
    m_statePrev.imuLinearVelocity << imuLinearVelocity[0], imuLinearVelocity[1],
        imuLinearVelocity[2];

    m_state = m_statePrev;

    if (!updateBaseStateFromIMUState(m_state,
                                     m_measPrev,
                                     m_estimatorOut.basePose,
                                     m_estimatorOut.baseTwist))
    {
        log()->error("[FloatingBaseEstimator::setupInitialStates] "
                     "Failed to initialize base link state from IMU state.");
        return false;
    }

    return true;
}

bool FloatingBaseEstimator::setupPriorDevs(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        log()->error("[FloatingBaseEstimator::setupPriorDevs] "
                     "The parameter handler has expired. Please check its scope.");
        return false;
    }

    std::string printPrefix{"setupPriorDevs"};

    std::vector<double> imuOrientation(3);
    if (!setupFixedVectorParamPrivate("imu_orientation", printPrefix, handler, imuOrientation))
    {
        return false;
    }

    std::vector<double> imuPosition(3);
    if (!setupFixedVectorParamPrivate("imu_position", printPrefix, handler, imuPosition))
    {
        return false;
    }

    std::vector<double> imuLinearVelocity(3);
    if (!setupFixedVectorParamPrivate("imu_linear_velocity",
                                      printPrefix,
                                      handler,
                                      imuLinearVelocity))
    {
        return false;
    }

    if (m_useModelInfo && m_isInvEKF)
    {
        // This conditional block will be soon deprecated
        std::vector<double> lContactFrameOrientation(3);
        if (!setupFixedVectorParamPrivate("l_contact_frame_orientation",
                                          printPrefix,
                                          handler,
                                          lContactFrameOrientation))
        {
            return false;
        }

        std::vector<double> lContactFramePosition(3);
        if (!setupFixedVectorParamPrivate("l_contact_frame_position",
                                          printPrefix,
                                          handler,
                                          lContactFramePosition))
        {
            return false;
        }

        std::vector<double> rContactFrameOrientation(3);
        if (!setupFixedVectorParamPrivate("r_contact_frame_orientation",
                                          printPrefix,
                                          handler,
                                          rContactFrameOrientation))
        {
            return false;
        }

        std::vector<double> rContactFramePosition(3);
        if (!setupFixedVectorParamPrivate("r_contact_frame_position",
                                          printPrefix,
                                          handler,
                                          rContactFramePosition))
        {
            return false;
        }

        m_priors.lContactFrameOrientation << lContactFrameOrientation[0],
            lContactFrameOrientation[1], lContactFrameOrientation[2];
        m_priors.lContactFramePosition << lContactFramePosition[0], lContactFramePosition[1],
            lContactFramePosition[2];

        m_priors.rContactFrameOrientation << rContactFrameOrientation[0],
            rContactFrameOrientation[1], rContactFrameOrientation[2];
        m_priors.rContactFramePosition << rContactFramePosition[0], rContactFramePosition[1],
            rContactFramePosition[2];
    }

    if (m_options.imuBiasEstimationEnabled)
    {
        std::vector<double> accelerometerBias(3);
        if (!setupFixedVectorParamPrivate("accelerometer_bias",
                                          printPrefix,
                                          handler,
                                          accelerometerBias))
        {
            return false;
        }

        std::vector<double> gyroscopeBias(3);
        if (!setupFixedVectorParamPrivate("gyroscope_bias", printPrefix, handler, gyroscopeBias))
        {
            return false;
        }

        m_priors.accelerometerBias << accelerometerBias[0], accelerometerBias[1],
            accelerometerBias[2];
        m_priors.gyroscopeBias << gyroscopeBias[0], gyroscopeBias[1], gyroscopeBias[2];
    }

    m_priors.imuOrientation << imuOrientation[0], imuOrientation[1], imuOrientation[2];
    m_priors.imuPosition << imuPosition[0], imuPosition[1], imuPosition[2];
    m_priors.imuLinearVelocity << imuLinearVelocity[0], imuLinearVelocity[1], imuLinearVelocity[2];

    return true;
}

bool FloatingBaseEstimator::setupFixedVectorParamPrivate(
    const std::string& param,
    const std::string& prefix,
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    std::vector<double>& vec)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        return false;
    }

    if (!handle->getParameter(param,
                              GenericContainer::
                                  make_vector(vec, GenericContainer::VectorResizeMode::Fixed)))
    {
        log()->error("[FloatingBaseEstimator::{}] "
                     "The parameter handler could not find \"{}\" in the configuration file."
                     "This is a required parameter.",
                     prefix,
                     param);
        return false;
    }

    return true;
}

bool FloatingBaseEstimator::updateBaseStateFromIMUState(
    const FloatingBaseEstimators::InternalState& state,
    const FloatingBaseEstimators::Measurements& meas,
    manif::SE3d& basePose,
    Eigen::Ref<Eigen::Matrix<double, 6, 1>> baseTwist)
{
    manif::SE3d A_H_IMU(state.imuPosition, state.imuOrientation);
    Eigen::Matrix<double, 6, 1> v_IMU;

    v_IMU.head<3>() = state.imuLinearVelocity;
    if (m_useIMUForAngVelEstimate)
    {
        v_IMU.tail<3>()
            = state.imuOrientation.toRotationMatrix() * (meas.gyro - state.gyroscopeBias);
    } else
    {
        v_IMU.tail<3>() = state.imuAngularVelocity;
    }

    Eigen::Matrix<double, 6, 1> tempTwist;
    if (m_useModelInfo)
    {
        if (!m_modelComp.getBaseStateFromIMUState(A_H_IMU, v_IMU, basePose, tempTwist))
        {
            log()->error("[FloatingBaseEstimator::updateBaseStateFromIMUState] "
                         "Failed to get base link state from IMU state.");
            return false;
        }

        if (m_useIMUVelForBaseVelComputation)
        {
            baseTwist = tempTwist;
        }
    } else
    {
        basePose = A_H_IMU;
        baseTwist = v_IMU;
    }

    return true;
}

bool FloatingBaseEstimator::resetEstimator(const Eigen::Quaterniond& newBaseOrientation,
                                           const Eigen::Vector3d& newBasePosition)
{
    auto A_H_B = manif::SE3d(newBasePosition, newBaseOrientation);
    manif::SE3d IMU_H_RF, IMU_H_LF;

    if (!m_modelComp.getIMU_H_feet(m_meas.encoders, IMU_H_RF, IMU_H_LF))
    {
        log()->error("[FloatingBaseEstimator::resetEstimator] "
                     "Could not reset estimator using new base pose.");
        return false;
    }

    auto A_H_IMU = A_H_B * m_modelComp.base_H_IMU();
    auto A_H_RF = A_H_IMU * IMU_H_RF;
    auto A_H_LF = A_H_IMU * IMU_H_LF;

    m_state.imuOrientation = A_H_IMU.quat();
    m_state.imuPosition = A_H_IMU.translation();
    m_state.rContactFrameOrientation = A_H_RF.quat();
    m_state.rContactFramePosition = A_H_RF.translation();
    m_state.lContactFrameOrientation = A_H_LF.quat();
    m_state.lContactFramePosition = A_H_LF.translation();

    return true;
}

bool FloatingBaseEstimator::resetEstimator(const FloatingBaseEstimators::InternalState& newState)
{
    m_state = newState;
    return true;
}
