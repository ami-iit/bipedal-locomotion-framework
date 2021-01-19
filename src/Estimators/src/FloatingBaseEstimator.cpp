/**
 * @file FloatingBaseEstimator.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */


#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>
#include <iDynTree/Core/EigenHelpers.h>

using namespace BipedalLocomotion::Estimators;

bool FloatingBaseEstimator::initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                       std::shared_ptr<iDynTree::KinDynComputations> kindyn,
                                       const iDynTree::Model& model)
{
    if (!m_modelComp.setKinDynObject(kindyn))
    {
        std::cerr << "[FloatingBaseEstimator::initialize] The pointer to KinDynComputations object could not be set."
        << std::endl;
        return false;
    }
    
    if (!m_modelComp.setModel(model))
    {
        std::cerr << "[FloatingBaseEstimator::initialize] The model could not be loaded."
        << std::endl;
        return false;
    }

    if (!initialize(handler))
    {
        return false;
    }

    return true;
}

bool FloatingBaseEstimator::initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    if (!m_modelComp.isModelSet())
    {
        std::cerr << "[FloatingBaseEstimator::initialize] The model does not seem to be loaded. Please call initialize(handler, model) to set the model."
        << std::endl;
        return false;
    }

    if (m_estimatorState != State::NotInitialized)
    {
        std::cerr << "[FloatingBaseEstimator::initialize] The estimator already seems to be initialized."
        << std::endl;
        return false;
    }

    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[FloatingBaseEstimator::initialize] The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    // setup sampling period
    if (!handle->getParameter("sampling_period_in_s", m_dt))
    {
        std::cerr << "[FloatingBaseEstimator::initialize] "
        "The parameter handler could not find \" sampling_period_in_s \" in the configuration file."
        << std::endl;
        return false;
    }

    // setup model related entities
    auto modelHandle = handle->getGroup("ModelInfo");
    if (!setupModelParams(modelHandle))
    {
        std::cerr << "[FloatingBaseEstimator::initialize] "
        "Could not load model related parameters."
        << std::endl;
        return false;
    }

    if (!customInitialization(handler))
    {
        std::cerr << "[FloatingBaseEstimator::initialize] "
        "Could not run custom initialization of the filter."
        << std::endl;
        return false;
    }

    m_estimatorState = State::Initialized;

    return true;
}

bool FloatingBaseEstimator::advance()
{
    if (m_estimatorState != State::Initialized && m_estimatorState != State::Running)
    {
        std::cerr << "[FloatingBaseEstimator::advance] Please initialize the estimator before calling advance()."
        << std::endl;
        return false;
    }

    bool ok{true};
    if (m_estimatorState == State::Initialized)
    {
        m_estimatorState = State::Running;
        m_measPrev = m_meas;
    }

    ok = ok && predictState(m_measPrev, m_dt);
    if (m_options.ekfUpdateEnabled)
    {
        ok = ok && updateKinematics(m_meas, m_dt);
    }

    ok = ok && updateBaseStateFromIMUState(m_state, m_measPrev,
                                           m_estimatorOut.basePose, m_estimatorOut.baseTwist);
    m_statePrev = m_state;
    m_measPrev = m_meas;

    m_estimatorOut.state = m_state;
    m_estimatorOut.stateStdDev = m_stateStdDev;

    return ok;
}

bool FloatingBaseEstimator::ModelComputations::setKinDynObject(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if (kinDyn != nullptr)
    {
        m_kindyn = kinDyn;
        m_validKinDyn = true;
        return true;
    }
    
    std::cerr << "[FloatingBaseEstimator::ModelComputations::setModel] Invalid KinDynComputations object." << std::endl;
    return false;
}

bool FloatingBaseEstimator::ModelComputations::setModel(const iDynTree::Model& model)
{
    if (!isKinDynValid())
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setModel] Please set kindyn object before calling this method."
        << std::endl;
        return false;
    }
    
    if (!m_kindyn->loadRobotModel(model))
    {
        return false;
    }

    m_nrJoints = model.getNrOfDOFs();
    m_modelSet = true;
    return true;
}

bool FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU(const std::string& baseLink,
                                                                 const std::string& imuFrame)
{
    m_baseLinkIdx = m_kindyn->model().getFrameIndex(baseLink);
    if (m_baseLinkIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] Specified base link not available in the loaded URDF Model."
        << std::endl;
        return false;
    }

    m_baseImuIdx = m_kindyn->model().getFrameIndex(imuFrame);
    if (m_baseImuIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] Specified IMU frame not available in the loaded URDF Model."
        << std::endl;
        return false;
    }

    if (m_baseLinkIdx != m_kindyn->model().getFrameLink(m_baseImuIdx))
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] Specified IMU not rigidly attached to the base link. Please specify a base link colocated IMU."
        << std::endl;
        return false;
    }


    if (m_kindyn->model().getDefaultBaseLink() != m_baseLinkIdx)
    {
        auto model = m_kindyn->model();
        model.setDefaultBaseLink(m_baseLinkIdx);
        setModel(model);
    }

    m_baseLink = baseLink;
    m_baseImuFrame = imuFrame;
    m_base_H_imu = m_kindyn->model().getFrameTransform(m_baseImuIdx);
    return true;
}

bool FloatingBaseEstimator::ModelComputations::setFeetContactFrames(const std::string& lFootContactFrame,
                                                                    const std::string& rFootContactFrame)
{
    m_lFootContactIdx = m_kindyn->model().getFrameIndex(lFootContactFrame);
    if (m_lFootContactIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setFeetContactFrames] Specified left foot contact frame not available in the loaded URDF Model."
        << std::endl;
        return false;
    }

    m_rFootContactIdx = m_kindyn->model().getFrameIndex(rFootContactFrame);
    if (m_rFootContactIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setFeetContactFrames] Specified right foot contact frame not available in the loaded URDF Model."
        << std::endl;
        return false;
    }

    m_lFootContactFrame = lFootContactFrame;
    m_rFootContactFrame = rFootContactFrame;

    return true;
}

bool FloatingBaseEstimator::ModelComputations::isModelInfoLoaded()
{
    bool loaded = (m_baseLinkIdx != iDynTree::FRAME_INVALID_INDEX) &&
             (m_baseImuIdx != iDynTree::FRAME_INVALID_INDEX) &&
             (m_lFootContactIdx != iDynTree::FRAME_INVALID_INDEX) &&
             (m_rFootContactIdx != iDynTree::FRAME_INVALID_INDEX);

    return loaded;
}

bool FloatingBaseEstimator::ModelComputations::getBaseStateFromIMUState(const iDynTree::Transform& A_H_IMU,
                                                                        const iDynTree::Twist& v_IMU,
                                                                        iDynTree::Transform& A_H_B,
                                                                        iDynTree::Twist& v_B)
{
    if (!isModelInfoLoaded())
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::getBaseStateFromIMUState] Please set required model info parameters, before calling getBaseStateFromIMUState(...)"
        << std::endl;
        return false;
    }

    A_H_B = A_H_IMU*(m_base_H_imu.inverse());

    // transform velocity (mixed-representation)
    auto A_o_BIMU = A_H_B.getRotation().changeCoordFrameOf(m_base_H_imu.getPosition());
    auto X = iDynTree::Transform(iDynTree::Rotation::Identity(), A_o_BIMU);
    v_B = X*v_IMU;

    return true;
}

bool FloatingBaseEstimator::ModelComputations::getIMU_H_feet(const iDynTree::JointPosDoubleArray& encoders,
                                                             iDynTree::Transform& IMU_H_l_foot,
                                                             iDynTree::Transform& IMU_H_r_foot)
{
    if (!isModelInfoLoaded() && (encoders.size() !=nrJoints()))
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::getIMU_H_feet] Please set required model info parameters, before calling getIMU_H_feet(...)"
        << std::endl;
        return false;
    }

    if (!m_kindyn->setJointPos(encoders))
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::getIMU_H_feet] Failed setting joint positions." << std::endl;
        return false;
    }
    IMU_H_l_foot = m_kindyn->getRelativeTransform(m_baseImuIdx, m_lFootContactIdx);
    IMU_H_r_foot = m_kindyn->getRelativeTransform(m_baseImuIdx, m_rFootContactIdx);

    return true;
}

bool FloatingBaseEstimator::ModelComputations::getIMU_H_feet(const iDynTree::JointPosDoubleArray& encoders,
                                                             iDynTree::Transform& IMU_H_l_foot,
                                                             iDynTree::Transform& IMU_H_r_foot,
                                                             iDynTree::MatrixDynSize& J_IMULF,
                                                             iDynTree::MatrixDynSize& J_IMURF)
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
    if ( (encoders.size() != encoderSpeeds.size()) ||
        (encoders.size() != modelComputations().nrJoints()))
    {
        std::cerr << "[FloatingBaseEstimator::setKinematics] kinematic measurements size mismatch"
        << std::endl;
        return false;
    }

    m_meas.encoders = encoders;
    m_meas.encodersSpeed = encoderSpeeds;
    return true;
}

bool FloatingBaseEstimator::setContacts(const bool& lfInContact,
                                        const bool& rfInContact)
{
    m_meas.lfInContact = lfInContact;
    m_meas.rfInContact = rfInContact;
    return true;
}

bool FloatingBaseEstimator::setContactStatus(const std::string& name, 
                                             const bool& contactStatus, 
                                             const double& timeNow)
{
    auto idx = m_modelComp.kinDyn()->model().getFrameIndex(name);
    if (!m_modelComp.kinDyn()->model().isValidFrameIndex(idx))
    {
        std::cerr << "[FloatingBaseEstimator::setContactStatus] Contact frame index: " << idx
        << " not found in loaded model, skipping measurement." << std::endl;
        return false;
    }
    
    auto& contacts = m_meas.stampedContactsStatus;
    
    // operator[] creates a key-value pair if key does not already exist, 
    // otherwise just an update is carried out
    contacts[idx].first = timeNow;
    contacts[idx].second = contactStatus;
    
    return true;
}

FloatingBaseEstimator::ModelComputations& FloatingBaseEstimator::modelComputations()
{
    return m_modelComp;
}


const FloatingBaseEstimators::Output& FloatingBaseEstimator::get() const
{
    return m_estimatorOut;
}


bool FloatingBaseEstimator::setupModelParams(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    // setup base link and imu
    std::string baseLink, imu;
    if (!handle->getParameter("base_link", baseLink))
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] The parameter handler could not find \" base_link \" in the configuration file."
        << std::endl;
        return false;
    }

    if (!handle->getParameter("base_link_imu", imu))
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] The parameter handler could not find \" base_link_imu \" in the configuration file."
        << std::endl;
        return false;
    }

    // setup base lfContact and rfContact
    std::string lfContact, rfContact;
    if (!handle->getParameter("left_foot_contact_frame", lfContact))
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] The parameter handler could not find \" base_link \" in the configuration file."
        << std::endl;
        return false;
    }

    if (!handle->getParameter("right_foot_contact_frame", rfContact))
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] The parameter handler could not find \" base_link \" in the configuration file."
        << std::endl;
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

bool FloatingBaseEstimator::setupOptions(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[FloatingBaseEstimator::setupOptions] The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    if (!handle->getParameter("enable_imu_bias_estimation", m_options.imuBiasEstimationEnabled))
    {
        std::cout << "[FloatingBaseEstimator::setupOptions] The parameter handler could not find \" enable_imu_bias_estimation \" in the configuration file. Setting default value to false."
        << std::endl;
        m_options.imuBiasEstimationEnabled = false;
    }

    if (m_options.imuBiasEstimationEnabled)
    {
        if (!handle->getParameter("enable_static_imu_bias_initialization", m_options.staticImuBiasInitializationEnabled))
        {
            std::cout << "[FloatingBaseEstimator::setupOptions] The parameter handler could not find \" enable_static_imu_bias_initialization \" in the configuration file. Setting default value to false."
            << std::endl;
            m_options.staticImuBiasInitializationEnabled = false;
        }

        if (m_options.staticImuBiasInitializationEnabled)
        {
            if (!handle->getParameter("nr_samples_for_imu_bias_initialization", m_options.nrSamplesForImuBiasInitialization))
            {
                std::cerr << "[FloatingBaseEstimator::setupOptions] The parameter handler could not find \" nr_samples_for_imu_bias_initialization \" in the configuration file. "
                "This is a required parameter since the static_imu_bias_initialization_enabled is set to true."
                << std::endl;
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
        std::cerr << "[FloatingBaseEstimator::setupOptions] [WARNING] EKF Measurement updates are disabled. This might cause quick divergence of the estimates. Please enable the update for expected performance."
        << std::endl;
    }

    std::vector<double> g;
    g.resize(3);
    if (handle->getParameter("acceleration_due_to_gravity", GenericContainer::make_vector(g, GenericContainer::VectorResizeMode::Fixed)))
    {
        m_options.accelerationDueToGravity << g[0], g[1], g[2];
    }

    std::cout << "[FloatingBaseEstimator::setupOptions] Setting value for acceleration due to gravity: "
    << m_options.accelerationDueToGravity.transpose() << std::endl;

    return true;
}

bool FloatingBaseEstimator::setupSensorDevs(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[FloatingBaseEstimator::setupSensorDevs] The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    std::string printPrefix{"setupSensorDevs"};

    std::vector<double> accNoise(3);
    if (!setupFixedVectorParamPrivate("accelerometer_measurement_noise_std_dev", printPrefix, handler, accNoise)) { return false; }

    std::vector<double> gyroNoise(3);
    if (!setupFixedVectorParamPrivate("gyroscope_measurement_noise_std_dev", printPrefix, handler, gyroNoise)) { return false; }

    std::vector<double> contactFootLinvelNoise(3);
    if (!setupFixedVectorParamPrivate("contact_foot_linear_velocity_noise_std_dev", printPrefix, handler, contactFootLinvelNoise)) { return false; }

    std::vector<double> contactFootAngvelNoise(3);
    if (!setupFixedVectorParamPrivate("contact_foot_angular_velocity_noise_std_dev", printPrefix, handler, contactFootAngvelNoise)) { return false; }

    std::vector<double> swingFootLinvelNoise(3);
    if (!setupFixedVectorParamPrivate("swing_foot_linear_velocity_noise_std_dev", printPrefix, handler, swingFootLinvelNoise)) { return false; }

    std::vector<double> swingFootAngvelNoise(3);
    if (!setupFixedVectorParamPrivate("swing_foot_angular_velocity_noise_std_dev", printPrefix, handler, swingFootAngvelNoise)) { return false; }

    std::vector<double> forwardKinematicsNoise(6);
    if (!setupFixedVectorParamPrivate("forward_kinematic_measurement_noise_std_dev", printPrefix, handler, forwardKinematicsNoise)) { return false; }

    std::vector<double> encodersNoise(m_modelComp.nrJoints());
    if (!setupFixedVectorParamPrivate("encoders_measurement_noise_std_dev", printPrefix, handler, encodersNoise)) { return false; }

    if (m_options.imuBiasEstimationEnabled)
    {
        std::vector<double> accBiasNoise(3);
        if (!setupFixedVectorParamPrivate("accelerometer_measurement_bias_noise_std_dev", printPrefix, handler, accBiasNoise)) { return false; }

        std::vector<double> gyroBiasNoise(3);
        if (!setupFixedVectorParamPrivate("gyroscope_measurement_bias_noise_std_dev", printPrefix, handler, gyroBiasNoise)) { return false; }

        m_sensorsDev.accelerometerBiasNoise << accBiasNoise[0], accBiasNoise[1], accBiasNoise[2];
        m_sensorsDev.gyroscopeBiasNoise << gyroBiasNoise[0], gyroBiasNoise[1], gyroBiasNoise[2];
    }

    m_sensorsDev.accelerometerNoise << accNoise[0], accNoise[1], accNoise[2];
    m_sensorsDev.gyroscopeNoise << gyroNoise[0], gyroNoise[1], gyroNoise[2];

    m_sensorsDev.contactFootLinvelNoise << contactFootLinvelNoise[0], contactFootLinvelNoise[1], contactFootLinvelNoise[2];
    m_sensorsDev.contactFootAngvelNoise << contactFootAngvelNoise[0], contactFootAngvelNoise[1], contactFootAngvelNoise[2];

    m_sensorsDev.swingFootLinvelNoise << swingFootLinvelNoise[0], swingFootLinvelNoise[1], swingFootLinvelNoise[2];
    m_sensorsDev.swingFootAngvelNoise << swingFootAngvelNoise[0], swingFootAngvelNoise[1], swingFootAngvelNoise[2];

    m_sensorsDev.forwardKinematicsNoise << forwardKinematicsNoise[0], forwardKinematicsNoise[1], forwardKinematicsNoise[2],
                                           forwardKinematicsNoise[3], forwardKinematicsNoise[4], forwardKinematicsNoise[5];

    m_sensorsDev.encodersNoise.resize(encodersNoise.size());
    m_sensorsDev.encodersNoise = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(encodersNoise.data(), encodersNoise.size());
    return true;
}

bool FloatingBaseEstimator::setupInitialStates(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[FloatingBaseEstimator::setupInitialStates] The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    std::string printPrefix{"setupInitialStates"};

    std::vector<double> imuOrientation(4);
    if (!setupFixedVectorParamPrivate("imu_orientation_quaternion_wxyz", printPrefix, handler, imuOrientation)) { return false; }

    std::vector<double> imuPosition(3);
    if (!setupFixedVectorParamPrivate("imu_position_xyz", printPrefix, handler, imuPosition)) { return false; }

    std::vector<double> imuLinearVelocity(3);
    if (!setupFixedVectorParamPrivate("imu_linear_velocity_xyz", printPrefix, handler, imuLinearVelocity)) { return false; }

    std::vector<double> lContactFrameOrientation(4);
    if (!setupFixedVectorParamPrivate("l_contact_frame_orientation_quaternion_wxyz", printPrefix, handler, lContactFrameOrientation)) { return false; }

    std::vector<double> lContactFramePosition(3);
    if (!setupFixedVectorParamPrivate("l_contact_frame_position_xyz", printPrefix, handler, lContactFramePosition)) { return false; }

    std::vector<double> rContactFrameOrientation(4);
    if (!setupFixedVectorParamPrivate("r_contact_frame_orientation_quaternion_wxyz", printPrefix, handler, rContactFrameOrientation)) { return false; }

    std::vector<double> rContactFramePosition(3);
    if (!setupFixedVectorParamPrivate("r_contact_frame_position_xyz", printPrefix, handler, rContactFramePosition)) { return false; }

    if (m_options.imuBiasEstimationEnabled)
    {
        std::vector<double> accelerometerBias(3);
        if (!setupFixedVectorParamPrivate("accelerometer_bias", printPrefix, handler, accelerometerBias)) { return false; }

        std::vector<double> gyroscopeBias(3);
        if (!setupFixedVectorParamPrivate("gyroscope_bias", printPrefix, handler, gyroscopeBias)) { return false; }

        m_statePrev.accelerometerBias << accelerometerBias[0], accelerometerBias[1], accelerometerBias[2];
        m_statePrev.gyroscopeBias << gyroscopeBias[0], gyroscopeBias[1], gyroscopeBias[2];
    }

    m_statePrev.imuOrientation = Eigen::Quaterniond(imuOrientation[0], imuOrientation[1], imuOrientation[2], imuOrientation[3]); // here loaded as w x y z
    m_statePrev.imuOrientation.normalize(); // normalize the user defined quaternion to respect internal tolerances for unit norm constraint
    m_statePrev.imuPosition << imuPosition[0], imuPosition[1], imuPosition[2];
    m_statePrev.imuLinearVelocity << imuLinearVelocity[0], imuLinearVelocity[1], imuLinearVelocity[2];


    m_statePrev.lContactFrameOrientation = Eigen::Quaterniond(lContactFrameOrientation[0], lContactFrameOrientation[1], lContactFrameOrientation[2], lContactFrameOrientation[3]);  // here loaded as w x y z
    m_statePrev.lContactFrameOrientation.normalize(); // normalize the user defined quaternion to respect internal tolerances for unit norm constraint
    m_statePrev.lContactFramePosition << lContactFramePosition[0], lContactFramePosition[1], lContactFramePosition[2];

    m_statePrev.rContactFrameOrientation = Eigen::Quaterniond(rContactFrameOrientation[0], rContactFrameOrientation[1], rContactFrameOrientation[2], rContactFrameOrientation[3]);  // here loaded as w x y z
    m_statePrev.rContactFrameOrientation.normalize(); // normalize the user defined quaternion to respect internal tolerances for unit norm constraint
    m_statePrev.rContactFramePosition << rContactFramePosition[0], rContactFramePosition[1], rContactFramePosition[2];

    m_state = m_statePrev;

    return true;
}

bool FloatingBaseEstimator::setupPriorDevs(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[FloatingBaseEstimator::setupPriorDevs] The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    std::string printPrefix{"setupPriorDevs"};

    std::vector<double> imuOrientation(3);
    if (!setupFixedVectorParamPrivate("imu_orientation", printPrefix, handler, imuOrientation)) { return false; }

    std::vector<double> imuPosition(3);
    if (!setupFixedVectorParamPrivate("imu_position", printPrefix, handler, imuPosition)) { return false; }

    std::vector<double> imuLinearVelocity(3);
    if (!setupFixedVectorParamPrivate("imu_linear_velocity", printPrefix, handler, imuLinearVelocity)) { return false; }

    std::vector<double> lContactFrameOrientation(3);
    if (!setupFixedVectorParamPrivate("l_contact_frame_orientation", printPrefix, handler, lContactFrameOrientation)) { return false; }

    std::vector<double> lContactFramePosition(3);
    if (!setupFixedVectorParamPrivate("l_contact_frame_position", printPrefix, handler, lContactFramePosition)) { return false; }

    std::vector<double> rContactFrameOrientation(3);
    if (!setupFixedVectorParamPrivate("r_contact_frame_orientation", printPrefix, handler, rContactFrameOrientation)) { return false; }

    std::vector<double> rContactFramePosition(3);
    if (!setupFixedVectorParamPrivate("r_contact_frame_position", printPrefix, handler, rContactFramePosition)) { return false; }

    if (m_options.imuBiasEstimationEnabled)
    {
        std::vector<double> accelerometerBias(3);
        if (!setupFixedVectorParamPrivate("accelerometer_bias", printPrefix, handler, accelerometerBias)) { return false; }

        std::vector<double> gyroscopeBias(3);
        if (!setupFixedVectorParamPrivate("gyroscope_bias", printPrefix, handler, gyroscopeBias)) { return false; }

        m_priors.accelerometerBias << accelerometerBias[0], accelerometerBias[1], accelerometerBias[2];
        m_priors.gyroscopeBias << gyroscopeBias[0], gyroscopeBias[1], gyroscopeBias[2];
    }

    m_priors.imuOrientation << imuOrientation[0], imuOrientation[1], imuOrientation[2];
    m_priors.imuPosition << imuPosition[0], imuPosition[1], imuPosition[2];
    m_priors.imuLinearVelocity << imuLinearVelocity[0], imuLinearVelocity[1], imuLinearVelocity[2];


    m_priors.lContactFrameOrientation << lContactFrameOrientation[0], lContactFrameOrientation[1], lContactFrameOrientation[2];
    m_priors.lContactFramePosition << lContactFramePosition[0], lContactFramePosition[1], lContactFramePosition[2];

    m_priors.rContactFrameOrientation << rContactFrameOrientation[0], rContactFrameOrientation[1], rContactFrameOrientation[2];
    m_priors.rContactFramePosition << rContactFramePosition[0], rContactFramePosition[1], rContactFramePosition[2];

    return true;
}


bool FloatingBaseEstimator::setupFixedVectorParamPrivate(const std::string& param, const std::string& prefix,
                                                         std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                                         std::vector<double>& vec)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        return false;
    }

    if (!handle->getParameter(param, GenericContainer::make_vector(vec, GenericContainer::VectorResizeMode::Fixed)))
    {
        std::cerr << "[FloatingBaseEstimator::" << prefix << "] The parameter handler could not find \""<< param <<"\" in the configuration file. This is a required parameter." << std::endl;
        return false;
    }

    return true;
}

bool FloatingBaseEstimator::updateBaseStateFromIMUState(const FloatingBaseEstimators::InternalState& state,
                                                        const FloatingBaseEstimators::Measurements& meas,
                                                        iDynTree::Transform& basePose,
                                                        iDynTree::Twist& baseTwist)
{
    iDynTree::Position imuPosition;
    iDynTree::toEigen(imuPosition) = state.imuPosition;
    iDynTree::Rotation imuRotation;
    iDynTree::toEigen(imuRotation) = state.imuOrientation.toRotationMatrix();
    auto A_H_IMU = iDynTree::Transform(imuRotation, imuPosition);

    iDynTree::Twist v_IMU;
    iDynTree::Vector3 imuLinVel, imuAngVel;
    iDynTree::toEigen(imuLinVel) = state.imuLinearVelocity;
    if (m_useIMUForAngVelEstimate)
    {
        iDynTree::toEigen(imuAngVel) = state.imuOrientation.toRotationMatrix()*(meas.gyro - state.gyroscopeBias);
    }
    else
    {
        iDynTree::toEigen(imuAngVel) = state.imuAngularVelocity;
    }
    v_IMU.setLinearVec3(imuLinVel);
    v_IMU.setAngularVec3(imuAngVel);

    iDynTree::Twist tempTwist;
    if (!m_modelComp.getBaseStateFromIMUState(A_H_IMU, v_IMU, basePose, tempTwist))
    {
        std::cerr << "[FloatingBaseEstimator::updateBaseStateFromIMUState]" << " Failed to get base link state from IMU state"
                  << std::endl;
        return false;
    }
    
    if (m_useIMUVelForBaseVelComputation)
    {
        baseTwist = tempTwist;
    }
    
    if (!m_modelComp.kinDyn()->setRobotState(iDynTree::toEigen(basePose.asHomogeneousTransform()), 
                                             iDynTree::make_span(meas.encoders.data(), meas.encoders.size()),
                                             iDynTree::toEigen(baseTwist), 
                                             iDynTree::make_span(meas.encodersSpeed.data(), meas.encodersSpeed.size()),
                                             iDynTree::make_span(m_options.accelerationDueToGravity.data(), m_options.accelerationDueToGravity.size())))
    {
        std::cerr << "[FloatingBaseEstimator::updateBaseStateFromIMUState]" << " Failed to get kindyncomputations robot state"
                  << std::endl;
        return false;
    }

    return true;
}

bool FloatingBaseEstimator::resetEstimator(const Eigen::Quaterniond& newBaseOrientation,
                                           const Eigen::Vector3d& newBasePosition)
{
    iDynTree::Rotation Rb;
    iDynTree::toEigen(Rb) = newBaseOrientation.toRotationMatrix();

    iDynTree::Position pb;
    iDynTree::toEigen(pb) = newBasePosition;

    auto A_H_B = iDynTree::Transform(Rb, pb);
    iDynTree::Transform IMU_H_RF, IMU_H_LF;
    iDynTree::JointPosDoubleArray s(m_meas.encoders.size());
    toEigen(s) = m_meas.encoders;

    if (!m_modelComp.getIMU_H_feet(s, IMU_H_RF, IMU_H_LF))
    {
        std::cerr << "[FloatingBaseEstimator::resetEstimator] Could not reset estimator using new base pose." << std::endl;
        return false;
    }

    auto A_H_IMU = A_H_B*m_modelComp.base_H_IMU();;
    auto A_H_RF = A_H_IMU*IMU_H_RF;
    auto A_H_LF = A_H_IMU*IMU_H_LF;

    // convert to iDynTree Rotation to  Eigen angle axis and then to quaternion for consistent conversion
    Eigen::AngleAxisd imuAngleAxis = Eigen::AngleAxisd(iDynTree::toEigen(A_H_IMU.getRotation()));
    Eigen::AngleAxisd rfAngleAxis = Eigen::AngleAxisd(iDynTree::toEigen(A_H_RF.getRotation()));
    Eigen::AngleAxisd lfAngleAxis = Eigen::AngleAxisd(iDynTree::toEigen(A_H_LF.getRotation()));

    m_state.imuOrientation = Eigen::Quaterniond(imuAngleAxis);
    m_state.imuPosition = iDynTree::toEigen(A_H_IMU.getPosition());
    m_state.rContactFrameOrientation = Eigen::Quaterniond(rfAngleAxis);
    m_state.rContactFramePosition = iDynTree::toEigen(A_H_RF.getPosition());
    m_state.lContactFrameOrientation = Eigen::Quaterniond(lfAngleAxis);
    m_state.lContactFramePosition = iDynTree::toEigen(A_H_LF.getPosition());

    return true;
}

bool FloatingBaseEstimator::resetEstimator(const FloatingBaseEstimators::InternalState& newState)
{
    m_state = newState;
    return true;
}

