/**
 * @file FloatingBaseEstimator.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */


#include <BipedalLocomotion/Estimators/FloatingBaseEstimator.h>

using namespace BipedalLocomotion::Estimators;

bool FloatingBaseEstimator::initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    if (!m_model_comp.modelSet())
    {
        std::cerr << "[FloatingBaseEstimator::initialize] "
        "Please call modelComputations().setModel(iDynTree::Model& model) to set the model, before calling initialize()."
        << std::endl;
        return false;
    }

    if (m_estimator_state != State::NotInitialized)
    {
        std::cerr << "[FloatingBaseEstimator::initialize] "
        "The estimator already seems to be initialized."
        << std::endl;
        return false;
    }

    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[FloatingBaseEstimator::initialize] "
        "The parameter handler has expired. Please check its scope."
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
    auto model_handle = handle->getGroup("ModelInfo");
    if (!setupModelParams(model_handle))
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

    m_estimator_state = State::Initialized;

    return true;
}

bool FloatingBaseEstimator::advance()
{
    m_state_prev = m_state;

    if (m_estimator_state != State::Initialized && m_estimator_state != State::Running)
    {
        std::cerr << "[FloatingBaseEstimator::advance] "
        "Please initialize the estimator before calling advance()."
        << std::endl;
        return false;
    }

    if (m_estimator_state == State::Initialized)
    {
        m_estimator_state = State::Running;
    }

    auto ok = predictState(m_meas, m_dt);
    if (m_options.enable_ekf_update)
    {
        ok = updateKinematics(m_meas, m_dt);
    }

    return ok;
}

bool FloatingBaseEstimator::ModelComputations::setModel(const iDynTree::Model& model)
{
    if (!m_kindyn.loadRobotModel(model))
    {
        return false;
    }

    m_model = model;
    m_nr_joints = model.getNrOfDOFs();
    m_model_set = true;
    return true;
}

bool FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU(const std::string& base_link,
                                                                 const std::string& imu_frame)
{
    m_base_link_idx = m_model.getFrameIndex(base_link);
    if (m_base_link_idx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] "
        "Specified base link not available in the loaded URDF Model."
        << std::endl;
        return false;
    }

    m_base_imu_idx = m_model.getFrameIndex(imu_frame);
    if (m_base_imu_idx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] "
        "Specified IMU frame not available in the loaded URDF Model."
        << std::endl;
        return false;
    }

    if (m_base_link_idx != m_model.getFrameLink(m_base_imu_idx))
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setBaseLinkAndIMU] "
        "Specified IMU not rigidly attached to the base link. Please specify a base link colocated IMU."
        << std::endl;
        return false;
    }


    if (m_model.getDefaultBaseLink() != m_base_link_idx)
    {
        m_model.setDefaultBaseLink(m_base_link_idx);
        this->setModel(m_model);
    }

    m_base_link = base_link;
    m_base_imu_frame = imu_frame;
    m_base_H_imu = m_model.getFrameTransform(m_base_imu_idx);
    return true;
}

bool FloatingBaseEstimator::ModelComputations::setFeetContactFrames(const std::string& l_foot_contact_frame,
                                                                    const std::string& r_foot_contact_frame)
{
    m_l_foot_contact_idx = m_model.getFrameIndex(l_foot_contact_frame);
    if (m_l_foot_contact_idx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setFeetContactFrames] "
        "Specified left foot contact frame not available in the loaded URDF Model."
        << std::endl;
        return false;
    }

    m_r_foot_contact_idx = m_model.getFrameIndex(r_foot_contact_frame);
    if (m_r_foot_contact_idx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::setFeetContactFrames] "
        "Specified right foot contact frame not available in the loaded URDF Model."
        << std::endl;
        return false;
    }

    m_l_foot_contact_frame = l_foot_contact_frame;
    m_r_foot_contact_frame = r_foot_contact_frame;

    return true;
}

bool FloatingBaseEstimator::ModelComputations::checkModelInfoLoaded()
{
    bool loaded = (m_base_link_idx != iDynTree::FRAME_INVALID_INDEX) &&
             (m_base_imu_idx != iDynTree::FRAME_INVALID_INDEX) &&
             (m_l_foot_contact_idx != iDynTree::FRAME_INVALID_INDEX) &&
             (m_r_foot_contact_idx != iDynTree::FRAME_INVALID_INDEX);

    return loaded;
}

bool FloatingBaseEstimator::ModelComputations::getBaseStateFromIMUState(const iDynTree::Transform& A_H_IMU,
                                                                        const iDynTree::Twist& v_IMU,
                                                                        iDynTree::Transform& A_H_B,
                                                                        iDynTree::Twist& v_B)
{
    if (!checkModelInfoLoaded())
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::getBaseStateFromIMUState] "
        "Please set required model info parameters, before calling getBaseStateFromIMUState(...)"
        << std::endl;
        return false;
    }

    A_H_B = A_H_IMU*(m_base_H_imu.inverse());

    // transform velocity (mixed-representation)
    auto base2IMUinWorld = A_H_B.getRotation()*m_base_H_imu.getPosition();
    auto X = iDynTree::Transform(iDynTree::Rotation::Identity(), base2IMUinWorld);
    v_B = X*v_IMU;

    return true;
}

bool FloatingBaseEstimator::ModelComputations::getIMU_H_feet(const iDynTree::JointPosDoubleArray& encoders,
                                                             iDynTree::Transform& IMU_H_l_foot,
                                                             iDynTree::Transform& IMU_H_r_foot)
{
    if (!checkModelInfoLoaded())
    {
        std::cerr << "[FloatingBaseEstimator::ModelComputations::getIMU_H_feet] "
        "Please set required model info parameters, before calling getIMU_H_feet(...)"
        << std::endl;
        return false;
    }

    m_kindyn.setJointPos(encoders);
    IMU_H_l_foot = m_kindyn.getRelativeTransform(m_base_imu_idx, m_l_foot_contact_idx);
    IMU_H_r_foot = m_kindyn.getRelativeTransform(m_base_imu_idx, m_r_foot_contact_idx);

    return true;
}

bool FloatingBaseEstimator::setIMUMeasurement(const Eigen::Vector3d& acc_meas,
                                              const Eigen::Vector3d& gyro_meas)
{
    m_meas.acc = acc_meas;
    m_meas.gyro = gyro_meas;
    return true;
}


bool FloatingBaseEstimator::setKinematics(const Eigen::VectorXd& encoders,
                                          const Eigen::VectorXd& encoder_speeds)
{
    if ( (encoders.size() != encoder_speeds.size()) ||
        (encoders.size() != modelComputations().nrJoints()))
    {
        std::cerr << "[FloatingBaseEstimator::setKinematics] "
        "kinematic measurements size mismatch"
        << std::endl;
        return false;
    }

    m_meas.encoders = encoders;
    m_meas.encoders_speed = encoder_speeds;
    return true;
}

bool FloatingBaseEstimator::setContacts(const bool& lf_contact,
                                        const bool& rf_contact)
{
    m_meas.lf_contact = lf_contact;
    m_meas.rf_contact = rf_contact;
    return true;
}

FloatingBaseEstimator::ModelComputations& FloatingBaseEstimator::modelComputations()
{
    return m_model_comp;
}


const FBEOutput& FloatingBaseEstimator::get() const
{
    return m_estimator_out;
}


bool FloatingBaseEstimator::setupModelParams(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] "
        "The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    // setup base link and imu
    std::string base_link, imu;
    if (!handle->getParameter("base_link", base_link))
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] "
        "The parameter handler could not find \" base_link \" in the configuration file."
        << std::endl;
        return false;
    }

    if (!handle->getParameter("base_link_imu", imu))
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] "
        "The parameter handler could not find \" base_link_imu \" in the configuration file."
        << std::endl;
        return false;
    }

    // setup base lf_contact and rf_contact
    std::string lf_contact, rf_contact;
    if (!handle->getParameter("left_foot_contact_frame", lf_contact))
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] "
        "The parameter handler could not find \" base_link \" in the configuration file."
        << std::endl;
        return false;
    }

    if (!handle->getParameter("right_foot_contact_frame", rf_contact))
    {
        std::cerr << "[FloatingBaseEstimator::setupModelParams] "
        "The parameter handler could not find \" base_link \" in the configuration file."
        << std::endl;
        return false;
    }

    if (!m_model_comp.setBaseLinkAndIMU(base_link, imu))
    {
        return false;
    }

    if (!m_model_comp.setFeetContactFrames(lf_contact, rf_contact))
    {
        return false;
    }

    return true;
}


