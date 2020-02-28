/**
 * @file Simulator.tpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_SIMULATOR_TPP
#define BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_SIMULATOR_TPP

#include <BipedalLocomotionControllers/Simulator/Simulator.h>

namespace BipedalLocomotionControllers
{
namespace Simulator
{

template <class Derived>
bool Simulator::initialize(std::weak_ptr<ParametersHandler::IParametersHandler<Derived>> handlerWeak)
{
    auto handler = handlerWeak.lock();
    if(handler == nullptr)
    {
        std::cerr << "[Simulator::initialize] The parameter handler is not valid."
                  << std::endl;
        return false;
    }

    if(m_state != State::NotInitialized)
    {
        std::cerr << "[Simulator::initialize] The simulator has been already initialized."
                  << std::endl;
        return false;
    }

    // get contact models parameters
    if (!handler->getParameter("length", m_length) || !handler->getParameter("width", m_width)
        || !handler->getParameter("spring_coeff", m_springCoeff)
        || !handler->getParameter("damper_coeff", m_damperCoeff))
    {
        std::cerr << "[Simulator::initialize] Unable to get the contact parameters." << std::endl;
        return false;
    }

    const std::unordered_map<std::string, std::any> parameters({{"length", m_length},
                                                                {"width", m_width},
                                                                {"spring_coeff", m_springCoeff},
                                                                {"damper_coeff", m_damperCoeff}});

    m_leftContact.model = std::make_unique<ContactModels::ContinuousContactModel>(parameters);
    m_rightContact.model = std::make_unique<ContactModels::ContinuousContactModel>(parameters);

    // get the contact frames
    std::string footFrame;
    if (!handler->getParameter("left_foot_frame", footFrame))
    {
        std::cerr << "[Simulator::initialize] Unable to get the frame name." << std::endl;
        return false;
    }
    m_leftContact.indexInTheModel = m_kinDyn.model().getFrameIndex(footFrame);
    if (m_leftContact.indexInTheModel == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[Simulator::initialize] Unable to find the frame named: " << footFrame
                  << std::endl;
        return false;
    }

    if (!handler->getParameter("right_foot_frame", footFrame))
    {
        std::cerr << "[Simulator::initialize] Unable to get the frame name." << std::endl;
        return false;
    }
    m_rightContact.indexInTheModel = m_kinDyn.model().getFrameIndex(footFrame);
    if (m_rightContact.indexInTheModel == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << "[Simulator::initialize] Unable to find the frame named: " << footFrame
                  << std::endl;
        return false;
    }

    if (!setBaseFrame(m_leftContact.indexInTheModel, "left_foot"))
    {
        std::cerr << "[Simulator::initialize] Unable to set the leftFootFrame." << std::endl;
        return false;
    }

    // set the right foot frame
    if (!setBaseFrame(m_rightContact.indexInTheModel, "right_foot"))
    {
        std::cerr << "[Simulator::initialize] Unable to set the rightFootFrame." << std::endl;
        return false;
    }

    // get the base frame
    std::string baseFrame;
    if (!handler->getParameter("base_frame", baseFrame))
    {
        std::cerr << "[Simulator::initialize] Unable to get the frame name." << std::endl;
        return false;
    }
    m_baseFrame = m_kinDyn.model().getFrameIndex(baseFrame);

    if (!setBaseFrame(m_baseFrame, "root"))
    {
        std::cerr << "[Simulator::initialize] Unable to set the root frame." << std::endl;
        return false;
    }

    if (!handler->getParameter("sampling_time", m_dT))
    {
        std::cerr << "[Simulator::initialize] Unable to get the sampling time." << std::endl;
        return false;
    }

    std::string controlMode;
    if (!handler->getParameter("control_mode", controlMode))
    {
        std::cerr << "[Simulator::initialize] Unable to get the control mode." << std::endl;
        return false;
    }

    if (controlMode == "torque")
        m_controlMode = ControlMode::Torque;
    else if (controlMode == "acceleration")
        m_controlMode = ControlMode::Acceleration;
    else
    {
        std::cerr << "[Simulator::initialize]  The control mode selected is not supported"
                  << std::endl;
        return false;
    }

    // instantiate the integrates
    m_jointVelocityIntegrator = std::make_unique<Integrator<iDynTree::VectorDynSize>>(m_dT);
    m_jointPositionIntegrator = std::make_unique<Integrator<iDynTree::VectorDynSize>>(m_dT);

    m_baseLinearVelocityIntegrator
        = std::make_unique<Integrator<iDynTree::LinearMotionVector3>>(m_dT);
    m_baseAngularVelocityIntegrator
        = std::make_unique<Integrator<iDynTree::AngularMotionVector3>>(m_dT);

    m_basePositionIntegrator = std::make_unique<Integrator<iDynTree::Vector3>>(m_dT);
    m_baseRotationIntegrator = std::make_unique<Integrator<iDynTree::Matrix3x3>>(m_dT);

    m_state = State::Initialized;

    return true;
}

} // namespace Simulator
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_SIMULATOR_SIMULATOR_TPP
