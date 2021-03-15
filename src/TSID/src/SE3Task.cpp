/**
 * @file SE3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TSID/SE3Task.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

bool SE3Task::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> paramHandler,
                         const System::VariablesHandler& variablesHandler)
{
    constexpr std::string_view errorPrefix = "[SE3Task::initialize] ";

    std::string frameName = "Unknown";
    constexpr std::string_view descriptionPrefix = "SE3Task Optimal Control Element - Frame name: ";

    if(m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        std::cerr << errorPrefix << descriptionPrefix << frameName
                  << " - KinDynComputations object is not valid." << std::endl;
        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        std::cerr << errorPrefix << descriptionPrefix << frameName
                  << " - The task supports only quantities expressed in MIXED representation. "
                     "Please provide a KinDynComputations with Frame velocity representation set "
                     "to MIXED_REPRESENTATION."
                  << std::endl;
        return false;
    }

    auto ptr = paramHandler.lock();
    if(ptr == nullptr)
    {
        std::cerr << errorPrefix << descriptionPrefix << frameName
                  << " - The parameter handler is not valid." << std::endl;
        return false;
    }

    std::string robotAccelerationVariableName;
    if (!ptr->getParameter("robot_acceleration_variable_name", robotAccelerationVariableName)
        || !variablesHandler.getVariable(robotAccelerationVariableName,
                                         m_robotAccelerationVariable))
    {
        std::cerr << errorPrefix << descriptionPrefix << frameName
                  << " - Error while retrieving the robot acceleration variable." << std::endl;
        return false;
    }

    if (m_robotAccelerationVariable.size
        != m_kinDyn->getNrOfDegreesOfFreedom() + m_spatialVelocitySize)
    {
        std::cerr << errorPrefix << descriptionPrefix << frameName
                  << " - Error while retrieving the robot acceleration variable." << std::endl;
        return false;
    }

    if (!ptr->getParameter("frame_name", frameName)
        || (m_frameIndex = m_kinDyn->model().getFrameIndex(frameName))
               == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << errorPrefix << descriptionPrefix << frameName
                  << "- Error while retrieving the frame that should be controlled." << std::endl;
        return false;
    }

    // set the gains for the controllers
    double kpLinear, kdLinear;
    double kpAngular, kdAngular;
    if (!ptr->getParameter("kp_linear", kpLinear) || !ptr->getParameter("kd_linear", kdLinear))
    {
        std::cerr << errorPrefix << descriptionPrefix << frameName
                  << " - Error while the gains for the position controller are retrieved."
                  << std::endl;
        return false;
    }

    if (!ptr->getParameter("kp_angular", kpAngular) || !ptr->getParameter("kd_angular", kdAngular))
    {
        std::cerr << errorPrefix << descriptionPrefix << frameName
                  << " - Error while the gains for the rotation controller are retrieved."
                  << std::endl;
        return false;
    }

    m_R3Controller.setGains({kpLinear, kdLinear});
    m_SO3Controller.setGains({kpAngular, kdAngular});

    // set the description
    m_description = std::string(descriptionPrefix) + frameName + ".";

    // resize the matrices
    m_A.resize(m_spatialVelocitySize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_spatialVelocitySize);

    return true;
}

bool SE3Task::update()
{
    m_b = -iDynTree::toEigen(m_kinDyn->getFrameBiasAcc(m_frameIndex));

    m_SO3Controller.setState(
        {BipedalLocomotion::Conversions::toManifRot(
             m_kinDyn->getWorldTransform(m_frameIndex).getRotation()),
         iDynTree::toEigen(m_kinDyn->getFrameVel(m_frameIndex).getAngularVec3())});

    m_R3Controller.setState(
        {iDynTree::toEigen(m_kinDyn->getWorldTransform(m_frameIndex).getPosition()),
         iDynTree::toEigen(m_kinDyn->getFrameVel(m_frameIndex).getLinearVec3())});

    m_SO3Controller.computeControlLaw();
    m_R3Controller.computeControlLaw();

    m_b.head<3>() += m_R3Controller.getControl().coeffs();
    m_b.tail<3>() += m_SO3Controller.getControl().coeffs();

    // Workaround because matrix view is not compatible with Eigen::Ref
    // https://github.com/robotology/idyntree/issues/797
    if (!m_kinDyn->getFrameFreeFloatingJacobian(m_frameIndex,
                                                this->subA(m_robotAccelerationVariable)))
    {
        std::cerr << "[SE3Task::update] Unable to get the jacobian." << std::endl;
        return false;
    }

    return true;
}

bool SE3Task::setReferenceTrajectory(const manif::SE3d& I_H_F,
                                     const manif::SE3d::Tangent& mixedVelocity,
                                     const manif::SE3d::Tangent& mixedAcceleration)
{

    bool ok = true;
    ok = ok && m_R3Controller.setDesiredState({I_H_F.translation(), mixedVelocity.lin()});
    ok = ok && m_R3Controller.setFeedForward(mixedAcceleration.lin());

    ok = ok && m_SO3Controller.setDesiredState({I_H_F.quat(), mixedVelocity.ang()});
    ok = ok && m_SO3Controller.setFeedForward(mixedAcceleration.ang());

    return ok;
}
