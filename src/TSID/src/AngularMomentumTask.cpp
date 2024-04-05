/**
 * @file AngularMomentumTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TSID/AngularMomentumTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

bool AngularMomentumTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[AngularMomentumTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool AngularMomentumTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[AngularMomentumTask::setVariablesHandler]";
    constexpr auto expectedWrenchSize{6};

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return false;
    }

    for (auto& contact : m_contactWrenches)
    {
        if (!variablesHandler.getVariable(contact.variable.name, contact.variable))
        {
            log()->error("{} Error while retrieving the contact variable named {}.",
                         errorPrefix,
                         contact.variable.name);
            return false;
        }

        if (contact.variable.size != expectedWrenchSize)
        {
            log()->error("{} The variable size associated to the contact named {} is different "
                         "from {}.",
                         errorPrefix,
                         contact.variable.name,
                         expectedWrenchSize);
            return false;
        }
    }

    // resize the matrices
    m_A.resize(m_angularMomentumSize, variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(m_angularMomentumSize);

    // set the const part of the matrix A
    for (const auto& contactWrench : m_contactWrenches)
    {
        iDynTree::toEigen(this->subA(contactWrench.variable)).rightCols<3>().setIdentity();
    }

    return true;
}

bool AngularMomentumTask::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[AngularMomentumTask::initialize]";

    if (m_kinDyn == nullptr || !m_kinDyn->isValid())
    {
        log()->error("{} KinDynComputations object is not valid.", errorPrefix);
        return false;
    }

    if (m_kinDyn->getFrameVelocityRepresentation()
        != iDynTree::FrameVelocityRepresentation::MIXED_REPRESENTATION)
    {
        log()->error("{} The task supports only quantities expressed in MIXED representation. "
                     "Please provide a KinDynComputations with Frame velocity representation set "
                     "to MIXED_REPRESENTATION.",
                     errorPrefix);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    double kp{0};
    if (!ptr->getParameter("kp", kp))
    {
        log()->error("{} Unable to load the proportional gain.", errorPrefix);
        return false;
    }
    m_R3Controller.setGains(kp);

    int numberOfContacts = 0;
    ptr->getParameter("max_number_of_contacts", numberOfContacts);

    m_contactWrenches.resize(numberOfContacts);

    for (int i = 0; i < numberOfContacts; i++)
    {
        auto groupWeak = ptr->getGroup("CONTACT_" + std::to_string(i));
        auto group = groupWeak.lock();
        if (group == nullptr)
        {
            log()->error("{} The group named CONTACT_{} does not exist.", errorPrefix, i);
            return false;
        }

        std::string frameName;
        if (!group->getParameter("frame_name", frameName)
            || (m_contactWrenches[i].frameIndex = m_kinDyn->model().getFrameIndex(frameName))
                   == iDynTree::FRAME_INVALID_INDEX)
        {
            log()->error("{} Error while retrieving the frame associated to the CONTACT_{}.",
                         errorPrefix,
                         i);
            return false;
        }

        if (!group->getParameter("variable_name", m_contactWrenches[i].variable.name))
        {
            log()->error("{} Error while retrieving the variable name associated to the "
                         "CONTACT_{}.",
                         errorPrefix,
                         i);
            return false;
        }
    }

    // set the description
    m_description = "Angular momentum task.";

    // reset the jacobian
    constexpr auto spatialVelocitySize{6};

    m_isInitialized = true;

    return true;
}

bool AngularMomentumTask::update()
{
    namespace iDyn = ::iDynTree;

    m_isValid = false;

    // update the control law
    m_R3Controller.setState(iDyn::toEigen(m_kinDyn->getCentroidalTotalMomentum().getAngularVec3()));
    m_R3Controller.computeControlLaw();
    m_b = m_R3Controller.getControl().coeffs();

    const Eigen::Vector3d comPosition = iDyn::toEigen(m_kinDyn->getCenterOfMassPosition());

    for (const auto& contactWrench : m_contactWrenches)
    {
        iDyn::toEigen(this->subA(contactWrench.variable)).leftCols<3>() = iDyn::skew(
            iDyn::toEigen(m_kinDyn->getWorldTransform(contactWrench.frameIndex).getPosition())
            - comPosition);
    }

    m_isValid = true;
    return m_isValid;
}

bool AngularMomentumTask::setSetPoint(Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                                      Eigen::Ref<const Eigen::Vector3d> angularMomentumDerivative)
{
    bool ok = m_R3Controller.setFeedForward(angularMomentumDerivative);
    ok = ok && m_R3Controller.setDesiredState(angularMomentum);
    return ok;
}

std::size_t AngularMomentumTask::size() const
{
    return m_angularMomentumSize;
}

AngularMomentumTask::Type AngularMomentumTask::type() const
{
    return Type::equality;
}

bool AngularMomentumTask::isValid() const
{
    return m_isValid;
}
