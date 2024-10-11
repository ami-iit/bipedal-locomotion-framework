/**
 * @file FeasibleContactWrenchTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/TSID/FeasibleContactWrenchTask.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::TSID;

bool FeasibleContactWrenchTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    if ((kinDyn == nullptr) || (!kinDyn->isValid()))
    {
        log()->error("[FeasibleContactWrenchTask::setKinDyn] Invalid kinDyn object.");
        return false;
    }

    m_kinDyn = kinDyn;
    return true;
}

bool FeasibleContactWrenchTask::setVariablesHandler(const System::VariablesHandler& variablesHandler)
{
    constexpr auto errorPrefix = "[FeasibleContactWrenchTask::setVariablesHandler]";

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize method.", errorPrefix);
        return false;
    }

    if (!variablesHandler.getVariable(m_contactWrench.variable.name, m_contactWrench.variable))
    {
        log()->error("{} Error while retrieving the contact variable named {}.",
                     errorPrefix,
                     m_contactWrench.variable.name);
        return false;
    }

    if (m_contactWrench.variable.size
        != static_cast<int>(BipedalLocomotion::Math::Wrenchd::SizeAtCompileTime))
    {
        log()->error("{} The variable size associated to the contact named {} is different "
                     "from {}.",
                     errorPrefix,
                     m_contactWrench.variable.name,
                     static_cast<int>(BipedalLocomotion::Math::Wrenchd::SizeAtCompileTime));
        return false;
    }

    // the additional constrains refer to the normal force
    constexpr std::size_t normalForceFeasibilityConstraints = 2;
    const auto rowsOfConeMatrix = m_cone.getA().rows();

    // resize the matrices
    m_A.resize(normalForceFeasibilityConstraints + rowsOfConeMatrix,
               variablesHandler.getNumberOfVariables());
    m_A.setZero();
    m_b.resize(normalForceFeasibilityConstraints + rowsOfConeMatrix);

    // the vector b will contains the cone vector plus the constraint related to the contact normal
    // force
    // 0 <= fz <= max_normal_force
    // [0 0 -1 0 0 0 ] * [fx fy fz taux tauy tauz]' <= 0
    // [0 0 1 0 0 0 ] * [fx fy fz taux tauy tauz]' <= max_normal_force
    // if the task is enabled max_normal_force is equal to the max double. If the task is disabled
    // max_normal_force is equal to 0
    m_b.head(rowsOfConeMatrix) = m_cone.getB();
    m_b.tail<2>()(0) = 0;
    m_b.tail<2>()(1) = std::numeric_limits<double>::max();

    // the matrix A in body coordinate is fixed
    m_AinBodyCoordinate.resize(normalForceFeasibilityConstraints + rowsOfConeMatrix,
                               BipedalLocomotion::Math::Wrenchd::SizeAtCompileTime);
    m_AinBodyCoordinate.topRows(rowsOfConeMatrix) = m_cone.getA();
    m_AinBodyCoordinate.bottomRows<2>() << 0, 0, -1, 0, 0, 0,
                                           0, 0,  1, 0, 0, 0;

    return true;
}

bool FeasibleContactWrenchTask::initialize(std::weak_ptr<const IParametersHandler> paramHandler)
{
    constexpr auto errorPrefix = "[FeasibleContactWrenchTask::initialize]";

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

    std::string frameName;
    if (!ptr->getParameter("frame_name", frameName)
        || (m_contactWrench.frameIndex = m_kinDyn->model().getFrameIndex(frameName))
               == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Error while retrieving the frame associated to the contact.", errorPrefix);
        return false;
    }

    if (!ptr->getParameter("variable_name", m_contactWrench.variable.name))
    {
        log()->error("{} Error while retrieving the variable name associated to the contact.",
                     errorPrefix);
        return false;
    }

    // initialize the cone
    if (!m_cone.initialize(paramHandler))
    {
        log()->error("{} Unable to initialize the cone.", errorPrefix);
        return false;
    }

    // set the description
    m_description = "Feasible contact wrench - Frame: " + frameName + ".";

    m_isInitialized = true;

    return true;
}

bool FeasibleContactWrenchTask::update()
{
    constexpr auto errorPrefix = "[FeasibleContactWrenchTask::update]";

    m_isValid = false;

    if (m_kinDyn == nullptr)
    {
        log()->error("{} KinDynComputations object is not valid. Please call setKinDyn().",
                     errorPrefix);
        return false;
    }

    if (!m_contactWrench.variable.isValid())
    {
        log()->error("{} The contact wrench variable is not valid. Please call "
                     "setVariablesHandler().",
                     errorPrefix);
        return false;
    }

    if (!m_isInitialized)
    {
        log()->error("{} The task is not initialized. Please call initialize().", errorPrefix);
        return false;
    }

    using namespace iDynTree;

    m_contactWrench.frame_R_inertial
        = toEigen(m_kinDyn->getWorldTransform(m_contactWrench.frameIndex).getRotation().inverse());

    toEigen(this->subA(m_contactWrench.variable)).leftCols<3>().noalias()
        = m_AinBodyCoordinate.leftCols<3>() * m_contactWrench.frame_R_inertial;
    toEigen(this->subA(m_contactWrench.variable)).rightCols<3>().noalias()
        = m_AinBodyCoordinate.rightCols<3>() * m_contactWrench.frame_R_inertial;

    m_isValid = true;

    // no need to update b
    return m_isValid;
}

void FeasibleContactWrenchTask::setContactActive(bool isActive)
{
    if (isActive)
    {
        // the last element of the vector b can be used to disable / enable the task
        m_b.tail<1>()(0) = std::numeric_limits<double>::max();
    } else
    {
        // the last element of the vector b can be used to disable / enable the task
        m_b.tail<1>()(0) = 0;
    }
}

std::size_t FeasibleContactWrenchTask::size() const
{
    return m_A.rows();
}

FeasibleContactWrenchTask::Type FeasibleContactWrenchTask::type() const
{
    return Type::inequality;
}

bool FeasibleContactWrenchTask::isValid() const
{
    return m_isValid;
}
