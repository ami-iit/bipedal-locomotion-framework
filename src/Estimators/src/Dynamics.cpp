/**
 * @file Dynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>

namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ParametersHandler;

const RDE::UKFInput& RDE::UkfInputProvider::getOutput() const
{
    return m_ukfInput;
}

bool RDE::UkfInputProvider::advance()
{
    return true;
}

bool RDE::UkfInputProvider::setInput(const UKFInput& input)
{
    m_ukfInput = input;

    return true;
}

bool RDE::UkfInputProvider::isOutputValid() const
{
    return m_ukfInput.robotJointPositions.size() != 0;
}

bool RDE::Dynamics::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> /*paramHandler*/)
{
    return true;
}

bool RDE::Dynamics::finalize(const System::VariablesHandler& /*stateVariableHandler*/)
{
    return true;
}

bool RDE::Dynamics::setSubModels(const std::vector<SubModel>& /*subModelList*/, const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& /*kinDynWrapperList*/)
{
    return true;
}

bool RDE::Dynamics::update()
{
    return true;
}

Eigen::Ref<const Eigen::VectorXd> RDE::Dynamics::getUpdatedVariable() const
{
    return m_updatedVariable;
}

int RDE::Dynamics::size() const
{
    return m_size;
}

Eigen::Ref<const Eigen::VectorXd> RDE::Dynamics::getCovariance()
{
    return m_covariances;
}

bool RDE::Dynamics::checkStateVariableHandler()
{
    return true;
}

Eigen::Ref<const Eigen::VectorXd> RDE::Dynamics::getInitialStateCovariance()
{
    return m_initialCovariances;
}
