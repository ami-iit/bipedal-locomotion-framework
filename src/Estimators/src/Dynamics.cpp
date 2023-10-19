/**
 * @file Dynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ParametersHandler;

const UKFInput& UkfInputProvider::getOutput() const
{
    return m_ukfInput;
}

bool UkfInputProvider::advance()
{
    return true;
}

bool UkfInputProvider::setInput(const UKFInput& input)
{
    m_ukfInput = input;

    return true;
}

bool UkfInputProvider::isOutputValid() const
{
    return m_ukfInput.robotJointPositions.size() != 0;
}

bool Dynamics::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> /**paramHandler**/)
{
    return true;
}

bool Dynamics::finalize(const System::VariablesHandler& /**stateVariableHandler**/)
{
    return true;
}

bool Dynamics::setSubModels(const std::vector<SubModel>& /**subModelList**/, const std::vector<std::shared_ptr<KinDynWrapper>>& /**kinDynWrapperList**/)
{
    return true;
}

bool Dynamics::update()
{
    return true;
}

Eigen::Ref<const Eigen::VectorXd> Dynamics::getUpdatedVariable() const
{
    return m_updatedVariable;
}

int Dynamics::size() const
{
    return m_size;
}

Eigen::Ref<const Eigen::VectorXd> Dynamics::getCovariance()
{
    return m_covariances;
}

bool Dynamics::checkStateVariableHandler()
{
    return true;
}

Eigen::Ref<const Eigen::VectorXd> Dynamics::getInitialStateCovariance()
{
    return m_initialCovariances;
}
