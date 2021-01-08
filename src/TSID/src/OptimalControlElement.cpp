/**
 * @file OptimalControlElement.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TSID/OptimalControlElement.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::TSID;

Eigen::Ref<Eigen::MatrixXd> OptimalControlElement::subA(const VariablesHandler::VariableDescription& description)
{
    return m_A.block(0, description.offset, 6, description.size);
}

bool OptimalControlElement::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    m_kinDyn = kinDyn;
    return (m_kinDyn != nullptr) && (m_kinDyn->isValid());
}

bool OptimalControlElement::initialize(std::weak_ptr<IParametersHandler> paramHandler,
                                       const VariablesHandler& variablesHandler)
{
    return true;
}

bool OptimalControlElement::update()
{
    return true;
}

Eigen::Ref<const Eigen::MatrixXd> OptimalControlElement::getA() const
{
    return m_A;
}

Eigen::Ref<const Eigen::VectorXd> OptimalControlElement::getB() const
{
    return m_b;
}

const std::string& OptimalControlElement::getDescription() const
{
    return m_description;
}
