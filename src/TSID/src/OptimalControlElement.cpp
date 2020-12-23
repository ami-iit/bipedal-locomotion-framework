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
