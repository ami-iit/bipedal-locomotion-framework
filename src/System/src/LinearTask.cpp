/**
 * @file LinearTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/LinearTask.h>

using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::System;

iDynTree::MatrixView<double>
LinearTask::subA(const VariablesHandler::VariableDescription& description)
{
    return iDynTree::MatrixView<double>(m_A).block(0,
                                                   description.offset,
                                                   m_A.rows(),
                                                   description.size);
}

iDynTree::MatrixView<const double>
LinearTask::subA(const VariablesHandler::VariableDescription& description) const
{
    return iDynTree::MatrixView<const double>(m_A).block(0,
                                                         description.offset,
                                                         m_A.rows(),
                                                         description.size);
}

bool LinearTask::setVariablesHandler(const VariablesHandler& variablesHandler)
{
    return true;
}

bool LinearTask::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    return true;
}

bool LinearTask::update()
{
    return true;
}

Eigen::Ref<const Eigen::MatrixXd> LinearTask::getA() const
{
    return m_A;
}

Eigen::Ref<const Eigen::VectorXd> LinearTask::getB() const
{
    return m_b;
}

const std::string& LinearTask::getDescription() const
{
    return m_description;
}

Eigen::VectorXd LinearTask::getResidual(Eigen::Ref<const Eigen::VectorXd> solution) const
{
    Eigen::VectorXd vec = -m_b;
    vec.noalias() += m_A * solution;

    return vec;
}
