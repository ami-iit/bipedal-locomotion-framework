/**
 * @file ContactModel.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContactModels/ContactModel.h>

using namespace BipedalLocomotion::ContactModels;

bool ContactModel::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    // the parameters has been update the previous quantities has to be evaluated again
    m_isContactWrenchComputed = false;
    m_isControlMatrixComputed = false;
    m_isAutonomousDynamicsComputed = false;
    m_isRegressorComputed = false;

    return initializePrivate(handler);
}

void ContactModel::setNullForceTransform(const iDynTree::Transform& nullForceTransform)
{
    // the state has been update the previous quantities has to be evaluated again
    m_isContactWrenchComputed = false;
    m_isControlMatrixComputed = false;
    m_isAutonomousDynamicsComputed = false;
    m_isRegressorComputed = false;


    setNullForceTransformPrivate(nullForceTransform);
}

void ContactModel::setState(const iDynTree::Twist& twist,
                            const iDynTree::Transform& transform)

{
    // the state has been update the previous quantities has to be evaluated again
    m_isContactWrenchComputed = false;
    m_isControlMatrixComputed = false;
    m_isAutonomousDynamicsComputed = false;
    m_isRegressorComputed = false;

    setStatePrivate(twist, transform);

    return;
}

const iDynTree::Wrench& ContactModel::getContactWrench()
{
    if (!m_isContactWrenchComputed)
    {
        computeContactWrench();
        m_isContactWrenchComputed = true;
    }

    return m_contactWrench;
}

const iDynTree::Vector6& ContactModel::getAutonomousDynamics()
{
    if (!m_isAutonomousDynamicsComputed)
    {
        computeAutonomousDynamics();
        m_isAutonomousDynamicsComputed = true;
    }

    return m_autonomousDynamics;
}

const iDynTree::Matrix6x6& ContactModel::getControlMatrix()
{
    if (!m_isControlMatrixComputed)
    {
        computeControlMatrix();
        m_isControlMatrixComputed = true;
    }

    return m_controlMatrix;
}

const iDynTree::MatrixDynSize& ContactModel::getRegressor()
{
    if (!m_isRegressorComputed)
    {
        computeRegressor();
        m_isRegressorComputed = true;
    }

    return m_regressor;
}
