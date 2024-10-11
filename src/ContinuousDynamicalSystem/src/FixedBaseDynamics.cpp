/**
 * @file FixedBaseDynamics.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/Model.h>
#include <iDynTree/Twist.h>
#include <iDynTree/EigenHelpers.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FixedBaseDynamics.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::ContinuousDynamicalSystem;
using namespace BipedalLocomotion::ParametersHandler;

bool FixedBaseDynamics::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[FixedBaseDynamics::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is expired. Please call the function passing a "
                     "pointer pointing an already allocated memory.",
                     logPrefix);
        return false;
    }

    if (!ptr->getParameter("gravity", m_gravity))
    {
        log()->info("{} The gravity vector is not found. The default one will be "
                    "used {}.",
                    logPrefix,
                    m_gravity.transpose());
    }

    if (ptr->getParameter("base_link", m_robotBase))
    {
        if (m_kinDyn.isValid())
        {
            if (!m_kinDyn.setFloatingBase(m_robotBase))
            {
                log()->error("{} Unable to set the floating base link named: {}.",
                             logPrefix,
                             m_robotBase);
                return false;
            }
        }
    } else
    {
        log()->info("{} The base_link name is not found. The default one stored in the model will "
                    "be used.",
                    logPrefix);
    }

    return true;
}

bool FixedBaseDynamics::setRobotModel(const iDynTree::Model& model)
{
    constexpr auto logPrefix = "[FixedBaseDynamics::setRobotModel]";
    constexpr std::size_t spatialVelocitySize = 6;

    if (!m_kinDyn.loadRobotModel(model))
    {
        log()->error("{} Unable to load the robot model.", logPrefix);
        return false;
    }

    // if the floating base name string is not empty it will be set the in model
    if(!m_robotBase.empty())
    {
        log()->info("{} Trying to set the floating base named {} into the kinDynComputations "
                    "object.",
                    logPrefix,
                    m_robotBase);
        if (!m_kinDyn.setFloatingBase(m_robotBase))
        {
            log()->error("{} Unable to set the floating base named {}.", logPrefix, m_robotBase);
            return false;
        }
    } else
    {
        log()->info("{} The following link will be used as robot base: {}.",
                    logPrefix,
                    m_kinDyn.getFloatingBase());
    }

    m_actuatedDoFs = model.getNrOfDOFs();

    // resize matrices
    m_massMatrix.resize(m_actuatedDoFs + spatialVelocitySize, m_actuatedDoFs + spatialVelocitySize);
    m_knownCoefficent.resize(m_actuatedDoFs + spatialVelocitySize);

    return true;
}

bool FixedBaseDynamics::setMassMatrixRegularization(const Eigen::Ref<const Eigen::MatrixXd>& matrix)
{
    constexpr auto logPrefix = "[FixedBaseDynamics::setMassMatrixRegularization]";

    m_useMassMatrixRegularizationTerm = false;

    if (!m_kinDyn.isValid())
    {
        log()->error("{} Please call setRobotModel() before.", logPrefix);
        return false;
    }

    if ((m_actuatedDoFs != matrix.rows()) || (matrix.cols() != matrix.rows()))
    {
        log()->error("{} The size of the regularization matrix is not correct. The correct size "
                     "is:  {} x {}. While the input of the function is a {} x {} matrix.",
                     logPrefix,
                     m_actuatedDoFs,
                     m_actuatedDoFs,
                     matrix.rows(),
                     matrix.cols());
        return false;
    }

    m_massMatrixReglarizationTerm = matrix;
    m_useMassMatrixRegularizationTerm = true;

    return true;
}

bool FixedBaseDynamics::dynamics(const std::chrono::nanoseconds& time, StateDerivative& stateDerivative)
{
    constexpr auto logPrefix = "[FixedBaseDynamics::dynamics]";

    if (!m_kinDyn.isValid())
    {
        log()->error("{} Please call setRobotModel() before.", logPrefix);
        return false;
    }

    // get the state
    const auto& [jointVelocity, jointPositions] = m_state;

    auto& [jointAcceleration, jointVelocityOutput] = stateDerivative;

    const auto& [jointTorques] = m_controlInput;

    // check the size of the vectors
    if (jointVelocity.size() != m_actuatedDoFs || jointPositions.size() != m_actuatedDoFs
        || jointTorques.size() != m_actuatedDoFs)
    {
        log()->error("{} Wrong size of the vectors.", logPrefix);
        return false;
    }


    jointVelocityOutput = jointVelocity;

    // update kindyncomputations object
    if (!m_kinDyn.setRobotState(jointPositions, jointVelocity, m_gravity))
    {
        log()->error("{} Unable to update the kinDyn object.", logPrefix);
        return false;
    }

    // compute the mass matrix
    if (!m_kinDyn.getFreeFloatingMassMatrix(m_massMatrix))
    {
        log()->error("{} Unable to get the mass matrix.", logPrefix);
        return false;
    }

    // compute the generalized bias forces
    // here we want to compute the robot acceleration as
    // robotAcceleration = M^-1 (-h + J' F + tau) = M^-1 * m_knownCoefficent

    if (!m_kinDyn.generalizedBiasForces(m_knownCoefficent))
    {
        log()->error("{} Unable to get the bias forces.", logPrefix);
        return false;
    }

    m_knownCoefficent *= -1;
    m_knownCoefficent.tail(m_actuatedDoFs) += jointTorques;

    // compute the joint acceleration solving the linear system. Here we assume that the
    // mass matrix is positive definite (check here for further informations:
    // https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)
    if (m_useMassMatrixRegularizationTerm)
    {
        jointAcceleration = (m_massMatrix.bottomRightCorner(m_actuatedDoFs, m_actuatedDoFs)
                             + m_massMatrixReglarizationTerm)
                                .llt()
                                .solve(m_knownCoefficent.tail(m_actuatedDoFs));
    } else
    {
        jointAcceleration = m_massMatrix.bottomRightCorner(m_actuatedDoFs, m_actuatedDoFs)
                                .llt()
                                .solve(m_knownCoefficent.tail(m_actuatedDoFs));
    }

    return true;
}

bool FixedBaseDynamics::setState(const State& state)
{
    m_state = state;
    return true;
}

const FixedBaseDynamics::State& FixedBaseDynamics::getState() const
{
    return m_state;
}

bool FixedBaseDynamics::setControlInput(const Input& controlInput)
{
    m_controlInput = controlInput;
    return true;
}
