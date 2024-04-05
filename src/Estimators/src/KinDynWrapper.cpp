/**
 * @file KinDynWrapper.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/QR>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/MatrixView.h>
#include <iDynTree/Span.h>

// BLF
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;

bool KinDynWrapper::setModel(const SubModel & model)
{
    constexpr auto errorPrefix = "[KinDynWrapper::setModel]";

    if (!this->loadRobotModel(model.getModel()))
    {
        log()->error("{} Error while setting the robot model.", errorPrefix);
        return false;
    }

    this->setFrameVelocityRepresentation(iDynTree::BODY_FIXED_REPRESENTATION);

    m_totalBaseJointTorques.resize(6 + this->getNrOfDegreesOfFreedom());
    m_totalJointTorques.resize(this->getNrOfDegreesOfFreedom());
    m_generalizedBiasForces.resize(6 + this->getNrOfDegreesOfFreedom());
    m_FTvBaseDot.resize(this->getNrOfDegreesOfFreedom());
    m_massMatrix.resize(6 + this->getNrOfDegreesOfFreedom(), 6 + getNrOfDegreesOfFreedom());
    m_nuDot.resize(6 + this->getNrOfDegreesOfFreedom());

    return true;
}

bool KinDynWrapper::forwardDynamics(Eigen::Ref<const Eigen::VectorXd> motorTorqueAfterGearbox,
                                    Eigen::Ref<const Eigen::VectorXd> frictionTorques,
                                    Eigen::Ref<const Eigen::VectorXd> tauExt,
                                    manif::SE3d::Tangent baseAcceleration,
                                    Eigen::Ref<Eigen::VectorXd> jointAcceleration)
{
    constexpr auto errorPrefix = "[KinDynWrapper::forwardDynamics]";

    if (motorTorqueAfterGearbox.size() != this->getNrOfDegreesOfFreedom())
    {
        log()->error("{} The size of the parameter `motorTorquesAfterGearbox` should match the "
                     "number of joints.",
                     errorPrefix);
        return false;
    }

    if (frictionTorques.size() != this->getNrOfDegreesOfFreedom())
    {
        log()->error("{} The size of the parameter `frictionTorques` should match the number of "
                     "joints.",
                     errorPrefix);
        return false;
    }

    if (tauExt.size() != this->getNrOfDegreesOfFreedom())
    {
        log()->error("{} The size of the parameter `tauExt` should match the number of joints.",
                     errorPrefix);
        return false;
    }

    if (!this->generalizedBiasForces(m_generalizedBiasForces))
    {
        log()->error("{} Failed while getting the generalized bias forces.", errorPrefix);
        return false;
    }

    if (!getFreeFloatingMassMatrix(m_massMatrix))
    {
        log()->error("{} Failed while getting the mass matrix.", errorPrefix);
        return false;
    }

    m_FTvBaseDot = m_massMatrix.bottomLeftCorner(this->getNrOfDegreesOfFreedom(), 6) * baseAcceleration.coeffs();

    m_totalJointTorques = motorTorqueAfterGearbox - frictionTorques + tauExt
                          - m_generalizedBiasForces.tail(this->getNrOfDegreesOfFreedom())
                          - m_FTvBaseDot;

    jointAcceleration = m_massMatrix.block(6, 6, this->getNrOfDegreesOfFreedom(), this->getNrOfDegreesOfFreedom())
                            .llt()
                            .solve(m_totalJointTorques);

    m_nuDot.head(6) = baseAcceleration.coeffs();
    m_nuDot.tail(this->getNrOfDegreesOfFreedom()) = jointAcceleration;

    return true;
}

bool KinDynWrapper::forwardDynamics(Eigen::Ref<const Eigen::VectorXd> motorTorqueAfterGearbox,
                                    Eigen::Ref<const Eigen::VectorXd> frictionTorques,
                                    Eigen::Ref<const Eigen::VectorXd> tauExt,
                                    Eigen::Ref<Eigen::VectorXd> nuDot)
{
    constexpr auto errorPrefix = "[KinDynWrapper::forwardDynamics]";

    if (motorTorqueAfterGearbox.size() != (this->getNrOfDegreesOfFreedom()))
    {
        log()->error("{} The size of the parameter `motorTorquesAfterGearbox` should match the "
                     "number of joints.",
                     errorPrefix);
        return false;
    }

    if (frictionTorques.size() != (this->getNrOfDegreesOfFreedom()))
    {
        log()->error("{} The size of the parameter `frictionTorques` should match the number of "
                     "joints.",
                     errorPrefix);
        return false;
    }

    if (tauExt.size() != (6 + this->getNrOfDegreesOfFreedom()))
    {
        log()->error("{} The size of the parameter `tauExt` should match the number of joints + 6.",
                     errorPrefix);
        return false;
    }

    if (!this->generalizedBiasForces(m_generalizedBiasForces))
    {
        log()->error("{} Failed while getting the generalized bias forces.", errorPrefix);
        return false;
    }

    if (!getFreeFloatingMassMatrix(m_massMatrix))
    {
        log()->error("{} Failed while getting the mass matrix.", errorPrefix);
        return false;
    }

    m_totalBaseJointTorques.setZero();

    m_totalBaseJointTorques.tail(this->getNrOfDegreesOfFreedom()) = motorTorqueAfterGearbox - frictionTorques;

    m_totalBaseJointTorques += tauExt - m_generalizedBiasForces;

    m_nuDot = m_massMatrix.llt().solve(m_totalBaseJointTorques);

    nuDot = m_nuDot;

    return true;
}

Eigen::Ref<const Eigen::VectorXd> KinDynWrapper::getNuDot()
{
    return m_nuDot;
}
