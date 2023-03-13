/**
 * @file SubModelKinDynWrapper.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <Eigen/QR>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixView.h>
#include <iDynTree/Core/Span.h>

// BLF
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace blf = BipedalLocomotion;
namespace RDE = BipedalLocomotion::Estimators::RobotDynamicsEstimator;

bool RDE::SubModelKinDynWrapper::initialize(
    RDE::SubModel& subModel, std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel)
{
    constexpr auto logPrefix = "[BipedalLocomotion::Estimators::SubModelKinDynWrapper::initialize]";

    this->m_subModel = subModel;

    // Load robot model to the kindyncomputation object
    if (!m_kinDyn.loadRobotModel(m_subModel.getModel()))
    {
        blf::log()->error("{} Unable to load the robot model.", logPrefix);
        return false;
    }

    m_numOfJoints = m_subModel.getModel().getNrOfDOFs();

    m_baseFrame = m_kinDyn.getFloatingBase();

    m_baseVelocity.setZero();

    m_jointPositionModel.resize(kinDynFullModel->getNrOfDegreesOfFreedom());
    m_jointVelocityModel.resize(kinDynFullModel->getNrOfDegreesOfFreedom());

    m_qj.resize(m_numOfJoints);
    m_dqj.resize(m_numOfJoints);

    m_FTranspose.resize(m_numOfJoints, 6);
    m_H.resize(m_numOfJoints, m_numOfJoints);
    m_genBiasJointTorques.resize(m_numOfJoints);
    m_pseudoInverseH.resize(m_numOfJoints, m_numOfJoints);
    m_FTBaseAcc.resize(m_numOfJoints);
    m_totalTorques.resize(m_numOfJoints);

    // Initialize attributes with correct sizes
    m_massMatrix.resize(6 + m_subModel.getModel().getNrOfDOFs(),
                        6 + m_subModel.getModel().getNrOfDOFs());
    m_massMatrix.setZero();

    m_genForces.resize(6 + m_subModel.getModel().getNrOfDOFs());
    m_genForces.setZero();

    for (int idx = 0; idx < m_subModel.getFTList().size(); idx++)
    {
        Eigen::MatrixXd tempJacobian(6, 6 + m_numOfJoints);
        tempJacobian.setZero();
        m_jacFTList[m_subModel.getFTList()[idx].name] = std::move(tempJacobian);
    }

    for (int idx = 0; idx < m_subModel.getAccelerometerList().size(); idx++)
    {
        Eigen::MatrixXd tempJacobian(6, 6 + m_numOfJoints);
        tempJacobian.setZero();
        m_jacAccList[m_subModel.getAccelerometerList()[idx].name] = std::move(tempJacobian);

        Eigen::VectorXd tempAcceleration(6);
        tempAcceleration.setZero();
        m_dJnuList[m_subModel.getAccelerometerList()[idx].name] = std::move(tempAcceleration);

        manif::SO3d rotMatrix;
        rotMatrix.coeffs().setZero();
        m_accRworldList[m_subModel.getAccelerometerList()[idx].name] = std::move(rotMatrix);
    }

    for (int idx = 0; idx < m_subModel.getGyroscopeList().size(); idx++)
    {
        Eigen::MatrixXd tempJacobian(6, 6 + m_numOfJoints);
        tempJacobian.setZero();
        m_jacGyroList[m_subModel.getGyroscopeList()[idx].name] = std::move(tempJacobian);
    }

    for (int idx = 0; idx < m_subModel.getExternalContactList().size(); idx++)
    {
        Eigen::MatrixXd tempJacobian(6, 6 + m_numOfJoints);
        tempJacobian.setZero();
        m_jacContactList[m_subModel.getExternalContactList()[idx]] = std::move(tempJacobian);
    }

    return true;
}

bool RDE::SubModelKinDynWrapper::updateKinDynState(
    std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel)
{
    constexpr auto logPrefix = "[BipedalLocomotion::Estimators::SubModelKinDynWrapper::"
                               "updateKinDynState]";

    // Get world transform for the base frame
    m_worldTBase = blf::Conversions::toManifPose(kinDynFullModel->getWorldTransform(m_baseFrame));

    // Get base velocity
    m_baseVelocity = blf::Conversions::toManifTwist(kinDynFullModel->getFrameVel(m_baseFrame));

    // Get position and velocity of the joints in the full model
    kinDynFullModel->getRobotState(m_jointPositionModel, m_jointVelocityModel, m_worldGravity);

    // Get positions and velocities of joints of the sub model
    for (int idx = 0; idx < m_subModel.getJointMapping().size(); idx++)
    {
        m_qj(idx) = m_jointPositionModel[m_subModel.getJointMapping()[idx]];
        m_dqj(idx) = m_jointVelocityModel[m_subModel.getJointMapping()[idx]];
    }

    if (!m_kinDyn.setRobotState(m_worldTBase.transform(),
                                m_qj,
                                iDynTree::make_span(m_baseVelocity.data(),
                                                    manif::SE3d::Tangent::DoF),
                                iDynTree::Span<double>(m_dqj.data(), m_dqj.size()),
                                m_worldGravity))
    {
        blf::log()->error("{} Unable to set the robot state.", logPrefix);
        return false;
    }

    return updateDynamicsVariableState();
}

bool RDE::SubModelKinDynWrapper::updateDynamicsVariableState()
{
    constexpr auto logPrefix = "[BipedalLocomotion::Estimators::SubModelKinDynWrapper::"
                               "updateDynamicsVariableState]";

    if (!m_kinDyn.getFreeFloatingMassMatrix(m_massMatrix))
    {
        blf::log()->error("{} Unable to get the mass matrix of the sub-model.", logPrefix);
        return false;
    }

    if (!m_kinDyn.generalizedBiasForces(
            iDynTree::make_span(m_genForces.data(), m_genForces.size())))
    {
        blf::log()->error("{} Unable to get the generalized bias forces of the sub-model.",
                          logPrefix);
        return false;
    }

    // The jacobians are computed wrt the base of the submodel
    // (floating base with known pos and vel thanks to the kinematics)
    for (int idx = 0; idx < m_subModel.getFTList().size(); idx++)
    {
        if (!m_kinDyn.getFrameFreeFloatingJacobian(m_subModel.getFTList()[idx].frame,
                                                   m_jacFTList[m_subModel.getFTList()[idx].name]))
        {
            blf::log()->error("{} Unable to get the compute the free floating jacobian of the "
                              "frame {}.",
                              logPrefix,
                              m_subModel.getFTList()[idx].frame);
            return false;
        }
    }

    // Update accelerometer jacobians, dJnu, rotMatrix
    for (int idx = 0; idx < m_subModel.getAccelerometerList().size(); idx++)
    {
        // Update jacobian
        if (!m_kinDyn
                 .getFrameFreeFloatingJacobian(m_subModel.getAccelerometerList()[idx].frame,
                                               m_jacAccList[m_subModel.getAccelerometerList()[idx]
                                                                .name]))
        {
            blf::log()->error("{} Unable to get the compute the free floating jacobian of the "
                              "frame {}.",
                              logPrefix,
                              m_subModel.getAccelerometerList()[idx].frame);
            return false;
        }

        // Update dJnu
        m_dJnuList[m_subModel.getAccelerometerList()[idx].name] = iDynTree::toEigen(
            m_kinDyn.getFrameBiasAcc(m_subModel.getAccelerometerList()[idx].frame));

        // Update rotMatrix
        m_accRworldList[m_subModel.getAccelerometerList()[idx].name] = blf::Conversions::toManifRot(
            m_kinDyn.getWorldTransform(m_subModel.getAccelerometerList()[idx].frame)
                .getRotation()
                .inverse());
    }

    // Update gyroscope jacobians
    for (int idx = 0; idx < m_subModel.getGyroscopeList().size(); idx++)
    {
        if (!m_kinDyn.getFrameFreeFloatingJacobian(m_subModel.getGyroscopeList()[idx].frame,
                                                   m_jacGyroList[m_subModel.getGyroscopeList()[idx]
                                                                     .name]))
        {
            blf::log()->error("{} Unable to get the compute the free floating jacobian of the "
                              "frame {}.",
                              logPrefix,
                              m_subModel.getGyroscopeList()[idx].frame);
            return false;
        }
    }

    // Update external contact jacobians
    for (int idx = 0; idx < m_subModel.getExternalContactList().size(); idx++)
    {
        if (!m_kinDyn.getFrameFreeFloatingJacobian(m_subModel.getExternalContactList()[idx],
                                                   m_jacContactList
                                                       [m_subModel.getExternalContactList()[idx]]))
        {
            blf::log()->error("{} Unable to get the compute the free floating jacobian of the "
                              "frame {}.",
                              logPrefix,
                              m_subModel.getExternalContactList()[idx]);
            return false;
        }
    }

    return true;
}

bool RDE::SubModelKinDynWrapper::inverseDynamics(Eigen::Ref<Eigen::VectorXd> motorTorqueAfterGearbox,
                                                 Eigen::Ref<Eigen::VectorXd> frictionTorques,
                                                 Eigen::Ref<Eigen::VectorXd> tauExt,
                                                 Eigen::Ref<Eigen::VectorXd> baseAcceleration,
                                                 Eigen::Ref<Eigen::VectorXd> jointAcceleration)
{
    constexpr auto logPrefix = "[SubModelKinDynWrapper::inverseDynamics]";

    if (m_subModel.getModel().getNrOfDOFs() == 0)
    {
        blf::log()->error("{} Inverse dynamics is not defined for sub-models with zero degrees of freedom.",
                          logPrefix);
        return false;
    }

    if (motorTorqueAfterGearbox.size() == 0 || frictionTorques.size() == 0 || tauExt.size() == 0 || baseAcceleration.size() == 0)
    {
        blf::log()->error("{} Wrong size of input parameters.",
                          logPrefix);
        return false;
    }

    m_FTranspose = m_massMatrix.block(6, 0, m_numOfJoints, 6);
    m_H = m_massMatrix.block(6, 6, m_numOfJoints, m_numOfJoints);

    m_genBiasJointTorques = m_genForces.tail(m_numOfJoints);

    m_pseudoInverseH = m_H.completeOrthogonalDecomposition().pseudoInverse();

    m_FTBaseAcc = m_FTranspose * baseAcceleration;
    m_totalTorques
        = motorTorqueAfterGearbox - frictionTorques - m_genBiasJointTorques + tauExt + m_FTBaseAcc;

    jointAcceleration = m_pseudoInverseH * m_totalTorques;

    return true;
}

bool RDE::SubModelKinDynWrapper::getBaseAcceleration(
    std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
    manif::SE3d::Tangent& robotBaseAcceleration,
    Eigen::Ref<const Eigen::VectorXd> robotJointAcceleration,
    manif::SE3d::Tangent& subModelBaseAcceleration)
{
    constexpr auto logPrefix = "[BipedalLocomotion::Estimators::SubModelKinDynWrapper::"
                               "getBaseAcceleration]";

    if (!kinDynFullModel->getFrameAcc(m_baseFrame,
                                      iDynTree::make_span(robotBaseAcceleration.data(),
                                                          manif::SE3d::Tangent::DoF),
                                      robotJointAcceleration,
                                      iDynTree::make_span(subModelBaseAcceleration.data(),
                                                          manif::SE3d::Tangent::DoF)))
    {
        blf::log()->error("{} Unable to get the acceleration of the frame {}.",
                          logPrefix,
                          m_baseFrame);
        return false;
    }

    return true;
}

const manif::SE3d::Tangent& RDE::SubModelKinDynWrapper::getBaseVelocity(
    std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel)
{
    m_baseVelocity = blf::Conversions::toManifTwist(kinDynFullModel->getFrameVel(m_baseFrame));
    return m_baseVelocity;
}

const std::string& RDE::SubModelKinDynWrapper::getBaseFrameName() const
{
    return m_baseFrame;
}

const Eigen::Ref<const Eigen::MatrixXd> RDE::SubModelKinDynWrapper::getMassMatrix() const
{
    return m_massMatrix;
}

const Eigen::Ref<const Eigen::VectorXd> RDE::SubModelKinDynWrapper::getGeneralizedForces() const
{
    return m_genForces;
}

const Eigen::Ref<const Eigen::MatrixXd>
RDE::SubModelKinDynWrapper::getFTJacobian(const std::string& ftName) const
{
    return m_jacFTList.at(ftName);
}

const Eigen::Ref<const Eigen::MatrixXd>
RDE::SubModelKinDynWrapper::getAccelerometerJacobian(const std::string& accName) const
{
    return m_jacAccList.at(accName);
}

const Eigen::Ref<const Eigen::MatrixXd>
RDE::SubModelKinDynWrapper::getGyroscopeJacobian(const std::string& gyroName) const
{
    return m_jacGyroList.at(gyroName);
}

const Eigen::Ref<const Eigen::MatrixXd>
RDE::SubModelKinDynWrapper::getExtContactJacobian(const std::string& extContactName) const
{
    return m_jacContactList.at(extContactName);
}

const Eigen::Ref<const Eigen::VectorXd>
RDE::SubModelKinDynWrapper::getAccelerometerBiasAcceleration(const std::string& accName) const
{
    return m_dJnuList.at(accName);
}

const manif::SO3d&
RDE::SubModelKinDynWrapper::getAccelerometerRotation(const std::string& accName) const
{
    return m_accRworldList.at(accName);
}
