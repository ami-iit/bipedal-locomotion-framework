/**
 * @file Homing.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */


#include <BipedalLocomotion/CartesianTrajectoryPlayer/Homing.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/Model/Model.h>

using namespace BipedalLocomotion::CartesianTrajectoryPlayer;

bool Homing::initialize(yarp::os::Searchable& options,
                        const iDynTree::Model& model,
                        const std::vector<std::string>& jointList)
{
    m_qDesired.resize(model.getNrOfDOFs());
    return m_IKSolver.initialize(options, model, jointList);
}

void Homing::setDesiredCoMPosition(Eigen::Ref<const Eigen::Vector3d> com)
{
    m_com = com;
}

void Homing::setDesiredFeetTransform(const manif::SE3d& leftFootTransform,
                                     const manif::SE3d& rightFootTransform)
{
    m_leftFoot = leftFootTransform;
    m_rightFoot = rightFootTransform;
}

bool Homing::setRobotState(Eigen::Ref<const Eigen::VectorXd> jointPos)
{
    iDynTree::VectorDynSize jointPos_iDynTree(jointPos.size());
    iDynTree::toEigen(jointPos_iDynTree) = jointPos;

    if (!m_IKSolver.setFullModelFeedBack(jointPos_iDynTree))
    {
        std::cerr << "[Homing::setRobotState] Error while setting the feedback to the IK "
                     "solver."
                  << std::endl;
        return false;
    }

    return true;
}

bool Homing::solveIK()
{
    iDynTree::Position desiredCoMPosition;
    iDynTree::toEigen(desiredCoMPosition) = m_com;

    if (m_IKSolver.usingAdditionalRotationTarget())
    {
        // torso orientation
        // TODO remove me
        const double yawLeft = m_leftFoot.asSO3().quat().toRotationMatrix().eulerAngles(2, 1, 0)(2);
        const double yawRight = m_rightFoot.asSO3().quat().toRotationMatrix().eulerAngles(2, 1, 0)(2);

        const double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                          std::cos(yawLeft) + std::cos(yawRight));

        iDynTree::Rotation yawRotation = iDynTree::Rotation::RotZ(meanYaw);

        if (!m_IKSolver.updateIntertiaToWorldFrameRotation(yawRotation.inverse()))
        {
            std::cerr << "[Homing::solveIK] Error updating the inertia to world frame rotation."
                      << std::endl;
            return false;
        }
    }

    iDynTree::Transform left(iDynTree::MatrixView<const double>(m_leftFoot.isometry().matrix()));
    iDynTree::Transform right(iDynTree::MatrixView<const double>(m_rightFoot.isometry().matrix()));

    if (!m_IKSolver.computeIK(left, right, desiredCoMPosition, m_qDesired))
    {
        std::cerr << "[Homing::solveIK] Inverse Kinematics failed while computing the "
                     "initial position."
                  << std::endl;
        return false;
    }

    return  true;
}

Eigen::Ref<const Eigen::VectorXd> Homing::getJointPos() const
{
    return iDynTree::toEigen(m_qDesired);
}
