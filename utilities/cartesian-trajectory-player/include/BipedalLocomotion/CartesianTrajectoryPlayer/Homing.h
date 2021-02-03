/**
 * @file Homing.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_DCM_WALKING_PIPELINE_DCM_WALKING_HOMING_H
#define BIPEDAL_LOCOMOTION_DCM_WALKING_PIPELINE_DCM_WALKING_HOMING_H

#include <Eigen/Dense>
#include <manif/manif.h>

#include <WalkingControllers/WholeBodyControllers/InverseKinematics.h>

namespace BipedalLocomotion
{
namespace CartesianTrajectoryPlayer
{

class Homing
{

    WalkingControllers::WalkingIK m_IKSolver;
    manif::SE3d m_leftFoot;
    manif::SE3d m_rightFoot;
    Eigen::Vector3d m_com;
    iDynTree::VectorDynSize m_qDesired;

public:
    void setDesiredCoMPosition(Eigen::Ref<const Eigen::Vector3d> com);

    void setDesiredFeetTransform(const manif::SE3d& leftFootTransform,
                                 const manif::SE3d& rightFootTransform);

    bool setRobotState(Eigen::Ref<const Eigen::VectorXd> jointPos);

    bool initialize(yarp::os::Searchable& options,
                    const iDynTree::Model& model,
                    const std::vector<std::string>& jointList);

    bool solveIK();

    Eigen::Ref<const Eigen::VectorXd> getJointPos() const;
};

} // namespace DCMWalkingPipeline
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_DCM_WALKING_PIPELINE_DCM_WALKING_PIPELINE_H
