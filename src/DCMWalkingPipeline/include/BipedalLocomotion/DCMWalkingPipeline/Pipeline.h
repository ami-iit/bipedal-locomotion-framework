/**
 * @file Pipeline.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_DCM_WALKING_PIPELINE_DCM_WALKING_PIPELINE_H
#define BIPEDAL_LOCOMOTION_DCM_WALKING_PIPELINE_DCM_WALKING_PIPELINE_H

#include <memory>

#include <LieGroupControllers/ProportionalController.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/Planners/ContactPhaseList.h>
#include <BipedalLocomotion/Planners/TimeVaryingDCMPlanner.h>

#include <WalkingControllers/WholeBodyControllers/TaskBasedTorqueControl.h>

#include <iDynTree/KinDynComputations.h>

#include <iostream>
#include <fstream>

namespace BipedalLocomotion
{
namespace DCMWalkingPipeline
{

class Pipeline
{

    // TODO please remove me
    std::ofstream m_stream;

    std::shared_ptr<Planners::ContactPhaseList> m_phaseList;
    Planners::SwingFootPlanner m_leftFootPlanner;
    Planners::SwingFootPlanner m_rightFootPlanner;
    Planners::TimeVaryingDCMPlanner m_dcmPlanner;

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynComputation;

    LieGroupControllers::ProportionalControllerR3d m_dcmController;

    Eigen::Vector3d m_dcmPosition;
    Eigen::Vector3d m_gravity;

    std::unique_ptr<WalkingControllers::TaskBasedTorqueControlYARPInterface> m_torqueControl;

    bool computeSimplifedModelControl();

    bool computeWholeBodyControl();


    iDynTree::VectorDynSize m_qRegularization;
public:

    Pipeline();

    void setRegularization(Eigen::Ref<const Eigen::VectorXd> reg);

    bool setRobotModel(const iDynTree::Model& model);

    bool setRobotState(const manif::SE3d& world_T_base,
                       Eigen::Ref<const Eigen::VectorXd> jointPos,
                       const manif::SE3d::Tangent& baseVelocity,
                       Eigen::Ref<const Eigen::VectorXd> jointVel);

    bool initialize(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    // remove me
    bool initializeTorqueController(yarp::os::Searchable& options,
                                    Eigen::Ref<const Eigen::VectorXd> jointPositionUpperLimits,
                                    Eigen::Ref<const Eigen::VectorXd> jointPositionLowerLimits);

    bool advance();

    Eigen::Ref<const Eigen::VectorXd> getJointTorques() const;

    bool generateTrajectory(const manif::SE3d& baseTransform);

    const manif::SE3d& getLeftFootTransform() const;

    const manif::SE3d& getRightFootTransform() const;

    Eigen::Ref<const Eigen::Vector3d> getDCMPosition() const;

    bool computeControl();

    void close();
};

} // namespace DCMWalkingPipeline
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_DCM_WALKING_PIPELINE_DCM_WALKING_PIPELINE_H
