/**
 * @file DistanceTask.h
 * @authors Ehsan Ranjbari
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef DISTANCE_TASK_H
#define DISTANCE_TASK_H

#include <memory>

#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>


namespace BipedalLocomotion
{
namespace IK
{

class DistanceTask : public IKLinearTask
{

    System::VariablesHandler::VariableDescription m_robotVelocityVariable; /**< Variable
                                                                              describing the robot
                                                                              velocity (base +
                                                                              joint) */
    
    static constexpr std::size_t m_spatialVelocitySize{6}; /**< Size of the spatial velocity vector. */

    bool m_isInitialized{false}; /**< True if the task has been initialized. */
    bool m_isValid{false}; /**< True if the task is valid. */

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< Pointer to a KinDynComputations
                                                               object */

    std::size_t m_DoFs{1}; /**< DoFs associated to the task */

    Eigen::MatrixXd m_jacobian, m_relativeJacobian;
    Eigen::MatrixXd m_world_T_framePosition;
    double m_kp;
    double m_desiredDistance{0.0};

    std::string m_baseName;
    std::string m_targetFrameName;
    iDynTree::FrameIndex m_baseIndex, m_targetFrameIndex;


public:

    double m_computedDistance{0.0};

    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn) override;

    bool setVariablesHandler(const System::VariablesHandler& variablesHandler) override;

    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

    bool update() override;

    bool setDesiredDistance(double desiredDistance);

    std::size_t size() const override;

    Type type() const override;

    bool isValid() const override;
};

BLF_REGISTER_IK_TASK(DistanceTask);

} // namespace IK
} // namespace BipedalLocomotion

#endif // DISTANCE_TASK_H
