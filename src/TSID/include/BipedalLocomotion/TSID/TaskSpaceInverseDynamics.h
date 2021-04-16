/**
 * @file TaskSpaceInverseDynamics.h
 * @authors Ines Sorrentino, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_TASK_SPACE_INVERSE_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_TASK_SPACE_INVERSE_DYNAMICS_H

#include <functional>
#include <optional>

#include <BipedalLocomotion/System/LinearTask.h>
#include <BipedalLocomotion/System/Source.h>

#include <BipedalLocomotion/Math/Wrench.h>

#include <iDynTree/KinDynComputations.h>
#include <manif/SE3.h>

namespace BipedalLocomotion
{

namespace TSID
{

struct ContactWrench
{
    int frameIndex{-1}; /**< Frame index used to express the contact wrench */

    /** Value of the contact wrench */
    BipedalLocomotion::Math::Wrenchd wrench{BipedalLocomotion::Math::Wrenchd::Zero()};
};

/**
 * State of the TaskSpaceInverseDynamics
 */
struct TSIDState
{
    manif::SE3d::Tangent baseAcceleration; /**< Mixed acceleration of the base */
    Eigen::VectorXd jointAccelerations; /**< Joints acceleration in rad per second per second */
    Eigen::VectorXd jointTorques; /**< Joint torques */
    std::unordered_map<std::string, ContactWrench> contactWrenches; /**< List of the information
                                                                       related to the contact
                                                                       wrenches */
};

/**
 * TaskSpaceInverseDynamics implements the interface for the task sapce inverse
 * dynamics. Please inherit this class if you want to implement your custom Task TSID.
 * The TSIDState is a struct containing the joint acceleration, joint torques
 * and contact wrenches. The TaskSpaceInverseDynamics can be used to generate the desired joint
 * torques to be sent to the low-level torque controllers. Here you can find an example of the
 * TaskSpaceInverseDynamics interface. <br/> <img
 * src="https://user-images.githubusercontent.com/43743081/112606007-308f7780-8e18-11eb-875f-d8a7c4b960eb.png" width="1500">
 */
class TaskSpaceInverseDynamics : public BipedalLocomotion::System::Source<TSIDState>
{

public:
    /**
     * Add a linear task in the task space inverse dynamics
     * @param task pointer to a given linear task
     * @param priority Priority associated to the task. The lower the number the higher the
     * priority.
     * @param weight Weight associated to the task. This parameter is optional. The default value is
     * an object that does not contain any value. So is an invalid weight.
     * @return true if the task has been added to the fixed base TSID.
     */
    virtual bool addTask(std::shared_ptr<System::LinearTask> task,
                         const std::string& taskName,
                         std::size_t priority,
                         std::optional<Eigen::Ref<const Eigen::VectorXd>> weight = {})
        = 0;
    /**
     * Get a vector containing the name of the tasks.
     * @return an std::vector containing all the names associated to the tasks
     */
    virtual std::vector<std::string> getTaskNames() const = 0;

    /**
     * Finalize the TSID.
     * @param handler parameter handler.
     * @note You should call this method after you add ALL the tasks.
     * @return true in case of success, false otherwise.
     */
    virtual bool finalize(const System::VariablesHandler& handler) = 0;

    /**
     * Get a specific task
     * @param name name associated to the task.
     * @return a weak ptr associated to an existing task in the TSID. If the task does not exist a
     * nullptr is returned.
     */
    virtual std::weak_ptr<System::LinearTask> getTask(const std::string& name) const = 0;

    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @note This implementation does nothing.
     * @return True in case of success, false otherwise.
     */
    virtual bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    /**
     * Destructor.
     */
    virtual ~TaskSpaceInverseDynamics() = default;
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TSID_TASK_SPACE_INVERSE_DYNAMICS_H
