/**
 * @file TaskSpaceInverseDynamics.h
 * @authors Ines Sorrentino, Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TASK_SPACE_INVERSE_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_TASK_SPACE_INVERSE_DYNAMICS_H

#include <unordered_map>

#include <Eigen/Dense>
#include <manif/SE3.h>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/System/ILinearTaskSolver.h>
#include <BipedalLocomotion/System/WeightProvider.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{

namespace TSID
{

/**
 * State of the TaskSpaceInverseDynamics
 */
struct TSIDState
{
    manif::SE3d::Tangent baseAcceleration; /**< Mixed acceleration of the base */
    Eigen::VectorXd jointAccelerations; /**< Joints acceleration in rad per second per second */
    Eigen::VectorXd jointTorques; /**< Joint torques */

    /**< List of the information related to the contact wrenches */
    std::unordered_map<std::string, Contacts::ContactWrench> contactWrenches;
};

/**
 * TaskSpaceInverseDynamics implements the interface for the task space inverse
 * dynamics. Please inherit this class if you want to implement your custom Task TSID.
 * The TSIDState is a struct containing the joint acceleration, joint torques
 * and contact wrenches. The TaskSpaceInverseDynamics can be used to generate the desired joint
 * torques to be sent to the low-level torque controllers.
 */
class TaskSpaceInverseDynamics : public System::ILinearTaskSolver<TSIDLinearTask, TSIDState>
{
public:
    /**
     * Destructor.
     */
    virtual ~TaskSpaceInverseDynamics() = default;
};

/**
 * TaskSpaceInverseDynamicsProblem store all the ingredients to run an modify at runtime an
 * TaskSpaceInverseDynamics problem
 */
struct TaskSpaceInverseDynamicsProblem
{
    System::VariablesHandler variablesHandler; /**< Container of the variables considered by the
                                                  TSID problem */
    /** Map containing the weight associated to each task stored considering the task name */
    std::unordered_map<std::string, std::shared_ptr<System::WeightProvider>> weights;
    std::unique_ptr<TaskSpaceInverseDynamics> tsid; /**< Pointer to the TSID solver */

    /**
     * Check if the problem is valid.
     * @return true if the problem is valid.
     */
    [[nodiscard]] bool isValid() const;

    /**
     * Get the element of the problem from a given index.
     * @tparam index a positive number from 0 to 2.
     * @note 0 is associated to the variablesHandler, 1 to the weights and 2 to the TSID. Thanks to
     * this method the TaskSpaceInverseDynamics behaves as a tuple. I.e., the
     * [`std::tie()`](https://en.cppreference.com/w/cpp/utility/tuple/tie) or
     * [Structured binding
     * declaration](https://en.cppreference.com/w/cpp/language/structured_binding) are supported by
     * TaskSpaceInverseDynamics.
     * @note the implementation of this method was taken from
     * https://devblogs.microsoft.com/oldnewthing/20201015-00/?p=104369
     */
    template <std::size_t index> std::tuple_element_t<index, TaskSpaceInverseDynamics>& get()
    {
        if constexpr (index == 0)
        {
            return variablesHandler;
        }
        if constexpr (index == 1)
        {
            return weights;
        }
        if constexpr (index == 2)
        {
            return tsid;
        }
    }
};

} // namespace TSID
} // namespace BipedalLocomotion

namespace std
{

// The following methods are required to make ::BipedalLocomotion::TSID::TaskSpaceInverseDynamics
// behaving as a tuple. Please check here:
// https://devblogs.microsoft.com/oldnewthing/20201015-00/?p=104369
template <>
struct tuple_size<::BipedalLocomotion::TSID::TaskSpaceInverseDynamics>
    : integral_constant<size_t, 3>
{
};

template <size_t Index>
struct tuple_element<Index, ::BipedalLocomotion::TSID::TaskSpaceInverseDynamics>
    : tuple_element<
          Index,
          tuple<::BipedalLocomotion::System::VariablesHandler,
                unordered_map<string, shared_ptr<::BipedalLocomotion::System::WeightProvider>>,
                unique_ptr<::BipedalLocomotion::TSID::TaskSpaceInverseDynamics>>>
{
};

} // namespace std

#endif // BIPEDAL_LOCOMOTION_TSID_TASK_SPACE_INVERSE_DYNAMICS_H
