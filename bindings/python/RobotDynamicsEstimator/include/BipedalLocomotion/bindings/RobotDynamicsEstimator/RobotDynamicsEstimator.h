/**
 * @file RobotDynamicsEstimator.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_DYNAMICS_ESTIMATOR_ROBOT_DYNAMICS_ESTIMATOR_H
#define BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_DYNAMICS_ESTIMATOR_ROBOT_DYNAMICS_ESTIMATOR_H

#include <pybind11/pybind11.h>
#include <iDynTree/KinDynComputations.h>
#include <BipedalLocomotion/bindings/type_caster/swig.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotDynamicsEstimator
{

template <class T> bool setKinDyn(T& myclass, ::pybind11::object& obj)
{
    std::shared_ptr<iDynTree::KinDynComputations>* cls
        = pybind11::detail::swig_wrapped_pointer_to_pybind<
            std::shared_ptr<iDynTree::KinDynComputations>>(obj);

    if (cls == nullptr)
    {
        throw ::pybind11::value_error("Invalid input for the function. Please provide a valid object.");
    }

    return myclass.setKinDyn(*cls);
};

void CreateRobotDynamicsEstimator(pybind11::module& module);
void CreateSubModel(pybind11::module& module);
void CreateSubModelCreator(pybind11::module& module);
void CreateKinDynWrapper(pybind11::module& module);

} // namespace RobotDynamicsEstimator
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_ROBOT_DYNAMICS_ESTIMATOR_ROBOT_DYNAMICS_ESTIMATOR_H

