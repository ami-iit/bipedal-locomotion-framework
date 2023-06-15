/**
 * @file IKLinearTask.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/KinDynComputations.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/System/LinearTask.h>

#include <BipedalLocomotion/bindings/IK/IKLinearTask.h>
#include <BipedalLocomotion/bindings/System/LinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateIKLinearTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace ::BipedalLocomotion::IK;

    py::class_<::BipedalLocomotion::IK::IKLinearTask,
               ::BipedalLocomotion::System::LinearTask,
               ::BipedalLocomotion::bindings::System::LinearTaskTrampoline<IKLinearTask>,
               std::shared_ptr<IKLinearTask>>(module, "IKLinearTask")
        .def(py::init<>())
        .def("set_kin_dyn",
             BipedalLocomotion::bindings::System::setKinDyn<IKLinearTask>,
             py::arg("kin_dyn"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
