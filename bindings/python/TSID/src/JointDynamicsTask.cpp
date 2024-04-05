/**
 * @file JointDynamicsTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/JointState.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/JointDynamicsTask.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <BipedalLocomotion/bindings/System/LinearTask.h>
#include <BipedalLocomotion/bindings/TSID/JointDynamicsTask.h>
#include <BipedalLocomotion/bindings/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateJointDynamicsTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<JointDynamicsTask,
               std::shared_ptr<JointDynamicsTask>, //
               TSIDLinearTask>(module, "JointDynamicsTask")
        .def(py::init())
        .def("set_kin_dyn",
             BipedalLocomotion::bindings::System::setKinDyn<JointDynamicsTask>,
             py::arg("kin_dyn"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
