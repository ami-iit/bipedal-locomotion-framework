/**
 * @file BaseDynamicsTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/BaseDynamicsTask.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <BipedalLocomotion/bindings/System/LinearTask.h>
#include <BipedalLocomotion/bindings/TSID/BaseDynamicsTask.h>
#include <BipedalLocomotion/bindings/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateBaseDynamicsTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<BaseDynamicsTask,
               std::shared_ptr<BaseDynamicsTask>, //
               TSIDLinearTask>(module, "BaseDynamicsTask")
        .def(py::init())
        .def("set_kin_dyn",
             BipedalLocomotion::bindings::System::setKinDyn<BaseDynamicsTask>,
             py::arg("kin_dyn"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
