/**
 * @file SO3Task.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/SO3Task.h>
#include <BipedalLocomotion/bindings/TSID/SO3Task.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateSO3Task(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<SO3Task, std::shared_ptr<SO3Task>, TSIDLinearTask>(module, "SO3Task")
        .def(py::init())
        .def("set_kin_dyn", &SO3Task::setKinDyn, py::arg("kin_dyn"))
        .def("set_set_point",
             &SO3Task::setSetPoint,
             py::arg("I_R_F"),
             py::arg("angular_velocity"),
             py::arg("angular_acceleration"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
