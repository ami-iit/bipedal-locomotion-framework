/**
 * @file SE3Task.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/bindings/IK/SE3Task.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateSE3Task(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<SE3Task, std::shared_ptr<SE3Task>, IKLinearTask>(module, "SE3Task")
        .def(py::init())
        .def("set_kin_dyn", &SE3Task::setKinDyn, py::arg("kin_dyn"))
        .def("set_set_point", &SE3Task::setSetPoint, py::arg("I_H_F"), py::arg("mixed_velocity"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
