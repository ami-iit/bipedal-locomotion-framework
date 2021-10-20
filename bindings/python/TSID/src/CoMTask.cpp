/**
 * @file CoMTask.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/CoMTask.h>
#include <BipedalLocomotion/bindings/TSID/CoMTask.h>
#include <BipedalLocomotion/bindings/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateCoMTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<CoMTask, std::shared_ptr<CoMTask>, TSIDLinearTask>(module, "CoMTask")
        .def(py::init())
        .def("set_kin_dyn", &CoMTask::setKinDyn, py::arg("kin_dyn"))
        .def("set_set_point",
             &CoMTask::setSetPoint,
             py::arg("position"),
             py::arg("velocity"),
             py::arg("acceleration"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
