/**
 * @file BaseDynamicsTask.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/FeasibleContactWrenchTask.h>
#include <BipedalLocomotion/bindings/TSID/FeasibleContactWrenchTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateFeasibleContactWrenchTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<FeasibleContactWrenchTask,
               std::shared_ptr<FeasibleContactWrenchTask>, //
               TSIDLinearTask>(module, "FeasibleContactWrenchTask")
        .def(py::init())
        .def("set_kin_dyn", &FeasibleContactWrenchTask::setKinDyn, py::arg("kin_dyn"))
        .def("set_contact_active",
             &FeasibleContactWrenchTask::setContactActive,
             py::arg("is_active"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
