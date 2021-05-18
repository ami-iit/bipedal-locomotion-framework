/**
 * @file QPInverseKinematics.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/bindings/IK/QPInverseKinematics.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateQPInverseKinematics(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<QPInverseKinematics, IntegrationBasedIK>(module, "QPInverseKinematics")
        .def(py::init())
        .def(
            "initialize",
            [](QPInverseKinematics& impl,
               std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                   handler) -> bool { return impl.initialize(handler); },
            py::arg("handler"))
        .def("add_task",
             &QPInverseKinematics::addTask,
             py::arg("task"),
             py::arg("taskName"),
             py::arg("priority"),
             py::arg("weight") = Eigen::VectorXd())
        .def("get_task_names", &QPInverseKinematics::getTaskNames)
        .def("finalize", &QPInverseKinematics::finalize, py::arg("handler"))
        .def("advance", &QPInverseKinematics::advance)
        .def("get_output", &QPInverseKinematics::getOutput)
        .def("is_output_valid", &QPInverseKinematics::isOutputValid);
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
