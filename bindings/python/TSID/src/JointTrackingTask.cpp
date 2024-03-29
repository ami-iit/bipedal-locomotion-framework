/**
 * @file JointTrackingTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/JointTrackingTask.h>

#include <BipedalLocomotion/bindings/System/LinearTask.h>
#include <BipedalLocomotion/bindings/TSID/JointTrackingTask.h>
#include <BipedalLocomotion/bindings/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateJointTrackingTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<JointTrackingTask, std::shared_ptr<JointTrackingTask>, TSIDLinearTask>( //
        module,
        "JointTrackingTask")
        .def(py::init())
        .def("set_kin_dyn",
             BipedalLocomotion::bindings::System::setKinDyn<JointTrackingTask>,
             py::arg("kin_dyn"))
        .def("set_set_point",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>>(&JointTrackingTask::setSetPoint),
             py::arg("joint_position"))
        .def("set_set_point",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>, Eigen::Ref<const Eigen::VectorXd>>(
                 &JointTrackingTask::setSetPoint),
             py::arg("joint_position"),
             py::arg("joint_velocity"))
        .def("set_set_point",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>,
                               Eigen::Ref<const Eigen::VectorXd>,
                               Eigen::Ref<const Eigen::VectorXd>>(&JointTrackingTask::setSetPoint),
             py::arg("joint_position"),
             py::arg("joint_velocity"),
             py::arg("joint_acceleration"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
