/**
 * @file JointTrackingTask.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/IK/IKLinearTask.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/bindings/IK/JointTrackingTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace IK
{

void CreateJointTrackingTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::IK;

    py::class_<JointTrackingTask, std::shared_ptr<JointTrackingTask>, IKLinearTask>( //
        module,
        "JointTrackingTask")
        .def(py::init())
        .def("set_kin_dyn", &JointTrackingTask::setKinDyn, py::arg("kin_dyn"))
        .def("set_set_point",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>>(&JointTrackingTask::setSetPoint),
             py::arg("joint_position"))
        .def("set_set_point",
             py::overload_cast<Eigen::Ref<const Eigen::VectorXd>, Eigen::Ref<const Eigen::VectorXd>>(
                 &JointTrackingTask::setSetPoint),
             py::arg("joint_position"),
             py::arg("joint_velocity"));
}

} // namespace IK
} // namespace bindings
} // namespace BipedalLocomotion
