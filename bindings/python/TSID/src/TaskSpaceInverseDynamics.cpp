/**
 * @file TaskSpaceInverseDynamics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/TSIDLinearTask.h>
#include <BipedalLocomotion/TSID/TaskSpaceInverseDynamics.h>

#include <BipedalLocomotion/bindings/System/ILinearTaskSolver.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/TSID/TaskSpaceInverseDynamics.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateTaskSpaceInverseDynamics(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<TSIDState>(module, "TSIDState")
        .def(py::init())
        .def_readwrite("base_acceleration", &TSIDState::baseAcceleration)
        .def_readwrite("joint_accelerations", &TSIDState::jointAccelerations)
        .def_readwrite("joint_torques", &TSIDState::jointTorques)
        .def_readwrite("contact_wrenches", &TSIDState::contactWrenches);

    BipedalLocomotion::bindings::System::CreateSource<TSIDState>(module, "ILinearTaskSolverTSID");

    BipedalLocomotion::bindings::System::CreateILinearTaskSolver<TSIDLinearTask,
                                                                 TSIDState> //
        (module, "ILinearTaskSolverTSID");

    py::class_<TaskSpaceInverseDynamics, //
               ::BipedalLocomotion::System::ILinearTaskSolver<TSIDLinearTask, TSIDState>> //
        (module, "TaskSpaceInverseDynamics");
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
