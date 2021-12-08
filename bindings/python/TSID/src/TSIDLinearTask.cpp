/**
 * @file SE3Task.cpp
 * @authors Paolo Maria Viceconte
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/LinearTask.h>
#include <BipedalLocomotion/TSID/TSIDLinearTask.h>

#include <BipedalLocomotion/bindings/System/LinearTask.h>
#include <BipedalLocomotion/bindings/TSID/TSIDLinearTask.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateTSIDLinearTask(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<::BipedalLocomotion::TSID::TSIDLinearTask,
               ::BipedalLocomotion::System::LinearTask,
               ::BipedalLocomotion::bindings::System::LinearTaskTrampoline<TSIDLinearTask>,
               std::shared_ptr<TSIDLinearTask>>(module, "TSIDLinearTask")
        .def(py::init<>());
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
