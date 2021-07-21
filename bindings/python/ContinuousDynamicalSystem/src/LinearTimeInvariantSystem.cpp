/**
 * @file LinearTimeInvariantSystem.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateLinearTimeInvariantSystem(pybind11::module& module)
{
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    namespace py = ::pybind11;

    py::class_<DynamicalSystem<LinearTimeInvariantSystem>,
               std::shared_ptr<DynamicalSystem<LinearTimeInvariantSystem>>>
        linearTimeInvariantSystem_base(module, "_LinearTimeInvariantSystemBase");

    py::class_<LinearTimeInvariantSystem,
               DynamicalSystem<LinearTimeInvariantSystem>,
               std::shared_ptr<LinearTimeInvariantSystem>>
        linearTimeInvariantSystem(module, "LinearTimeInvariantSystem");

    CreateDynamicalSystem<LinearTimeInvariantSystem,
                          DynamicalSystem<LinearTimeInvariantSystem>,
                          std::shared_ptr<LinearTimeInvariantSystem>>(linearTimeInvariantSystem);

    linearTimeInvariantSystem.def("set_system_matrices",
                                  &LinearTimeInvariantSystem::setSystemMatrices,
                                  py::arg("A"),
                                  py::arg("B"));
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
