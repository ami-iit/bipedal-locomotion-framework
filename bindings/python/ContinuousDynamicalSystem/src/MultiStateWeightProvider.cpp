/**
 * @file MultiStateWeightProvider.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/MultiStateWeightProvider.h>
#include <BipedalLocomotion/System/WeightProvider.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/MultiStateWeightProvider.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateMultiStateWeightProvider(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;

    py::class_<MultiStateWeightProvider,
               ::BipedalLocomotion::System::WeightProvider,
               std::shared_ptr<MultiStateWeightProvider>>(module, "MultiStateWeightProvider")
        .def(py::init())
        .def("set_state", &MultiStateWeightProvider::setState, py::arg("state"));
}
} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
