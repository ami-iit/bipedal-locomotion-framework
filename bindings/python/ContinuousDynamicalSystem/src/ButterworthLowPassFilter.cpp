/**
 * @file ButterworthLowPass.cpp
 * @authors Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/ContinuousDynamicalSystem/ButterworthLowPassFilter.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/ContinuousDynamicalSystem/ButterworthLowPassFilter.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace ContinuousDynamicalSystem
{

void CreateButterworthLowPassFilter(pybind11::module& module)
{
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    namespace py = ::pybind11;

    BipedalLocomotion::bindings::System::CreateAdvanceable<Eigen::VectorXd, //
                                                           Eigen::VectorXd>(module,
                                                                            "ButterworthLowPassFilter");

    py::class_<ButterworthLowPassFilter, //
               ::BipedalLocomotion::System::Advanceable<Eigen::VectorXd, //
                                                        Eigen::VectorXd>>(module,
                                                                          "ButterworthLowPassFilter")
        .def(py::init());
}

} // namespace ContinuousDynamicalSystem
} // namespace bindings
} // namespace BipedalLocomotion
