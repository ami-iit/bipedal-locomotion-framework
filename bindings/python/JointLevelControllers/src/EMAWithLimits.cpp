/**
 * @file EMAWithLimits.cpp
 * @authors Giulio Romualdi
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/JointLevelControllers/EMAWithLimits.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <BipedalLocomotion/bindings/JointLevelControllers/EMAWithLimits.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace JointLevelControllers
{

void CreateEMAWithLimits(pybind11::module& module)
{
    namespace py = ::pybind11;

    BipedalLocomotion::bindings::System::CreateAdvanceable<::Eigen::VectorXd,
                                                           ::Eigen::VectorXd>( //
        module,
        "EMAWithLimits");

    py::class_<::BipedalLocomotion::JointLevelControllers::EMAWithLimits,
               ::BipedalLocomotion::System::Advanceable<::Eigen::VectorXd,
                                                        ::Eigen::VectorXd>> //
        (module, "EMAWithLimits")
            .def(py::init())
            .def("reset",
                 &::BipedalLocomotion::JointLevelControllers::EMAWithLimits::reset,
                 py::arg("initial_condition"));
}
} // namespace JointLevelControllers
} // namespace bindings
} // namespace BipedalLocomotion
