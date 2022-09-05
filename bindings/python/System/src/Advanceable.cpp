/**
 * @file Advanceable.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <BipedalLocomotion/bindings/System/Advanceable.h>

// this will no longer necessary after
// https://github.com/pybind/pybind11/pull/3818
// Release https://github.com/pybind/pybind11/releases/tag/v2.10.0
#if !(defined(PYBIND11_VERSION_HEX) \
      || (defined(PYBIND11_VERSION_HEX) && (PYBIND11_VERSION_HEX < 0x020A0000)))
namespace pybind11
{
namespace detail
{
template <> struct type_caster<std::monostate> : public void_caster<std::monostate>
{
};
} // namespace detail
} // namespace pybind11

#endif

namespace BipedalLocomotion
{
namespace bindings
{
namespace System
{


void CreateSharedSource(pybind11::module& module)
{
    CreateSource<Eigen::VectorXd>(module, "VectorXd");
}

void CreateSharedInputPort(pybind11::module& module)
{
    CreateInputPort<::BipedalLocomotion::System::EmptySignal>(module, "Empty");
}

void CreateSharedOutputPort(pybind11::module& module)
{
    CreateOutputPort<::BipedalLocomotion::System::EmptySignal>(module, "Empty");
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
