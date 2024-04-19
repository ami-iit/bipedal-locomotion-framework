/**
 * @file Advanceable.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/cast.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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

void CreateCommonDataStructure(pybind11::module& module)
{
    CreateInputPort<::BipedalLocomotion::System::EmptySignal>(module, "Empty");
    CreateInputPort<::Eigen::VectorXd>(module, "VectorXd");
    CreateOutputPort<::BipedalLocomotion::System::EmptySignal>(module, "Empty");
    CreateOutputPort<::Eigen::VectorXd>(module, "VectorXd");

    CreateAdvanceableImpl<::BipedalLocomotion::System::EmptySignal, ::Eigen::VectorXd>(module, "EmptyVectorXd");
    CreateAdvanceableImpl<::Eigen::VectorXd, ::BipedalLocomotion::System::EmptySignal>(module, "VectorXdEmpty");
    CreateSourceImpl<::Eigen::VectorXd>(module, "VectorXd");
    CreateSinkImpl<::Eigen::VectorXd>(module, "VectorXd");

    CreateAdvanceableImpl<::Eigen::VectorXd, ::Eigen::VectorXd>(module, "VectorXd");
}

} // namespace System
} // namespace bindings
} // namespace BipedalLocomotion
