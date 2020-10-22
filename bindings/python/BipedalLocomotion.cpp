/**
 * @file BipedalLocomotion.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */


#include <vector>

#include "BipedalLocomotion_GenericContainer.h"
#include "BipedalLocomotion_ParametersHandler.h"
#include "BipedalLocomotion_Planners.h"

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace
{

namespace py = ::pybind11;
PYBIND11_MODULE(pybind, m)
{
    BipedalLocomotion::bindings::BipedalLocomotionGenericContainerBindings(m);
    BipedalLocomotion::bindings::BipedalLocomotionParametersHandlerBindings(m);
    BipedalLocomotion::bindings::BipedalLocomotionPlannersBindings(m);
}

} // namespace
} // namespace BipedalLocomotion
