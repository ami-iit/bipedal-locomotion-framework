/**
 * @file bipedal_locomotion.h
 * @authors Giulio Romualdi, Diego Ferigo
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <Eigen/Core>
#include <manif/SE3.h>
#include <pybind11/pybind11.h>

namespace BipedalLocomotion::Planners
{
class Contact;
}

namespace BipedalLocomotion::bindings
{
// Custom formatter of Eigen vectors
const Eigen::IOFormat FormatEigenVector //
    (Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

// Conversions from custom classes to string
std::string ToString(const manif::SE3d& se3);
std::string ToString(const Planners::Contact& contact);

// BaseTypes.cpp
void CreateBaseTypes(pybind11::module& module);

// QuinticSpline.cpp
void CreateQuinticSpline(pybind11::module& module);

// ParametersHandler.cpp
void CreateIParameterHandler(pybind11::module& module);
void CreateStdParameterHandler(pybind11::module& module);

// Contacts.cpp
void CreateContact(pybind11::module& module);
void CreateContactList(pybind11::module& module);
void CreateContactPhase(pybind11::module& module);
void CreateContactPhaseList(pybind11::module& module);

// SwingFootPlanner.cpp
void CreateSwingFootPlanner(pybind11::module& module);

// DCMPlanner.cpp
void CreateDCMPlanner(pybind11::module& module);

// TimeVaryingDCMPlanner.cpp
void CreateTimeVaryingDCMPlanner(pybind11::module& module);
} // namespace BipedalLocomotion::bindings
