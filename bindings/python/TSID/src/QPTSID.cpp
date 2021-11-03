/**
 * @file QPInverseKinematics.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/TSID/QPFixedBaseTSID.h>
#include <BipedalLocomotion/TSID/QPTSID.h>
#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/bindings/TSID/QPTSID.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace TSID
{

void CreateQPTSID(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<QPTSID, TaskSpaceInverseDynamics>(module, "QPTSID")
        .def(py::init())
        .def("__repr__", &QPTSID::toString);
}

void CreateQPFixedBaseTSID(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::TSID;

    py::class_<QPFixedBaseTSID, QPTSID>(module, "QPFixedBaseTSID")
        .def(py::init())
        .def("set_kin_dyn", &QPFixedBaseTSID::setKinDyn, py::arg("kin_dyn"));
}

} // namespace TSID
} // namespace bindings
} // namespace BipedalLocomotion
