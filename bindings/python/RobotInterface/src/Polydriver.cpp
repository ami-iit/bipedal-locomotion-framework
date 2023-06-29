/**
 * @file RobotInterface.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/RobotInterface/YarpHelper.h>

#include <BipedalLocomotion/bindings/RobotInterface/Polydriver.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotInterface
{

void CreatePolyDriver(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace yarp::dev;
    py::class_<PolyDriver, std::shared_ptr<PolyDriver>>(module, "PolyDriver");
}

void CreatePolyDriverDescriptor(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::RobotInterface;
    using namespace BipedalLocomotion::ParametersHandler;

    py::class_<PolyDriverDescriptor>(module, "PolyDriverDescriptor")
        .def(py::init())
        .def_readwrite("key", &PolyDriverDescriptor::key)
        .def_readwrite("poly", &PolyDriverDescriptor::poly)
        .def("is_valid", &PolyDriverDescriptor::isValid);

    module.def(
        "construct_remote_control_board_remapper",
        [](std::shared_ptr<const IParametersHandler> handler) -> PolyDriverDescriptor {
            return constructRemoteControlBoardRemapper(handler);
        },
        py::arg("handler"));

    module.def(
        "construct_generic_sensor_client",
        [](std::shared_ptr<const IParametersHandler> handler) -> PolyDriverDescriptor {
            return constructGenericSensorClient(handler);
        },
        py::arg("handler"));

    module.def(
        "construct_multiple_analog_sensors_client",
        [](std::shared_ptr<const IParametersHandler> handler) -> PolyDriverDescriptor {
            return constructMultipleAnalogSensorsClient(handler);
        },
        py::arg("handler"));

    module.def(
        "construct_multiple_analog_sensors_remapper",
        [](std::shared_ptr<const IParametersHandler> handler) -> PolyDriverDescriptor {
            return constructMultipleAnalogSensorsRemapper(handler);
        },
        py::arg("handler"));

    module.def(
        "construct_RGBD_sensor_client",
        [](std::shared_ptr<const IParametersHandler> handler) -> PolyDriverDescriptor {
            return constructRDGBSensorClient(handler);
        },
        py::arg("handler"));
}

} // namespace RobotInterface
} // namespace bindings
} // namespace BipedalLocomotion
