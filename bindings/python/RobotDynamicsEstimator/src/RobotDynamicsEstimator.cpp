/**
 * @file RobotDynamicsEstimator.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <iDynTree/Model.h>

#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/RobotDynamicsEstimator.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>

#include <BipedalLocomotion/bindings/RobotDynamicsEstimator/RobotDynamicsEstimator.h>
#include <BipedalLocomotion/bindings/System/Advanceable.h>
#include <BipedalLocomotion/bindings/System/VariablesHandler.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace RobotDynamicsEstimator
{

void CreateRobotDynamicsEstimator(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace BipedalLocomotion::System;
    using namespace BipedalLocomotion::ParametersHandler;
    using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;

    py::class_<RobotDynamicsEstimatorInput>(module, "RobotDynamicsEstimatorInput")
        .def(py::init())
        .def_readwrite("base_pose", &RobotDynamicsEstimatorInput::basePose)
        .def_readwrite("base_velocity", &RobotDynamicsEstimatorInput::baseVelocity)
        .def_readwrite("base_acceleration", &RobotDynamicsEstimatorInput::baseAcceleration)
        .def_readwrite("joint_positions", &RobotDynamicsEstimatorInput::jointPositions)
        .def_readwrite("joint_velocities", &RobotDynamicsEstimatorInput::jointVelocities)
        .def_readwrite("motor_currents", &RobotDynamicsEstimatorInput::motorCurrents)
        .def_readwrite("ft_wrenches", &RobotDynamicsEstimatorInput::ftWrenches)
        .def_readwrite("linear_accelerations", &RobotDynamicsEstimatorInput::linearAccelerations)
        .def_readwrite("angular_velocities", &RobotDynamicsEstimatorInput::angularVelocities)
        .def_readwrite("friction_torques", &RobotDynamicsEstimatorInput::frictionTorques);

    py::class_<RobotDynamicsEstimatorOutput>(module, "RobotDynamicsEstimatorOutput")
            .def(py::init())
            .def_readwrite("ds", &RobotDynamicsEstimatorOutput::ds)
            .def_readwrite("tau_m", &RobotDynamicsEstimatorOutput::tau_m)
            .def_readwrite("tau_F", &RobotDynamicsEstimatorOutput::tau_F)
            .def_readwrite("ft_wrenches", &RobotDynamicsEstimatorOutput::ftWrenches)
            .def_readwrite("ft_wrenches_biases", &RobotDynamicsEstimatorOutput::ftWrenchesBiases)
            .def_readwrite("linear_accelerations", &RobotDynamicsEstimatorOutput::linearAccelerations)
            .def_readwrite("accelerometer_biases", &RobotDynamicsEstimatorOutput::accelerometerBiases)
            .def_readwrite("angular_velocities", &RobotDynamicsEstimatorOutput::angularVelocities)
            .def_readwrite("gyroscope_biases", &RobotDynamicsEstimatorOutput::gyroscopeBiases)
            .def_readwrite("contact_wrenches", &RobotDynamicsEstimatorOutput::contactWrenches)
            .def(py::pickle([](const RobotDynamicsEstimatorOutput &output) { //__getstate__
                return py::make_tuple(output.ds,
                                      output.tau_m,
                                      output.tau_F,
                                      output.ftWrenches,
                                      output.ftWrenchesBiases,
                                      output.linearAccelerations,
                                      output.accelerometerBiases,
                                      output.angularVelocities,
                                      output.gyroscopeBiases,
                                      output.contactWrenches);
                },
                [](py::tuple t) { // __setstate__
                      if(t.size() != 7)
                      throw std::runtime_error("Invalid state!");
                      RobotDynamicsEstimatorOutput rde;
                      rde.ds = t[0].cast<Eigen::VectorXd>();
                      rde.tau_m = t[1].cast<Eigen::VectorXd>();
                      rde.tau_F = t[2].cast<Eigen::VectorXd>();
                      rde.ftWrenches = t[3].cast<std::map<std::string, Eigen::VectorXd>>();
                      rde.ftWrenchesBiases = t[4].cast<std::map<std::string, Eigen::VectorXd>>();
                      rde.linearAccelerations = t[5].cast<std::map<std::string, Eigen::VectorXd>>();
                      rde.accelerometerBiases = t[6].cast<std::map<std::string, Eigen::VectorXd>>();
                      rde.angularVelocities = t[7].cast<std::map<std::string, Eigen::VectorXd>>();
                      rde.gyroscopeBiases = t[8].cast<std::map<std::string, Eigen::VectorXd>>();
                      rde.contactWrenches = t[9].cast<std::map<std::string, Eigen::VectorXd>>();

                      return rde;
                 }
                 ));

    BipedalLocomotion::bindings::System::CreateAdvanceable
            <RobotDynamicsEstimatorInput,
            RobotDynamicsEstimatorOutput>(module, "RobotDynamicsEstimator");

    py::class_<BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator,
            Advanceable<RobotDynamicsEstimatorInput, RobotDynamicsEstimatorOutput>>
            (module, "RobotDynamicsEstimator")
            .def(py::init())
            .def("finalize",
                 &BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator::finalize,
                 py::arg("state_variable_handler"),
                 py::arg("measurement_variable_handler"),
                 py::arg("kindyn_full_model"))
            .def_static("build",
                        [](std::shared_ptr<const IParametersHandler> handler,
                        py::object& obj,
                        const std::vector<SubModel>& subModelList,
                        const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList) ->
            std::unique_ptr<BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator> {
        // get the kindyn computation object from the swig bindings
        std::shared_ptr<iDynTree::KinDynComputations>* cls
                = py::detail::swig_wrapped_pointer_to_pybind<
                std::shared_ptr<iDynTree::KinDynComputations>>(obj);
        if (cls == nullptr)
        {
            throw ::pybind11::value_error("Invalid input for the function. Please provide "
                                          "an iDynTree::KinDynComputations object.");
        }

        auto estimator =
                BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator::build(handler,
                                                                                                     *cls,
                                                                                                     subModelList,
                                                                                                     kinDynWrapperList);

        return estimator;
    },
    py::arg("handler"),
    py::arg("kindyn_full_model"),
    py::arg("sub_model_list"),
    py::arg("kindyn_wrapper_list"))
    .def("set_initial_state",
         [](BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator& obj,
         const BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimatorOutput& state) -> bool
    {
        return obj.setInitialState(state);
    },
    py::arg("initial_state"));
}

void CreateSubModel(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
    using namespace BipedalLocomotion::System;

    py::class_<SubModel>(module, "SubModel")
        .def(py::init());
}

void CreateSubModelCreator(pybind11::module& module)
{
    namespace py = ::pybind11;

    auto setModelAndSensors = [] (BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator& subModelCreator,
                                   ::pybind11::object& model,
                                   ::pybind11::object& sensors)
    {
        iDynTree::Model* modelPtr
            = pybind11::detail::swig_wrapped_pointer_to_pybind<
                iDynTree::Model>(model);

        if (modelPtr == nullptr)
        {
            throw ::pybind11::value_error("Invalid input for the function. Please provide an "
                                          "iDynTree::Model object.");
        }

        iDynTree::SensorsList* sensorsPtr
            = pybind11::detail::swig_wrapped_pointer_to_pybind<
                iDynTree::SensorsList>(sensors);

        if (sensorsPtr == nullptr)
        {
            throw ::pybind11::value_error("Invalid input for the function. Please provide an "
                                          "iDynTree::SensorsList object.");
        }

        subModelCreator.setModelAndSensors(*modelPtr, *sensorsPtr);
    };

    auto createSubModels = [] (BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator& subModelCreator,
                               std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>& parameterHandler)
    {
        subModelCreator.createSubModels(parameterHandler);
    };

    py::class_<BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator>(module, "SubModelCreator")
        .def(py::init())
        .def("create_sub_models",
             createSubModels,
             py::arg("parameter_handler"))
        .def("set_model_and_sensors",
             setModelAndSensors,
             py::arg("model"),
             py::arg("sensors"))
        .def("set_kindyn",
             BipedalLocomotion::bindings::RobotDynamicsEstimator::setKinDyn<
                BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator>,
             py::arg("kindyn"))
        .def("get_nr_of_sub_models",
             &BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator::getNrOfSubModels)
        .def("get_sub_model_list",
             &BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator::getSubModelList)
        .def("get_sub_model",
             &BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator::getSubModel,
             py::arg("index"));
}

void CreateKinDynWrapper(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
    using namespace BipedalLocomotion::System;

    py::class_<KinDynWrapper, std::shared_ptr<KinDynWrapper>>(module, "KinDynWrapper")
        .def(py::init())
        .def("set_model", &BipedalLocomotion::Estimators::RobotDynamicsEstimator::KinDynWrapper::setModel);
}

} // namespace RobotDynamicsEstimator
} // namespace bindings
} // namespace BipedalLocomotion
