/**
 * @file RobotDynamicsEstimator.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/RobotDynamicsEstimator.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
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
        .def_readwrite("basePose", &RobotDynamicsEstimatorInput::basePose)
        .def_readwrite("baseVelocity", &RobotDynamicsEstimatorInput::baseVelocity)
        .def_readwrite("baseAcceleration", &RobotDynamicsEstimatorInput::baseAcceleration)
        .def_readwrite("jointPositions", &RobotDynamicsEstimatorInput::jointPositions)
        .def_readwrite("jointVelocities", &RobotDynamicsEstimatorInput::jointVelocities)
        .def_readwrite("motorCurrents", &RobotDynamicsEstimatorInput::motorCurrents)
        .def_readwrite("ftWrenches", &RobotDynamicsEstimatorInput::ftWrenches)
        .def_readwrite("linearAccelerations", &RobotDynamicsEstimatorInput::linearAccelerations)
        .def_readwrite("angularVelocities", &RobotDynamicsEstimatorInput::angularVelocities);

    py::class_<RobotDynamicsEstimatorOutput>(module, "RobotDynamicsEstimatorOutput")
            .def(py::init())
            .def_readwrite("ds", &RobotDynamicsEstimatorOutput::ds)
            .def_readwrite("tau_m", &RobotDynamicsEstimatorOutput::tau_m)
            .def_readwrite("tau_F", &RobotDynamicsEstimatorOutput::tau_F)
            .def_readwrite("ftWrenches", &RobotDynamicsEstimatorOutput::ftWrenches)
            .def_readwrite("ftWrenchesBiases", &RobotDynamicsEstimatorOutput::ftWrenchesBiases)
            .def_readwrite("accelerometerBiases", &RobotDynamicsEstimatorOutput::accelerometerBiases)
            .def_readwrite("gyroscopeBiases", &RobotDynamicsEstimatorOutput::gyroscopeBiases)
            .def(py::pickle([](const RobotDynamicsEstimatorOutput &output) { //__getstate__
                return py::make_tuple(output.ds,
                                      output.tau_m,
                                      output.tau_F,
                                      output.ftWrenches,
                                      output.ftWrenchesBiases,
                                      output.accelerometerBiases,
                                      output.gyroscopeBiases);
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
                      rde.accelerometerBiases = t[5].cast<std::map<std::string, Eigen::VectorXd>>();
                      rde.gyroscopeBiases = t[6].cast<std::map<std::string, Eigen::VectorXd>>();

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
                 py::arg("stateVariableHandler"),
                 py::arg("measurementVariableHandler"),
                 py::arg("kinDynFullModel"))
            .def_static("build",
                        [](std::shared_ptr<const IParametersHandler> handler,
                        py::object& obj,
                        const std::vector<SubModel>& subModelList,
                        const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList) ->
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
    py::arg("kinDynFullModel"),
    py::arg("subModelList"),
    py::arg("kinDynWrapperList"))
    .def("setInitialState",
         [](BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator& obj,
         const BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimatorOutput& state) -> bool
    {
        return obj.setInitialState(state);
    },
    py::arg("initialState"));
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
        .def("createSubModels",
             createSubModels,
             py::arg("parameterHandler"))
        .def("setModelAndSensors",
             setModelAndSensors,
             py::arg("model"),
             py::arg("sensors"))
        .def("setKinDyn",
             BipedalLocomotion::bindings::RobotDynamicsEstimator::setKinDyn<
                BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator>,
             py::arg("kinDyn"))
        .def("getNrOfSubModels",
             &BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator::getNrOfSubModels)
        .def("getSubModelList",
             &BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator::getSubModelList)
        .def("getSubModel",
             &BipedalLocomotion::Estimators::RobotDynamicsEstimator::SubModelCreator::getSubModel,
             py::arg("index"));
}

void CreateSubModelKinDynWrapper(pybind11::module& module)
{
    namespace py = ::pybind11;
    using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;
    using namespace BipedalLocomotion::System;

    py::class_<SubModelKinDynWrapper, std::shared_ptr<SubModelKinDynWrapper>>(module, "SubModelKinDynWrapper")
        .def(py::init())
        .def("setKinDyn",
             BipedalLocomotion::bindings::RobotDynamicsEstimator::setKinDyn<SubModelKinDynWrapper>,
             py::arg("kinDyn"))
        .def("initialize", &SubModelKinDynWrapper::initialize)
        .def("updateState", &SubModelKinDynWrapper::updateState)
        .def("forwardDynamics", &SubModelKinDynWrapper::forwardDynamics)
        .def("getBaseAcceleration", &SubModelKinDynWrapper::getBaseAcceleration)
        .def("getBaseVelocity", &SubModelKinDynWrapper::getBaseVelocity)
        .def("getBaseFrameName", &SubModelKinDynWrapper::getBaseFrameName)
        .def("getMassMatrix", &SubModelKinDynWrapper::getMassMatrix)
        .def("getGeneralizedForces", &SubModelKinDynWrapper::getGeneralizedForces)
        .def("getFTJacobian", &SubModelKinDynWrapper::getFTJacobian)
        .def("getAccelerometerJacobian", &SubModelKinDynWrapper::getAccelerometerJacobian)
        .def("getGyroscopeJacobian", &SubModelKinDynWrapper::getGyroscopeJacobian)
        .def("getExtContactJacobian", &SubModelKinDynWrapper::getExtContactJacobian)
        .def("getAccelerometerBiasAcceleration", &SubModelKinDynWrapper::getAccelerometerBiasAcceleration)
        .def("getAccelerometerRotation", &SubModelKinDynWrapper::getAccelerometerRotation);
}

} // namespace RobotDynamicsEstimator
} // namespace bindings
} // namespace BipedalLocomotion
