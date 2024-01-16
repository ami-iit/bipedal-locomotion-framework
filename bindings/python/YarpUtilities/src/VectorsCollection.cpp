/**
 * @file VectorsCollection.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionClient.h>

#include <BipedalLocomotion/bindings/YarpUtilities/BufferedPort.h>
#include <BipedalLocomotion/bindings/YarpUtilities/VectorsCollection.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace YarpUtilities
{
void CreateVectorsCollectionServer(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace ::BipedalLocomotion::YarpUtilities;

    py::class_<VectorsCollectionServer>(module, "VectorsCollectionServer")
        .def(py::init())
        .def(
            "initialize",
            [](VectorsCollectionServer& impl,
               std::shared_ptr<const ::BipedalLocomotion::ParametersHandler::IParametersHandler>
                   handler) -> bool { return impl.initialize(handler); },
            py::arg("handler"))
        .def("populate_metadata", &VectorsCollectionServer::populateMetadata)
        .def("finalize_metadata", &VectorsCollectionServer::finalizeMetadata)
        .def("clear_data", &VectorsCollectionServer::clearData)
        .def("send_data", &VectorsCollectionServer::sendData, py::arg("force_strict") = false)
        .def("populate_data",
             [](VectorsCollectionServer& impl,
                const std::string& key,
                Eigen::Ref<const Eigen::VectorXd> data) -> bool {
                 return impl.populateData(key, data);
             });
}

void CreateVectorsCollectionClient(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace ::BipedalLocomotion::YarpUtilities;

    py::class_<VectorsCollectionClient>(module, "VectorsCollectionClient")
        .def(py::init())
        .def(
            "initialize",
            [](VectorsCollectionClient& impl,
               std::shared_ptr<const ::BipedalLocomotion::ParametersHandler::IParametersHandler>
                   handler) -> bool { return impl.initialize(handler); },
            py::arg("handler"))
        .def("connect", &VectorsCollectionClient::connect)
        .def("disconnect", &VectorsCollectionClient::disconnect)
        .def("getMetadata", 
        [](VectorsCollectionClient& impl) -> std::map<std::string, std::vector<std::string>>
        {
            BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata metadata;
            impl.getMetadata(metadata);
            return metadata.vectors;
        })
        .def("readData", /*&VectorsCollectionClient::readData, py::arg("shouldWait") = true,
            py::return_value_policy::reference_internal); */
             [](VectorsCollectionClient& impl, bool shouldWait) -> std::map<std::string, std::vector<double>>
             {
                BipedalLocomotion::YarpUtilities::VectorsCollection* collection = impl.readData(shouldWait);
                return collection->vectors;
             });
}
} // namespace YarpUtilities
} // namespace bindings
} // namespace BipedalLocomotion
