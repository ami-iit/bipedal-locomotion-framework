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
             })
        .def("prepare_data", &VectorsCollectionServer::prepareData);
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
        .def("get_metadata",
        [](VectorsCollectionClient& impl) -> BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata
        {
            BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata metadata;
            impl.getMetadata(metadata);
            return metadata;
        })
        .def("read_data",
             [](VectorsCollectionClient& impl, bool shouldWait) -> std::map<std::string, std::vector<double>>
             {
                BipedalLocomotion::YarpUtilities::VectorsCollection* collection = impl.readData(shouldWait);
                return collection->vectors;
             });
}

void CreateVectorsCollectionMetadata(pybind11::module& module)
{
    namespace py = ::pybind11;

    using namespace ::BipedalLocomotion::YarpUtilities;

    py::class_<VectorsCollectionMetadata>(module, "VectorsCollectionMetadata")
        .def(py::init())
        .def(py::init<const std::map<std::string, std::vector<std::string>>&>())
        .def("to_string", &VectorsCollectionMetadata::toString)
        .def_readwrite("vectors", &VectorsCollectionMetadata::vectors);
}
} // namespace YarpUtilities
} // namespace bindings
} // namespace BipedalLocomotion
