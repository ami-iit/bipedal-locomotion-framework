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
} // namespace YarpUtilities
} // namespace bindings
} // namespace BipedalLocomotion
