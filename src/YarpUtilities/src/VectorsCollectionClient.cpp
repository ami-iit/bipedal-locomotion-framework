/**
 * @file VectorsCollectionClient.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionClient.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionMetadataService.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

#include <unordered_set>

using namespace BipedalLocomotion::YarpUtilities;

struct VectorsCollectionClient::Impl
{
    yarp::os::BufferedPort<VectorsCollection> port; /**< Buffered port used to communicate with
                                                            the server. */
    yarp::os::Port rpcPort; /**< RPC port used to communicate with the server. */
    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadataService rpcInterface;

    std::string localRpcPortName; /**< Name of the local rpc port. */
    std::string localPortName; /**< Name of the local port. */
    std::string remoteRpcPortName; /**< Name of the remote rpc port. */
    std::string remotePortName; /**< Name of the remote port. */
    std::string carrier; /**< Carrier used to connect the port. */

    bool isConnected{false}; /**< True if the client is connected. */
};

VectorsCollectionClient::VectorsCollectionClient()
{
    m_pimpl = std::make_unique<Impl>();
}

VectorsCollectionClient::~VectorsCollectionClient() = default;

bool VectorsCollectionClient::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[VectorsCollectionClient::initialize]";
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is nullptr.", logPrefix);
        return false;
    }

    // open the port
    std::string localPort;
    if (!ptr->getParameter("local", localPort))
    {
        log()->error("{} Unable to retrieve the port prefix.", logPrefix);
        return false;
    }

    std::string remotePort;
    if (!ptr->getParameter("remote", remotePort))
    {
        log()->error("{} Unable to retrieve the port prefix.", logPrefix);
        return false;
    }

    m_pimpl->remotePortName = remotePort + "/measures:o";
    m_pimpl->remoteRpcPortName = remotePort + "/rpc:i";

    m_pimpl->localPortName = localPort + "/measures:i";
    if (!m_pimpl->port.open(m_pimpl->localPortName))
    {
        log()->error("{} Unable to open the port named {}.", //
                     logPrefix,
                     m_pimpl->localPortName);
        return false;
    }

    // open the rpc port
    m_pimpl->localRpcPortName = localPort + "/rpc:o";
    if (!m_pimpl->rpcPort.open(m_pimpl->localRpcPortName))
    {
        log()->error("{} Unable to open the rpc port named {}.",
                     logPrefix,
                     m_pimpl->localRpcPortName);
        return false;
    }

    if (!ptr->getParameter("carrier", m_pimpl->carrier))
    {
        log()->error("{} Unable to retrieve the port prefix.", logPrefix);
        return false;
    }

    return true;
}

bool VectorsCollectionClient::disconnect()
{
    if (!m_pimpl->isConnected)
    {
        return true;
    }

    if (!yarp::os::Network::disconnect(m_pimpl->remotePortName,
                                       m_pimpl->localPortName)
        || !yarp::os::Network::disconnect(m_pimpl->localRpcPortName, //
                                          m_pimpl->remoteRpcPortName))
    {
        return false;
    }

    m_pimpl->isConnected = false;
    return true;
}

bool VectorsCollectionClient::connect()
{
    constexpr auto rpcCarrier = "tcp";
    m_pimpl->isConnected = false;

    if (!yarp::os::Network::connect(m_pimpl->remotePortName,
                                    m_pimpl->localPortName,
                                    m_pimpl->carrier)
        || !yarp::os::Network::connect(m_pimpl->localRpcPortName, //
                                       m_pimpl->remoteRpcPortName,
                                       rpcCarrier))
    {
        return false;
    }

    if (!m_pimpl->rpcInterface.yarp().attachAsClient(m_pimpl->rpcPort))
    {
        return false;
    }

    m_pimpl->isConnected = true;

    return true;
}

bool VectorsCollectionClient::getMetadata(
    BipedalLocomotion::YarpUtilities::VectorsCollectionMetadata& metadata)
{
    if (!m_pimpl->isConnected)
    {
        log()->error("[VectorsCollectionClient::getMetadata] Please call connect before asking for "
                     "the metadata");
        return false;
    }

    metadata = m_pimpl->rpcInterface.getMetadata();
    return true;
}

BipedalLocomotion::YarpUtilities::VectorsCollection*
VectorsCollectionClient::readData(bool shouldWait /*= true */)
{
    return m_pimpl->port.read(shouldWait);
}
