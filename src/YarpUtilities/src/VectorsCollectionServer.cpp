/**
 * @file VectorsCollectionServer.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>

#include <functional>
#include <optional>
#include <unordered_set>

using namespace BipedalLocomotion::YarpUtilities;

struct VectorsCollectionServer::Impl
{
    yarp::os::BufferedPort<VectorsCollection> port; /**< Buffered port used to communicate with
                                                            the client. */
    yarp::os::Port rpcPort; /**< RPC port used to communicate with the client. */

    VectorsCollectionMetadata metadata; /**< Metadata of the vectors collection. */

    std::atomic<bool> isMetadataFinalized{false}; /**< True if the metadata has been finalized. */
    std::unordered_set<std::string> setOfKeys; /**< Set of keys. */
    std::optional<std::reference_wrapper<VectorsCollection>> collection; /**< Reference to the
                                                                            collection. */

    /**
     * Check if the collection is valid.
     * @return True if the collection is valid.
     */
    [[nodiscard]] bool isCollectionValid() const;
};

bool VectorsCollectionServer::Impl::isCollectionValid() const
{
    return collection.has_value();
}

VectorsCollectionServer::VectorsCollectionServer()
{
    m_pimpl = std::make_unique<Impl>();
}

VectorsCollectionServer::~VectorsCollectionServer() = default;

bool VectorsCollectionServer::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[VectorsCollectionServer::initialize]";
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is nullptr.", logPrefix);
        return false;
    }

    // open the port
    std::string remote;
    if (!ptr->getParameter("remote", remote))
    {
        log()->error("{} Unable to retrieve the port remote.", logPrefix);
        return false;
    }

    const std::string portName = remote + "/measures:o";
    if (!m_pimpl->port.open(portName))
    {
        log()->error("{} Unable to open the port named {}.", logPrefix, portName);
        return false;
    }

    // open the rpc port
    const std::string rpcPortName = remote + "/rpc:i";
    if (!m_pimpl->rpcPort.open(rpcPortName))
    {
        log()->error("{} Unable to open the rpc port named {}.", logPrefix, rpcPortName);
        return false;
    }

    if (!VectorsCollectionMetadataService::yarp().attachAsServer(m_pimpl->rpcPort))
    {
        log()->error("{} Unable to attach the metadata port.", logPrefix);
        return false;
    }

    return true;
}

bool VectorsCollectionServer::populateMetadata(const std::string& key,
                                               const std::vector<std::string>& metadata)
{
    // check if the metadata has been already populated
    if (m_pimpl->isMetadataFinalized)
    {
        log()->error("[VectorsCollectionServer::populateMetadata] The metadata has been already "
                     "populated.");
        return false;
    }

    // check if the key has been already used
    if (m_pimpl->setOfKeys.find(key) != m_pimpl->setOfKeys.end())
    {
        log()->error("[VectorsCollectionServer::populateMetadata] The key {} has been already "
                     "used.",
                     key);
        return false;
    }

    // populate the metadata
    m_pimpl->metadata.vectors[key] = metadata;

    // add the key to the set of keys
    m_pimpl->setOfKeys.insert(key);

    return true;
}

bool VectorsCollectionServer::finalizeMetadata()
{
    // check if the metadata has been already finalized
    if (m_pimpl->isMetadataFinalized)
    {
        log()->error("[VectorsCollectionServer::finalizeMetadata] The metadata has been already "
                     "finalized.");
        return false;
    }

    // check if the metadata is empty
    if (m_pimpl->metadata.vectors.empty())
    {
        log()->error("[VectorsCollectionServer::finalizeMetadata] The metadata is empty.");
        return false;
    }

    // set the metadata as finalized
    m_pimpl->isMetadataFinalized = true;

    return true;
}

void VectorsCollectionServer::prepareData()
{
    m_pimpl->collection = m_pimpl->port.prepare();
}

bool VectorsCollectionServer::populateData(const std::string& key,
                                           const iDynTree::Span<const double>& data)
{
    constexpr auto logPrefix = "[VectorsCollectionServer::setData]";

    // check if the metadata has been finalized
    if (!m_pimpl->isMetadataFinalized)
    {
        log()->error("{} The metadata has not been finalized.", logPrefix);
        return false;
    }

    // check if the key exists
    if (m_pimpl->setOfKeys.find(key) == m_pimpl->setOfKeys.end())
    {
        log()->error("{} The key {} does not exist.", logPrefix, key);
        return false;
    }

    if (!m_pimpl->isCollectionValid())
    {
        log()->error("{} The data collection is not valid. Please call prepareData before "
                     "calling this function.",
                     logPrefix);
        return false;
    }

    m_pimpl->collection.value().get().vectors[key].assign(data.begin(), data.end());

    return true;
}

void VectorsCollectionServer::sendData(bool forceStrict /*= false */)
{
    m_pimpl->port.write(forceStrict);
}

bool VectorsCollectionServer::clearData()
{
    // check if the reference to the collection is valid
    if (!m_pimpl->isCollectionValid())
    {
        log()->error("[VectorsCollectionServer::clearData] The reference to the collection is "
                     "invalid. Please call prepareData before calling this function.");
        return false;
    }

    m_pimpl->collection.value().get().vectors.clear();
    return true;
}

bool VectorsCollectionServer::areMetadataReady()
{
    return m_pimpl->isMetadataFinalized;
}

VectorsCollectionMetadata VectorsCollectionServer::getMetadata()
{
    if (!m_pimpl->isMetadataFinalized)
    {
        return VectorsCollectionMetadata();
    }

    return m_pimpl->metadata;
}
