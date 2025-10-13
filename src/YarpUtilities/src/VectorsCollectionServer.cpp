/**
 * @file VectorsCollectionServer.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

#include <vector>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>

#include <atomic>
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
    std::atomic<int> metadataVersion{-1}; /**< Version of the metadata. */
    std::unordered_set<std::string> setOfKeys; /**< Set of keys. */
    std::vector<std::vector<std::string>> versionHistory; /**< History of the metadata versions. */
    std::vector<std::string> pendingKeys; /**< Keys collected since last finalize. */
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

    // if the metadata was already finalized, reset the flag and allow to populate new metadata
    if (m_pimpl->isMetadataFinalized)
    {
        m_pimpl->isMetadataFinalized = false;
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

    // add the key to the pending keys
    m_pimpl->pendingKeys.push_back(key);

    return true;
}

bool VectorsCollectionServer::finalizeMetadata()
{
    // check if the metadata is empty
    if (m_pimpl->metadata.vectors.empty())
    {
        log()->error("[VectorsCollectionServer::finalizeMetadata] The metadata is empty.");
        return false;
    }

    // check if there are new keys to finalize
    if (m_pimpl->pendingKeys.empty())
    {
        log()->error("[VectorsCollectionServer::finalizeMetadata] No new keys to finalize.");
        return false;
    }

    // Generate new metadata version
    std::sort(m_pimpl->pendingKeys.begin(), m_pimpl->pendingKeys.end());
    m_pimpl->versionHistory.push_back(m_pimpl->pendingKeys);

    // increment the metadata version
    const int newVersion = m_pimpl->metadataVersion.fetch_add(1) + 1;
    m_pimpl->metadata.version = newVersion;

    // clear the pending keys
    m_pimpl->pendingKeys.clear();

    // set the metadata as finalized
    m_pimpl->isMetadataFinalized = true;

    return true;
}

void VectorsCollectionServer::prepareData()
{
    m_pimpl->collection = m_pimpl->port.prepare();

    if (m_pimpl->collection.has_value())
    {
        m_pimpl->collection.value().get().version = m_pimpl->metadataVersion.load();
    }
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

    VectorsCollectionMetadata result = m_pimpl->metadata;
    result.version = m_pimpl->metadataVersion.load();
    return result;
}

VectorsCollectionMetadata
VectorsCollectionServer::getMetadataIncremental(const std::int32_t fromVersion)
{
    if (!m_pimpl->isMetadataFinalized)
    {
        return VectorsCollectionMetadata();
    }

    const int currentVersion = m_pimpl->metadataVersion.load();
    VectorsCollectionMetadata result;
    result.version = currentVersion;

    // Only -1 returns the full metadata
    if (fromVersion == -1)
    {
        result.vectors = m_pimpl->metadata.vectors;
        return result;
    }

    // Any other negative value triggers a warning and returns empty result
    if (fromVersion < -1)
    {
        log()->warn("[VectorsCollectionServer::getMetadataIncremental] The provided fromVersion "
                    "({}) is negative and not equal to -1. Returning empty metadata.",
                    fromVersion);
        return result;
    }

    if (fromVersion >= currentVersion)
    {
        log()->warn("[VectorsCollectionServer::getMetadataIncremental] The provided fromVersion "
                    "({}) is greater than or equal to the current version ({}). Returning empty "
                    "metadata.",
                    fromVersion,
                    currentVersion);
        return result;
    }

    // Collect the incremental metadata from the specified version
    const int firstVersion = fromVersion + 1;
    for (int version = firstVersion;
         version <= currentVersion && version < static_cast<int>(m_pimpl->versionHistory.size());
         ++version)
    {
        for (const auto& key : m_pimpl->versionHistory[version])
        {
            const auto it = m_pimpl->metadata.vectors.find(key);
            if (it != m_pimpl->metadata.vectors.end())
            {
                result.vectors[key] = it->second;
            }
        }
    }

    return result;
}
