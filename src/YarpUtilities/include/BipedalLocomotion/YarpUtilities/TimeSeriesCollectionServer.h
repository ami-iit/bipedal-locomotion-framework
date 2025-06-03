/**
 * @file TimeSeriesServer.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_YARP_UTILITIES_TIME_SERIES_COLLECTION_SERVER_H
#define BIPEDAL_LOCOMOTION_YARP_UTILITIES_TIME_SERIES_COLLECTION_SERVER_H

// std
#include <vector>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/YarpUtilities/TimeSeriesCollection.h>
#include <BipedalLocomotion/YarpUtilities/TimeSeriesCollectionMetadata.h>
#include <BipedalLocomotion/YarpUtilities/TimeSeriesCollectionMetadataService.h>

#include <iDynTree/Span.h>

namespace BipedalLocomotion
{

namespace YarpUtilities
{

/**
 * TimeSeriesCollectionServer is a class that extends the TimeSeriesMetadataService and
 * allows to send a collection of TimeSeries to a client.
 * @note The TimeSeriesCollectionServer allows to send a TimeSeries to a client. You can use the
 * TimeSeriesCollectionServer as follows
 * @code{.cpp}
 * auto handler = std::make_shared<ParametersHandler::StdImplementation>();
 * // fill the handler with the parameters shown in the documentation
 *
 * TimeSeriesCollectionServer server;
 * server.initialize(handler);
 * // The metadata should be populated before the data is sent.
 * // The metadata are available to the client only after the finalizeMetadata function is called.
 * server.populateMetadata("key1", {"metadata1", "metadata2", "metadata3"});
 * server.populateMetadata("key2", {"metadata4", "metadata5", "metadata6"});
 * server.finalizeMetadata();
 *
 * using namespace std::chrono_literals;
 *
 * // prepare the data
 * server.prepareData();
 * server.clearData(); // mandatory
 * std::chrono::nanoseconds relativeTime = 0ns; // relative time in nanoseconds
 * server.populateData("key1", {1.0, 2.0, 3.0}, relativeTime);
 * server.populateData("key2", {4.0, 5.0, 6.0}, relativeTime);
 *
 * std::chrono::nanoseconds relativeTime1 = -1s; // relative time in seconds
 * server.populateData("key1", {1.0, 2.0, 3.0}, relativeTime1);
 * server.populateData("key2", {4.0, 5.0, 6.0}, relativeTime1);
 *
 * server.sendData();
 * @endcode
 */
class TimeSeriesCollectionServer : public TimeSeriesCollectionMetadataService
{
public:
    /**
     * Constructor.
     */
    TimeSeriesCollectionServer();

    /**
     * Destructor.
     */
    virtual ~TimeSeriesCollectionServer();

    // clang-format off
    /**
     * Initialize the server.
     * @param handler pointer to the parameters handler.
     * @note The following parameters are required:
     * |   Parameter Name   |   Type   |                                   Description                                   |
     * |:------------------:|:--------:|:-------------------------------------------------------------------------------:|
     * |        remote      |  string  |                          Name of the port that will be created.                 |
     * @return true if the server has been initialized successfully, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);
    // clang-format on

    /**
     * Populate the metadata.
     * @param key key of the metadata.
     * @param metadata metadata.
     * @return true if the metadata has been populated successfully, false otherwise.
     */
    bool populateMetadata(const std::string& key, const std::vector<std::string>& metadata);

    /**
     * Finalize the metadata.
     * @return true if the metadata has been finalized successfully, false otherwise.
     * @note this function should be called after the metadata has been populated. It opens the
     * ports.
     */
    bool finalizeMetadata();

    /**
     * Set the data.
     * @param key key of the data.
     * @param data data.
     * @param relativeTime relative time of the data in nanoseconds.
     * @return true if the data has been set successfully, false otherwise.
     * @note this function should be called after the metadata has been finalized and after the
     * prepareData function has been called.
     */
    bool populateData(const std::string& key,
                      const iDynTree::Span<const double>& data,
                      const std::chrono::nanoseconds& relativeTime);

    /**
     * Get the metadata.
     * @return the metadata.
     * @note if the metadata is not ready, an empty TimeSeriesCollectionMetadata is returned.
     */
    TimeSeriesCollectionMetadata getMetadata() override;

    /**
     * Check if the metadata is ready.
     * @return true if the metadata is ready, false otherwise.
     */
    bool areMetadataReady() override;

    /**
     * Prepare the data.
     * @note this function should be called before the data is populated.
     */
    void prepareData();

    /**
     * Send the data filled with populateData
     * @param forceStrict If this is true, wait until any previous sends are complete. If false, the
     * current object will not be sent on connections that are currently busy.
     */
    void sendData(bool forceStrict = false);

    /**
     * Deallocates the memory held by the internal buffer used to store data.
     * @note If you have previously populated the object using populateData and intend
     * to reuse it without reallocating memory, you may skip calling this function. Otherwise, use
     * TimeSeriesCollectionServer::clearData to deallocate the memory.
     * @note Note that this function only clears the data and does not affect the metadata.
     * @return true if the data has been cleared successfully, false otherwise.
     */
    bool clearData();

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};
} // namespace YarpUtilities
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_YARP_UTILITIES_TIME_SERIES_COLLECTION_SERVER_H
