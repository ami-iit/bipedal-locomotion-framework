namespace yarp BipedalLocomotion.YarpUtilities

struct TimeSeriesCollection
{
    1: map<string, list<list<double>>> timeseries;
    2: map<string, list<double>> relativeTimestampsInSeconds;
}

struct TimeSeriesCollectionMetadata
{
    1: map<string, list<string>> vectors;
}

service TimeSeriesCollectionMetadataService
{
    /**
     * Read the sensor metadata necessary to interpret the data.
     */
    TimeSeriesCollectionMetadata getMetadata();

    /**
     * Check if the metadata is ready.
     */
    bool areMetadataReady();
}
