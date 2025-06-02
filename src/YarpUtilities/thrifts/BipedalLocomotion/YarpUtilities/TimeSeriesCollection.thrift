namespace yarp BipedalLocomotion.YarpUtilities

struct TimeSeriesCollection
{
    1: map<string, list<list<double>>> timeseries;
    2: map<string, list<i64>> relativeTimestampsInNanoSeconds;
}

struct TimeSeriesMetadata
{
    1: map<string, list<string>> vectors;
}

service TimeSeriesMetadataService
{
    /**
     * Read the sensor metadata necessary to interpret the data.
     */
    TimeSeriesMetadata getMetadata();

    /**
     * Check if the metadata is ready.
     */
    bool areMetadataReady();
}
