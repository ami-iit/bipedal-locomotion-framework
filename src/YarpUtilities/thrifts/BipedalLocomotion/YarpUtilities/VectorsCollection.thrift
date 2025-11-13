namespace yarp BipedalLocomotion.YarpUtilities

struct VectorsCollection
{
    1: map<string, list<double>> vectors;
    2: i32 version;
}

struct VectorsCollectionMetadata
{
    1: map<string, list<string>> vectors;
    2: i32 version;
}

service VectorsCollectionMetadataService
{
    /**
     * Read the sensor metadata necessary to interpret the data.
     */
    VectorsCollectionMetadata getMetadata();

    /**
     * Read only the metadata introduced after the provided version.
     * Pass -1 to get the full metadata.
     */
    VectorsCollectionMetadata getMetadataIncremental(1: i32 fromVersion);

    /**
     * Check if the metadata is ready.
     */
    bool areMetadataReady();
}
