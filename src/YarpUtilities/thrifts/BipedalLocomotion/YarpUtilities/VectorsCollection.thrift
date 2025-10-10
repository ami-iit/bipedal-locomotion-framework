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
     * Check if the metadata is ready.
     */
    bool areMetadataReady();
}
