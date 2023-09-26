namespace yarp BipedalLocomotion.YarpUtilities

struct VectorsCollection
{
    1: map<string, list<double>> vectors;
}

struct VectorsCollectionMetadata
{
    1: map<string, list<string>> vectors;
}

service VectorsCollectionMetadataService
{
    /**
     * Read the sensor metadata necessary to interpret the data.
     */
    VectorsCollectionMetadata getMetadata();
}
