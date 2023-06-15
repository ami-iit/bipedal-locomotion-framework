/**
 * @copyright 2020,2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_VECTORS_COLLECTION_WRAPPER_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_VECTORS_COLLECTION_WRAPPER_H


#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <unordered_map>
#include <string>
#include <mutex>

namespace BipedalLocomotion
{

struct VectorPortData
{
    std::string varName;
    std::string inputPortName;
    yarp::os::BufferedPort<yarp::sig::Vector> port;
};


/**
 * VectorsCollectionWrapper class connects and reads from arbitrary vector ports on the network and
 * populates the VectorsCollection structure and streams the relevant data in a single port.
 *
 * The configuration parameters are,
 * |         Parameter    |  Required | Type               |                             Description                             |
 * |:--------------------:|:---------:|:------------------:|:-------------------------------------------------------------------:|
 * |sampling_period_in_s  |   No      |     double         | Device period in seconds with default value 0.01 seconds            |
 * |port_prefix           |   No      |     string         | Prefix of the output port with default value /vcWrapper             |
 * |remote_port_names     |   Yes     | vector of string   | List of vector ports to be wrapped (port names must start with '/').|
 * |remote_var_names      |   Yes     | vector of strings  | List  of identifiers for the wrapped ports in the same order and size as remote_var_names |
 * |output_port_name      |   Yes     |     string         | Name of the output port (port name must start with '/').            |
 */
class VectorsCollectionWrapper : public yarp::dev::DeviceDriver,
                                 public yarp::os::PeriodicThread
{
public:
    VectorsCollectionWrapper(double period,
                             yarp::os::ShouldUseSystemClock useSystemClock
                             = yarp::os::ShouldUseSystemClock::No);
    VectorsCollectionWrapper();
    ~VectorsCollectionWrapper();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual void run() final;

private:
    using PortName = std::string;
    using VarOutName = std::string;
    using BufferedVectorPort = yarp::os::BufferedPort<yarp::sig::Vector>;
    using BufferedVectorsCollectionPort = yarp::os::BufferedPort<BipedalLocomotion::YarpUtilities::VectorsCollection>;

    std::mutex m_mutex;
    std::string m_wrapperPortName; ///< name of the output port
    std::unordered_map<PortName, VectorPortData> m_portManager; ///< hash of input ports to be opened for desired remote ports
                                                                ///< PortName corresponds to the name of the remote port
                                                                ///< buffered vector port is an input port created inhouse
                                                                ///< input port names follow the naming convention as,
                                                                ///< portPrefix + varName + :i
                                                                ///< connections are made at the opening stage of the device
                                                                ///< through yarp connect remote input

    BufferedVectorsCollectionPort m_wrapperPort; ///< output port
    std::string m_portPrefix{"/vcWrapper"};

    bool setupPortManager(const std::vector<PortName>& portNames,
                          const std::vector<VarOutName>& varNames,
                          const std::string& portPrefix,
                          std::unordered_map<PortName, VectorPortData>& portManager);
};

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_VECTORS_COLLECTION_WRAPPER_H
