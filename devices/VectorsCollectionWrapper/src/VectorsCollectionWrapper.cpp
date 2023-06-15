/**
 * @copyright 2020, 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */


#include <BipedalLocomotion/VectorsCollectionWrapper.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/TextLogging/LoggerBuilder.h>
#include <BipedalLocomotion/TextLogging/YarpLogger.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>


using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion;

VectorsCollectionWrapper::VectorsCollectionWrapper(double period,
                                                   yarp::os::ShouldUseSystemClock useSystemClock)
    : yarp::os::PeriodicThread(period, useSystemClock)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());
}

VectorsCollectionWrapper::VectorsCollectionWrapper()
    : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
    // Use the yarp clock in blf
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::YarpClockFactory>());

    // the logging message are streamed using yarp
    BipedalLocomotion::TextLogging::LoggerBuilder::setFactory(
        std::make_shared<BipedalLocomotion::TextLogging::YarpLoggerFactory>());
}

VectorsCollectionWrapper::~VectorsCollectionWrapper() = default;

bool VectorsCollectionWrapper::open(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[VectorsCollectionWrapper::open]";
    auto params = std::make_shared<ParametersHandler::YarpImplementation>(config);

    double devicePeriod{0.01};
    if (params->getParameter("sampling_period_in_s", devicePeriod))
    {
        this->setPeriod(devicePeriod);
    }
    params->getParameter("port_prefix", m_portPrefix);

    std::vector<std::string> remotePortNames, remoteVarNames;
    if (!params->getParameter("remote_port_names", remotePortNames))
    {
        log()->error("{} Unable to get required parameter \"remote_port_names\".", logPrefix);
        return false;
    }

    if (!params->getParameter("remote_var_names", remoteVarNames))
    {
        log()->error("{} Unable to get required parameter \"remote_var_names\".", logPrefix);
        return false;
    }

    if (remotePortNames.size() != remoteVarNames.size())
    {
        log()->error("{} Lists \"remote_port_names\" and \"remote_var_names\""
                     "must follow same size and order.", logPrefix);
        return false;
    }

    bool ok = setupPortManager(remotePortNames, remoteVarNames, m_portPrefix, m_portManager);
    if (!ok)
    {
        log()->error("{} Unable to configure port manager.", logPrefix);
        return false;
    }

    if (!params->getParameter("output_port_name", m_wrapperPortName))
    {
        log()->error("{} Unable to get required parameter \"output_port_name\".", logPrefix);
        return false;
    }

    if (m_wrapperPortName[0] != '/')
    {
        log()->error("{} \"output_port_name\" should begin with character \'/\'.", logPrefix);
        return false;
    }

    m_wrapperPort.open(m_wrapperPortName);

    if (ok)
    {
        return start();
    }

    return ok;
}

bool VectorsCollectionWrapper::setupPortManager(const std::vector<PortName>& portNames,
                                                const std::vector<VarOutName>& varNames,
                                                const std::string& portPrefix,
                                                std::unordered_map<PortName, VectorPortData>& portManager)
{
    constexpr auto logPrefix = "[VectorsCollectionWrapper::setupPortManager]";
    std::size_t idx{0};
    for (const auto& remote : portNames)
    {
        const auto& varOutName = varNames[idx];
        std::string inPort{portPrefix + varOutName + ":i"};
        if (remote[0] != '/')
        {
            log()->error("{} Every port in \"remote_port_names\" "
                         "should begin with character \'/\'.", logPrefix);
            return false;
        }

        // open input port
        portManager[remote].inputPortName = inPort;
        portManager[remote].varName = varOutName;
        portManager[remote].port.open(inPort);
        if (!yarp::os::Network::connect(remote, inPort))
        {
            log()->error("{} Unable to connect the ports {} and {}",
                         logPrefix, remote, inPort);
            return false;
        }

        idx++;
    }
    return true;
}

void VectorsCollectionWrapper::run()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    auto& data = m_wrapperPort.prepare();
    data.vectors.clear();
    // get the data from the ports
    // wrap in VectorsCollection and send
    for (auto& [remote, portData] : m_portManager)
    {
        yarp::sig::Vector* vec{nullptr};
        vec = portData.port.read(/*shouldWait = */ false);
        if (vec)
        {
            data.vectors[portData.varName].assign(vec->data(),
                                                  vec->data() + vec->size());
        }
    }

    m_wrapperPort.write();
}


bool VectorsCollectionWrapper::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (isRunning())
    {
        stop();
    }

    return true;
}
